"""Publishes LiTime LiFePO4 battery state via BLE (bleak).

Supports LiTime, PowerQueen, and Redodo batteries that share the same BMS protocol.

Only the batteries listed in the ``battery_ids`` parameter are ever connected to. A battery
ID is its advertised BLE name, which is printed on the battery itself, e.g.
"L-24100BNB1000123". With no IDs configured there is nothing to do, so the node says so and
exits instead of scanning.

Startup is all or nothing: one scan resolves the configured names to BLE addresses, and if
any of them is not advertising, the node reports what it did see and exits rather than
running with part of the pack missing.
"""

import asyncio
import concurrent.futures
import struct
import threading
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import BatteryState

try:
    from bleak import BleakClient, BleakScanner

    BLEAK_AVAILABLE = True
except ImportError:
    BLEAK_AVAILABLE = False

NOTIFY_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
WRITE_UUID = "0000ffe2-0000-1000-8000-00805f9b34fb"

CMD_QUERY = bytes([0x00, 0x00, 0x04, 0x01, 0x13, 0x55, 0xAA, 0x17])

# Shortest frame every mandatory field still fits inside; see parse_response.
MIN_FRAME_LEN = 92

SCAN_DURATION_SEC = 10.0
CONNECT_TIMEOUT_SEC = 10.0
RESPONSE_TIMEOUT_SEC = 3.0


def parse_response(data: bytes) -> Optional[dict]:
    """Parse a LiTime BMS status response frame.

    ``data`` can hold more than one frame: the caller concatenates every notification chunk
    it receives. Only the frame starting at offset 0 is parsed, and each field is read within
    that frame's declared length, so a trailing partial frame cannot bleed into the result.
    """
    if len(data) < MIN_FRAME_LEN or data[0] != 0x00 or data[1] != 0x00:
        return None

    # Byte 2 declares the payload length; the 4 framing bytes are not counted in it.
    frame_len = data[2] + 4
    if frame_len < MIN_FRAME_LEN or len(data) < frame_len:
        return None

    checksum = sum(data[: frame_len - 1]) & 0xFF
    if checksum != data[frame_len - 1]:
        return None

    voltage = struct.unpack_from("<H", data, 12)[0] / 1000.0
    current = struct.unpack_from("<i", data, 48)[0] / 1000.0
    soc = struct.unpack_from("<H", data, 90)[0]
    remaining_ah = struct.unpack_from("<H", data, 62)[0] / 100.0
    capacity_ah = struct.unpack_from("<I", data, 64)[0] / 100.0

    # Unpopulated sensors read as 0 and are dropped, so this is a list of the readings that
    # exist, not one entry per sensor. Only the first is used as the pack temperature.
    temps = []
    for i in range(5):
        offset = 52 + i * 2
        if offset + 2 <= frame_len:
            t = struct.unpack_from("<h", data, offset)[0]
            if t != 0 or i == 0:
                temps.append(float(t))

    cell_voltages = []
    for i in range(16):
        offset = 16 + i * 2
        cv = struct.unpack_from("<H", data, offset)[0] / 1000.0
        if cv > 0:
            cell_voltages.append(cv)

    cycles = 0
    if frame_len >= 100:
        cycles = struct.unpack_from("<I", data, 96)[0]

    soh = 0
    if frame_len >= 96:
        soh = struct.unpack_from("<I", data, 92)[0]

    battery_state = struct.unpack_from("<H", data, 88)[0]

    # soh and cycles are parsed for completeness; BatteryState has no field to carry them.
    return {
        "voltage": voltage,
        "current": current,
        "soc": soc,
        "soh": soh,
        "remaining_ah": remaining_ah,
        "capacity_ah": capacity_ah,
        "temps": temps,
        "cell_voltages": cell_voltages,
        "cycles": cycles,
        "battery_state": battery_state,
    }


def sanitize_topic_name(ble_name: str) -> str:
    """Convert a BLE device name to a valid ROS topic segment."""
    return ble_name.replace(" ", "_").replace(":", "_").replace("-", "_").lower()


def resolve_addresses(battery_ids, advertised: dict) -> dict:
    """Pick the configured batteries out of a scan result, keyed by battery ID.

    Matching is on the exact advertised name: an ID that differs only in case, or that is a
    prefix of some other device's name, is a different battery and must not be connected to.
    """
    return {
        battery_id: advertised[battery_id] for battery_id in battery_ids if battery_id in advertised
    }


class _BatteryDevice:
    """One configured battery: where to reach it and how its last poll went."""

    def __init__(self, battery_id: str, publisher):
        self.battery_id = battery_id
        self.publisher = publisher
        # Filled in by the startup scan; still None means it was not advertising then.
        self.address: Optional[str] = None
        # None until the first poll, so that the first failure is logged like any other.
        self.last_poll_ok: Optional[bool] = None


class LiTimeBatteryNode(Node):
    def __init__(self):
        super().__init__("litime_battery")

        self.declare_parameter("battery_ids", [""])
        none_configured = rclpy.Parameter("battery_ids", rclpy.Parameter.Type.STRING_ARRAY, [])
        configured = self.get_parameter_or("battery_ids", none_configured).value
        # dict keys drop duplicates while keeping the configured order.
        self.battery_ids = list(dict.fromkeys(bid for bid in configured if bid))

        # Set before the early return below, so `ready` can always be read.
        self._devices: dict[str, _BatteryDevice] = {}

        # Exit if no batteries are defined
        if not self.battery_ids:
            self.get_logger().error(
                "No batteries configured, Set the 'battery_ids' "
                "parameter to the BLE names printed on the batteries, e.g. "
                "['L-24100BNB1000123']."
            )
            return

        # Use latched QoS
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Set up one publisher per battery
        for battery_id in self.battery_ids:
            topic_name = sanitize_topic_name(battery_id)
            publisher = self.create_publisher(BatteryState, f"/batteries/{topic_name}/state", qos)
            self._devices[battery_id] = _BatteryDevice(battery_id, publisher)

        # Dedicated asyncio event loop in a background thread for bleak
        self._loop = asyncio.new_event_loop()
        self._ble_lock = asyncio.Lock()
        self._ble_thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._ble_thread.start()

        # Look up each configured battery's BLE MAC address
        self._run_ble(self._resolve_addresses(), SCAN_DURATION_SEC + 5.0, "BLE scan")

        # _resolve_addresses has already reported which ones are missing.
        if not self.ready:
            return

        self._poll_timer = self.create_timer(30.0, self._poll_wrapper)
        self.get_logger().info(
            f"LiTime battery node started. Configured batteries: {', '.join(self._devices)}"
        )
        self._poll_wrapper()

    @property
    def ready(self) -> bool:
        """Whether there are batteries to poll and every one of them was found.

        All or nothing: a configured battery that is missing is reported at startup rather
        than leaving a topic that never publishes.
        """
        return bool(self._devices) and all(
            device.address is not None for device in self._devices.values()
        )

    def shutdown(self):
        """Stop the BLE thread. Safe to call even if startup failed part-way through."""
        loop = getattr(self, "_loop", None)
        if loop is None:
            return
        loop.call_soon_threadsafe(loop.stop)
        self._ble_thread.join(timeout=5.0)
        if not self._ble_thread.is_alive():
            loop.close()

    def _poll_wrapper(self):
        """Poll every configured battery on the BLE thread."""
        per_device = CONNECT_TIMEOUT_SEC + RESPONSE_TIMEOUT_SEC
        self._run_ble(self._poll_all(), len(self._devices) * per_device + 5.0, "Poll cycle")

    def _run_ble(self, coro, timeout: float, what: str):
        """Await a BLE coroutine from a ROS timer callback."""
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        try:
            future.result(timeout=timeout)
        except concurrent.futures.TimeoutError:
            future.cancel()
            self.get_logger().warn(f"{what} timed out after {timeout:.0f}s and was cancelled")
        except Exception as e:
            self.get_logger().warn(f"{what} failed: {e}")

    async def _resolve_addresses(self):
        """Look up each configured battery's BLE address.

        Addresses are fixed in hardware, so polling connects by address from here on and never needs to scan again.
        """
        async with self._ble_lock:
            devices = await BleakScanner.discover(timeout=SCAN_DURATION_SEC)

        advertised = {dev.name: dev.address for dev in devices if dev.name}
        for battery_id, address in resolve_addresses(self._devices, advertised).items():
            device = self._devices[battery_id]
            device.address = address
            self.get_logger().info(
                f"Found battery {battery_id} at {address} → {device.publisher.topic_name}"
            )

        missing = [d.battery_id for d in self._devices.values() if d.address is None]
        if missing:
            # List advertised names in sorted order so the log is deterministic and easier to read.
            seen = ", ".join(f"{name} ({addr})" for name, addr in sorted(advertised.items()))
            self.get_logger().error(
                f"Configured batteries not found: {', '.join(missing)}. "
                f"Advertised BLE names seen: {seen or 'none'}. Exiting."
            )

    async def _poll_all(self):
        """Poll each configured battery in turn, one BLE connection at a time."""
        if self._ble_lock.locked():
            self.get_logger().warn("A BLE cycle is still running, skipping this poll")
            return

        async with self._ble_lock:
            for device in list(self._devices.values()):
                if device.address is not None:
                    await self._poll_device(device)

    async def _poll_device(self, device: _BatteryDevice):
        """Query one battery over BLE and publish its state."""
        response = bytearray()
        complete = asyncio.Event()

        def on_notify(_sender, data: bytearray):
            response.extend(data)
            # The reply arrives in several chunks. Stop waiting as soon as they add up to a
            # frame that parses, rather than always sitting out the full timeout.
            if parse_response(bytes(response)) is not None:
                complete.set()

        try:
            async with BleakClient(device.address, timeout=CONNECT_TIMEOUT_SEC) as client:
                await client.start_notify(NOTIFY_UUID, on_notify)
                await client.write_gatt_char(WRITE_UUID, CMD_QUERY)
                try:
                    await asyncio.wait_for(complete.wait(), timeout=RESPONSE_TIMEOUT_SEC)
                except asyncio.TimeoutError:
                    pass  # Reported below, along with however much did arrive.
                await client.stop_notify(NOTIFY_UUID)
        except Exception as e:
            self._report_failure(device, f"BLE connection to {device.battery_id} failed: {e}")
            return

        parsed = parse_response(bytes(response))
        if parsed is None:
            self._report_failure(
                device,
                f"No parseable response from {device.battery_id} ({len(response)} bytes)",
            )
            return

        if device.last_poll_ok is not True:
            self.get_logger().info(f"Reading battery {device.battery_id} at {device.address}")
        device.last_poll_ok = True

        device.publisher.publish(self._build_msg(device, parsed))

    def _report_failure(self, device: _BatteryDevice, message: str):
        """Log a failed poll once per outage, including the very first attempt."""
        if device.last_poll_ok is not False:
            self.get_logger().warn(message)
        device.last_poll_ok = False

    def _build_msg(self, device: _BatteryDevice, parsed: dict) -> BatteryState:
        """Turn a parsed BMS frame into a BatteryState message."""
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = sanitize_topic_name(device.battery_id)
        msg.voltage = parsed["voltage"]
        # The BMS reports discharge as positive current; BatteryState wants it negative.
        msg.current = -parsed["current"]
        msg.percentage = parsed["soc"] / 100.0
        msg.charge = parsed["remaining_ah"]
        msg.capacity = parsed["capacity_ah"]
        msg.design_capacity = float("nan")  # Not reported by this BMS.
        msg.present = True
        msg.cell_voltage = [float(v) for v in parsed["cell_voltages"]]
        msg.cell_temperature = [float(t) for t in parsed["temps"]]
        msg.temperature = parsed["temps"][0] if parsed["temps"] else float("nan")

        # The BMS's own charge flag takes precedence over the sign of the current.
        if parsed["battery_state"] == 1:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif parsed["current"] > 0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        return msg


def main(args=None):
    rclpy.init(args=args)

    node = LiTimeBatteryNode()
    try:
        if node.ready:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
