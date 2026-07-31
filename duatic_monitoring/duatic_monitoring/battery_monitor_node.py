"""Publishes the combined state of the rover's battery pack.

Reads every configured battery over BLE (bleak) and publishes their combined state on
``/batteries/rover_main/state``.

Only the batteries listed in the ``battery_ids`` parameter are ever connected to.

Startup is all or nothing: one scan resolves the configured names to BLE addresses, and if
any of them is not advertising, the node reports what it did see and exits rather than
running with part of the pack missing.

Set ``mock_battery_count`` instead to simulate a pack of draining batteries, for working on
consumers of the aggregate without the hardware.

The packs are assumed to be wired in parallel: voltage and percentage are averaged, current
and capacities summed.
"""

import asyncio
import concurrent.futures
import random
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

OUTPUT_TOPIC = "/batteries/duarover/state"

# Shape of a simulated battery. Not parameters: they exist only to give consumers of the
# aggregate plausible numbers to react to.
MOCK_NAME_PREFIX = "lt_batt"
MOCK_DRAIN_PERCENT_PER_MIN = 0.1
MOCK_VOLTAGE_FULL = 58.4
MOCK_VOLTAGE_EMPTY = 44.0
MOCK_CURRENT = -2.5
MOCK_TEMPERATURE = 25.0
MOCK_CAPACITY_AH = 100.0


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


class _BatteryDevice:
    """One battery in the pack: how to reach it, how its last poll went, what it last read."""

    def __init__(self, battery_id: str):
        self.battery_id = battery_id
        # Filled in by the startup scan; still None means it was not advertising then.
        self.address: Optional[str] = None
        # None until the first poll, so that the first failure is logged like any other.
        self.last_poll_ok: Optional[bool] = None
        # Kept across cycles: a battery that fails one poll keeps contributing its previous
        # reading, so one BLE hiccup does not make the aggregate dip.
        self.last_state: Optional[BatteryState] = None
        # Mock mode only: simulated state of charge, drained on every tick.
        self.percentage = 0.0


class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__("battery_monitor")

        # An empty default cannot infer its type, hence the one empty string, filtered out below.
        self.declare_parameter("battery_ids", [""])
        self.declare_parameter("poll_interval_sec", 30.0)
        self.declare_parameter("mock_battery_count", 0)

        configured = self.get_parameter("battery_ids").value
        # dict keys drop duplicates while keeping the configured order.
        battery_ids = list(dict.fromkeys(bid for bid in configured if bid))

        self._interval = self.get_parameter("poll_interval_sec").value
        mock_count = self.get_parameter("mock_battery_count").value
        self._mock = mock_count > 0

        # Set before any early return below, so `ready` and `shutdown` can always be used.
        self._devices: dict[str, _BatteryDevice] = {}
        self._loop = None

        # Use latched QoS, so a consumer that starts late still gets the last known state.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(BatteryState, OUTPUT_TOPIC, qos)

        if self._mock:
            self._start_mock(mock_count, battery_ids)
        else:
            self._start_real(battery_ids)

    @property
    def ready(self) -> bool:
        """Whether there is a pack to read and, on real hardware, every battery was found.

        All or nothing: a configured battery that is missing is reported at startup rather
        than publishing an aggregate that silently omits part of the pack.
        """
        if not self._devices:
            return False
        return self._mock or all(d.address is not None for d in self._devices.values())

    def shutdown(self):
        """Stop the BLE thread. Safe to call even if startup failed part-way through."""
        if self._loop is None:
            return
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._ble_thread.join(timeout=5.0)
        if not self._ble_thread.is_alive():
            self._loop.close()

    # --- Aggregate -----------------------------------------------------------------------

    def _publish_aggregate(self):
        """Publish the combined state of every battery that has produced a reading.

        A battery that has never been read is left out entirely; until at least one has been
        read there is nothing to publish.
        """
        states = [d.last_state for d in self._devices.values() if d.last_state is not None]
        if not states:
            return

        n = len(states)
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""

        # Parallel batteries: average voltage, sum current/capacity/charge.
        msg.voltage = sum(s.voltage for s in states) / n
        msg.current = sum(s.current for s in states)
        msg.percentage = sum(s.percentage for s in states) / n
        msg.capacity = sum(s.capacity for s in states)
        msg.charge = sum(s.charge for s in states)
        msg.design_capacity = sum(s.design_capacity for s in states)
        msg.present = all(s.present for s in states)

        # Temperature: take the max (worst case), ignoring the NaN of an unpopulated sensor.
        temps = [s.temperature for s in states if s.temperature == s.temperature]
        msg.temperature = max(temps) if temps else float("nan")

        # Status: if any is charging → charging, else if any discharging → discharging.
        statuses = [s.power_supply_status for s in states]
        if BatteryState.POWER_SUPPLY_STATUS_CHARGING in statuses:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif BatteryState.POWER_SUPPLY_STATUS_DISCHARGING in statuses:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE

        for s in states:
            msg.cell_voltage.extend(s.cell_voltage)
            msg.cell_temperature.extend(s.cell_temperature)

        self._pub.publish(msg)

    # --- Mock batteries ------------------------------------------------------------------

    def _start_mock(self, count: int, battery_ids: list):
        """Set up simulated batteries instead of touching BLE at all."""
        if battery_ids:
            self.get_logger().warn(
                f"mock_battery_count is {count}, so the {len(battery_ids)} configured "
                "battery_ids are ignored and no battery is connected to."
            )

        for i in range(count):
            device = _BatteryDevice(f"{MOCK_NAME_PREFIX}_{i + 1:02d}")
            device.percentage = round(random.uniform(1.0, 100.0), 1)
            self._devices[device.battery_id] = device

        shown = ", ".join(f"{d.battery_id} ({d.percentage:.0f}%)" for d in self._devices.values())
        self.get_logger().info(f"Mock batteries started: {shown}")

        self._timer = self.create_timer(self._interval, self._mock_tick)
        self._mock_tick()

    def _mock_tick(self):
        """Drain every simulated battery by one interval's worth and publish the aggregate."""
        drain = MOCK_DRAIN_PERCENT_PER_MIN * (self._interval / 60.0)
        for device in self._devices.values():
            device.percentage = max(0.0, device.percentage - drain)
            device.last_state = self._mock_state(device)
        self._publish_aggregate()

    def _mock_state(self, device: _BatteryDevice) -> BatteryState:
        """Build the state a simulated battery would report at its current charge."""
        frac = device.percentage / 100.0

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = device.battery_id
        msg.voltage = MOCK_VOLTAGE_EMPTY + frac * (MOCK_VOLTAGE_FULL - MOCK_VOLTAGE_EMPTY)
        msg.current = MOCK_CURRENT if device.percentage > 0 else 0.0
        msg.percentage = frac
        msg.charge = frac * MOCK_CAPACITY_AH
        msg.capacity = MOCK_CAPACITY_AH
        msg.design_capacity = MOCK_CAPACITY_AH
        msg.present = True
        msg.temperature = MOCK_TEMPERATURE

        if device.percentage > 0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        return msg

    # --- Real batteries over BLE ---------------------------------------------------------

    def _start_real(self, battery_ids: list):
        """Resolve every configured battery's BLE address and start polling them."""
        if not battery_ids:
            self.get_logger().error(
                "No batteries configured. Set the 'battery_ids' parameter to the BLE names "
                "printed on the batteries, e.g. ['L-24100BNB1000123'], or set "
                "'mock_battery_count' to run without hardware. Exiting."
            )
            return

        if not BLEAK_AVAILABLE:
            self.get_logger().error(
                "bleak is not installed, so no battery can be read. Install python3-bleak, "
                "or set 'mock_battery_count' to run without hardware. Exiting."
            )
            return

        for battery_id in battery_ids:
            self._devices[battery_id] = _BatteryDevice(battery_id)

        # Dedicated asyncio event loop in a background thread for bleak
        self._loop = asyncio.new_event_loop()
        self._ble_lock = asyncio.Lock()
        self._ble_thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._ble_thread.start()

        self._run_ble(self._resolve_addresses(), SCAN_DURATION_SEC + 5.0, "BLE scan")

        # _resolve_addresses has already reported which ones are missing.
        if not self.ready:
            return

        self._timer = self.create_timer(self._interval, self._poll_wrapper)
        self.get_logger().info(
            f"Battery monitor started. Configured batteries: {', '.join(self._devices)}"
        )
        self._poll_wrapper()

    def _poll_wrapper(self):
        """Poll every battery on the BLE thread, then publish one aggregate for the cycle.

        Publishing happens here rather than in the coroutine so that it stays on the ROS
        timer thread.
        """
        per_device = CONNECT_TIMEOUT_SEC + RESPONSE_TIMEOUT_SEC
        self._run_ble(self._poll_all(), len(self._devices) * per_device + 5.0, "Poll cycle")
        self._publish_aggregate()

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

        Addresses are fixed in hardware, so polling connects by address from here on and never
        needs to scan again.
        """
        async with self._ble_lock:
            devices = await BleakScanner.discover(timeout=SCAN_DURATION_SEC)

        advertised = {dev.name: dev.address for dev in devices if dev.name}
        for device in self._devices.values():
            # Exact advertised name only: an ID that differs in case, or that is a prefix of
            # some other device's name, is a different battery and must not be connected to.
            device.address = advertised.get(device.battery_id)
            if device.address is not None:
                self.get_logger().info(f"Found battery {device.battery_id} at {device.address}")

        missing = [d.battery_id for d in self._devices.values() if d.address is None]
        if missing:
            # List advertised names in sorted order so the log is deterministic and easier to read.
            seen = ", ".join(f"{name} ({addr})" for name, addr in sorted(advertised.items()))
            self.get_logger().error(
                f"Configured batteries not found: {', '.join(missing)}. "
                f"Advertised BLE names seen: {seen or 'none'}. Exiting."
            )

    async def _poll_all(self):
        """Poll each battery in turn, one BLE connection at a time."""
        if self._ble_lock.locked():
            self.get_logger().warn("A BLE cycle is still running, skipping this poll")
            return

        async with self._ble_lock:
            for device in list(self._devices.values()):
                if device.address is not None:
                    await self._poll_device(device)

    async def _poll_device(self, device: _BatteryDevice):
        """Query one battery over BLE and record its state for the next aggregate."""
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

        device.last_state = self._build_msg(device, parsed)

    def _report_failure(self, device: _BatteryDevice, message: str):
        """Log a failed poll once per outage, including the very first attempt."""
        if device.last_poll_ok is not False:
            self.get_logger().warn(message)
        device.last_poll_ok = False

    def _build_msg(self, device: _BatteryDevice, parsed: dict) -> BatteryState:
        """Turn a parsed BMS frame into a BatteryState message."""
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = device.battery_id
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

    node = BatteryMonitorNode()
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
