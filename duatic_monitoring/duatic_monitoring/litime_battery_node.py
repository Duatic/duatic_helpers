"""Publishes LiTime LiFePO4 battery state via BLE (bleak).

Supports LiTime, PowerQueen, and Redodo batteries that share the same BMS protocol.
Discovers and monitors all visible BLE batteries simultaneously.
"""

import asyncio
import struct
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

try:
    from bleak import BleakClient, BleakScanner

    BLEAK_AVAILABLE = True
except ImportError:
    BLEAK_AVAILABLE = False

SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
NOTIFY_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
WRITE_UUID = "0000ffe2-0000-1000-8000-00805f9b34fb"

CMD_QUERY = bytes([0x00, 0x00, 0x04, 0x01, 0x13, 0x55, 0xAA, 0x17])

# BLE advertisement name prefixes for auto-discovery
DEVICE_PREFIXES = ("L-", "LT-", "R-", "RO-", "P-", "PQ-")


def parse_response(data: bytes) -> Optional[dict]:
    """Parse a LiTime BMS status response frame."""
    if len(data) < 92 or data[0] != 0x00 or data[1] != 0x00:
        return None

    expected_len = data[2] + 4
    if len(data) < expected_len:
        return None

    checksum = sum(data[: expected_len - 1]) & 0xFF
    if checksum != data[expected_len - 1]:
        return None

    voltage = struct.unpack_from("<H", data, 12)[0] / 1000.0
    current = struct.unpack_from("<i", data, 48)[0] / 1000.0
    soc = struct.unpack_from("<H", data, 90)[0]
    remaining_ah = struct.unpack_from("<H", data, 62)[0] / 100.0
    capacity_ah = struct.unpack_from("<I", data, 64)[0] // 100

    temps = []
    for i in range(5):
        offset = 52 + i * 2
        if offset + 2 <= len(data) - 1:
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
    if len(data) >= 100:
        cycles = struct.unpack_from("<I", data, 96)[0]

    soh = 0
    if len(data) >= 96:
        soh = struct.unpack_from("<I", data, 92)[0]

    # Determine charge state from byte 88
    battery_state = 0
    if len(data) >= 90:
        battery_state = struct.unpack_from("<H", data, 88)[0]

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


def _sanitize_topic_name(ble_name: str) -> str:
    """Convert a BLE device name to a valid ROS topic segment."""
    return ble_name.replace(" ", "_").replace(":", "_").lower()


class _BatteryDevice:
    """Tracks one discovered BLE battery."""

    def __init__(self, address: str, ble_name: str, publisher):
        self.address = address
        self.ble_name = ble_name
        self.publisher = publisher
        self.connected = False


class LiTimeBatteryNode(Node):
    def __init__(self):
        super().__init__("litime_battery")

        self.declare_parameter("poll_interval_sec", 30.0)
        self.declare_parameter("ble_address", "")
        self.declare_parameter("scan_interval_sec", 30.0)

        self._interval = self.get_parameter("poll_interval_sec").value
        self._ble_address = self.get_parameter("ble_address").value
        self._scan_interval = self.get_parameter("scan_interval_sec").value

        if not BLEAK_AVAILABLE:
            self.get_logger().error("bleak is not installed. Run: pip install bleak")
            return

        # Multi-battery: keyed by BLE address
        self._devices: dict[str, _BatteryDevice] = {}

        # Single-battery mode when ble_address is explicitly set
        self._single_mode = bool(self._ble_address)

        if self._single_mode:
            topic_name = _sanitize_topic_name(self._ble_address)
            pub = self.create_publisher(BatteryState, f"/batteries/{topic_name}/state", 10)
            self._devices[self._ble_address] = _BatteryDevice(
                self._ble_address, self._ble_address, pub
            )

        self._timer = self.create_timer(self._interval, self._poll_wrapper)

        # Periodic rescan for new devices
        if not self._single_mode:
            self._scan_timer = self.create_timer(self._scan_interval, self._scan_wrapper)

        # Dedicated asyncio event loop in a background thread for bleak
        self._loop = asyncio.new_event_loop()
        self._ble_thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._ble_thread.start()

        mode = f"single ({self._ble_address})" if self._single_mode else "discover-all"
        self.get_logger().info(
            f"LiTime battery node started (mode={mode}, "
            f"poll={self._interval}s, scan={self._scan_interval}s)"
        )

        # Initial scan
        if not self._single_mode:
            self._scan_wrapper()

    def _scan_wrapper(self):
        """Schedule BLE scan on the BLE thread."""
        future = asyncio.run_coroutine_threadsafe(self._scan_devices(), self._loop)
        try:
            future.result(timeout=15.0)
        except Exception as e:
            self.get_logger().warn(f"BLE scan failed: {e}")

    async def _scan_devices(self):
        """Scan for all LiTime-compatible BLE devices."""
        self.get_logger().info("Scanning for LiTime BMS devices...")
        devices = await BleakScanner.discover(timeout=10.0)
        for dev in devices:
            name = dev.name or ""
            if not any(name.startswith(prefix) for prefix in DEVICE_PREFIXES):
                continue
            if dev.address in self._devices:
                continue

            topic_name = _sanitize_topic_name(name)
            pub = self.create_publisher(BatteryState, f"/batteries/{topic_name}/state", 10)
            self._devices[dev.address] = _BatteryDevice(dev.address, name, pub)
            self.get_logger().info(
                f"Discovered battery: {name} ({dev.address}) " f"→ /batteries/{topic_name}/state"
            )

        if not self._devices:
            self.get_logger().warn("No LiTime BMS devices found")

    def _poll_wrapper(self):
        """Schedule polling all batteries on the BLE thread."""
        future = asyncio.run_coroutine_threadsafe(self._poll_all(), self._loop)
        try:
            future.result(timeout=60.0)
        except Exception as e:
            self.get_logger().warn(f"Poll cycle failed: {e}")

    async def _poll_all(self):
        """Poll each discovered battery sequentially."""
        for device in list(self._devices.values()):
            await self._poll_device(device)

    async def _poll_device(self, device: _BatteryDevice):
        """Query a single battery via BLE and publish its state."""
        response_data = bytearray()

        def on_notify(_sender, data: bytearray):
            response_data.extend(data)

        try:
            async with BleakClient(device.address, timeout=10.0) as client:
                if not device.connected:
                    self.get_logger().info(f"Connected to {device.ble_name} ({device.address})")
                    device.connected = True

                await client.start_notify(NOTIFY_UUID, on_notify)
                await client.write_gatt_char(WRITE_UUID, CMD_QUERY)
                await asyncio.sleep(2.0)
                await client.stop_notify(NOTIFY_UUID)

        except Exception as e:
            if device.connected:
                self.get_logger().warn(f"BLE connection lost to {device.ble_name}: {e}")
                device.connected = False
            return

        if not response_data:
            self.get_logger().warn(f"No response from {device.ble_name}")
            return

        parsed = parse_response(bytes(response_data))
        if parsed is None:
            self.get_logger().warn(
                f"Failed to parse response from {device.ble_name} " f"({len(response_data)} bytes)"
            )
            return

        topic_name = _sanitize_topic_name(device.ble_name)

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = topic_name
        msg.voltage = parsed["voltage"]
        msg.current = -parsed["current"]  # ROS convention: negative = discharge
        msg.percentage = parsed["soc"] / 100.0
        msg.charge = parsed["remaining_ah"]
        msg.capacity = float(parsed["capacity_ah"])
        msg.present = True
        msg.cell_voltage = [float(v) for v in parsed["cell_voltages"]]
        msg.cell_temperature = [float(t) for t in parsed["temps"]]

        # Map battery_state: 0=idle, 1=charging, 4=disabled
        if parsed["battery_state"] == 1:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif parsed["current"] > 0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        msg.design_capacity = float("nan")
        msg.temperature = parsed["temps"][0] if parsed["temps"] else float("nan")

        device.publisher.publish(msg)
        self.get_logger().debug(
            f"{device.ble_name}: {parsed['voltage']:.1f}V, "
            f"{parsed['current']:.1f}A, SoC={parsed['soc']}%"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LiTimeBatteryNode()
    if not BLEAK_AVAILABLE:
        node.get_logger().fatal("Cannot start without bleak. Exiting.")
        node.destroy_node()
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._loop.call_soon_threadsafe(node._loop.stop)
        node.destroy_node()
        rclpy.shutdown()
