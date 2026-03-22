"""Publishes LiTime LiFePO4 battery state via BLE (bleak).

Supports LiTime, PowerQueen, and Redodo batteries that share the same BMS protocol.
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


class LiTimeBatteryNode(Node):
    def __init__(self):
        super().__init__("litime_battery")

        self.declare_parameter("poll_interval_sec", 5.0)
        self.declare_parameter("ble_address", "")
        self.declare_parameter("battery_id", "battery_0")

        self._interval = self.get_parameter("poll_interval_sec").value
        self._ble_address = self.get_parameter("ble_address").value
        self._battery_id = self.get_parameter("battery_id").value

        if not BLEAK_AVAILABLE:
            self.get_logger().error(
                "bleak is not installed. Run: pip install bleak"
            )
            return

        self._pub = self.create_publisher(
            BatteryState, f"{self._battery_id}/battery", 10
        )
        self._timer = self.create_timer(self._interval, self._poll_wrapper)
        self._connected = False

        # Dedicated asyncio event loop in a background thread for bleak
        self._loop = asyncio.new_event_loop()
        self._ble_thread = threading.Thread(
            target=self._loop.run_forever, daemon=True
        )
        self._ble_thread.start()

        self.get_logger().info(
            f"LiTime battery node started (id={self._battery_id}, "
            f"address={self._ble_address or 'auto-discover'}, "
            f"interval={self._interval}s)"
        )

    def _poll_wrapper(self):
        """Schedule the async poll on the BLE thread's event loop."""
        future = asyncio.run_coroutine_threadsafe(self._poll(), self._loop)
        try:
            future.result(timeout=30.0)
        except Exception as e:
            self.get_logger().warn(f"Poll failed: {e}")

    async def _discover_device(self) -> Optional[str]:
        """Scan for a LiTime-compatible BLE device."""
        self.get_logger().info("Scanning for LiTime BMS devices...")
        devices = await BleakScanner.discover(timeout=10.0)
        for dev in devices:
            name = dev.name or ""
            if any(name.startswith(prefix) for prefix in DEVICE_PREFIXES):
                self.get_logger().info(
                    f"Found LiTime device: {name} ({dev.address})"
                )
                return dev.address
        self.get_logger().warn("No LiTime BMS device found during scan")
        return None

    async def _poll(self):
        address = self._ble_address
        if not address:
            address = await self._discover_device()
            if not address:
                return
            self._ble_address = address

        response_data = bytearray()

        def on_notify(_sender, data: bytearray):
            response_data.extend(data)

        try:
            async with BleakClient(address, timeout=10.0) as client:
                if not self._connected:
                    self.get_logger().info(f"Connected to {address}")
                    self._connected = True

                await client.start_notify(NOTIFY_UUID, on_notify)
                await client.write_gatt_char(WRITE_UUID, CMD_QUERY)
                await asyncio.sleep(2.0)
                await client.stop_notify(NOTIFY_UUID)

        except Exception as e:
            if self._connected:
                self.get_logger().warn(f"BLE connection lost: {e}")
                self._connected = False
            return

        if not response_data:
            self.get_logger().warn("No response from BMS")
            return

        parsed = parse_response(bytes(response_data))
        if parsed is None:
            self.get_logger().warn(
                f"Failed to parse BMS response ({len(response_data)} bytes)"
            )
            return

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._battery_id
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

        # Unused fields
        msg.design_capacity = float("nan")
        msg.temperature = parsed["temps"][0] if parsed["temps"] else float("nan")

        self._pub.publish(msg)
        self.get_logger().debug(
            f"Battery: {parsed['voltage']:.1f}V, {parsed['current']:.1f}A, "
            f"SoC={parsed['soc']}%, SoH={parsed['soh']}%"
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
