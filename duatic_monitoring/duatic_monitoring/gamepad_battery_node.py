"""Publishes Bluetooth gamepad battery state from /sys/class/power_supply/."""

from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class GamepadBatteryNode(Node):
    def __init__(self):
        super().__init__("gamepad_battery")

        self.declare_parameter("poll_interval_sec", 10.0)
        self.declare_parameter("power_supply_path", "")

        interval = self.get_parameter("poll_interval_sec").value
        self._configured_path = self.get_parameter("power_supply_path").value

        self._pub = self.create_publisher(BatteryState, "gamepad/battery", 10)
        self._timer = self.create_timer(interval, self._poll)
        self._last_path: str | None = None

        self.get_logger().info(f"Polling gamepad battery every {interval}s")

    def _find_gamepad_battery(self) -> Path | None:
        """Auto-discover a gamepad battery in /sys/class/power_supply/."""
        if self._configured_path:
            p = Path(self._configured_path)
            return p if p.exists() else None

        base = Path("/sys/class/power_supply")
        if not base.exists():
            return None

        # Look for known gamepad battery patterns
        patterns = ["ps-controller-battery-*", "hid-*-battery", "sony_controller_battery_*"]
        for pattern in patterns:
            matches = list(base.glob(pattern))
            if matches:
                return matches[0]
        return None

    def _poll(self):
        path = self._find_gamepad_battery()
        if path is None:
            if self._last_path is not None:
                self.get_logger().warn("Gamepad battery no longer detected")
                self._last_path = None
            return

        if str(path) != self._last_path:
            self.get_logger().info(f"Found gamepad battery: {path.name}")
            self._last_path = str(path)

        try:
            capacity = int((path / "capacity").read_text().strip())
            status_text = (path / "status").read_text().strip()
        except (FileNotFoundError, ValueError) as e:
            self.get_logger().warn(f"Failed to read battery: {e}")
            return

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gamepad"
        msg.percentage = capacity / 100.0
        msg.present = True

        status_map = {
            "Charging": BatteryState.POWER_SUPPLY_STATUS_CHARGING,
            "Discharging": BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
            "Full": BatteryState.POWER_SUPPLY_STATUS_FULL,
            "Not charging": BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING,
        }
        msg.power_supply_status = status_map.get(
            status_text, BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        )
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN

        # Mark unused fields as NaN per REP-0199
        msg.voltage = float("nan")
        msg.current = float("nan")
        msg.charge = float("nan")
        msg.capacity = float("nan")
        msg.design_capacity = float("nan")
        msg.temperature = float("nan")

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GamepadBatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
