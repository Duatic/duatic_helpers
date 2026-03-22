"""Publishes simulated LiFePO4 battery states that drain over time.

Supports multiple batteries from a single node via the ``count`` and
``name_prefix`` parameters.  Each battery gets a random initial SoC
between 1 % and 100 %.
"""

import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import BatteryState


class MockBatteryNode(Node):
    def __init__(self):
        super().__init__("mock_battery")

        self.declare_parameter("count", 4)
        self.declare_parameter("name_prefix", "lt_batt")
        self.declare_parameter("drain_rate_percent_per_min", 0.1)
        self.declare_parameter("poll_interval_sec", 30.0)
        self.declare_parameter("voltage_full", 27.6)
        self.declare_parameter("voltage_empty", 20.0)

        count = self.get_parameter("count").value
        prefix = self.get_parameter("name_prefix").value
        self._drain_rate = self.get_parameter("drain_rate_percent_per_min").value
        self._interval = self.get_parameter("poll_interval_sec").value
        self._v_full = self.get_parameter("voltage_full").value
        self._v_empty = self.get_parameter("voltage_empty").value

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Create a publisher and random initial percentage for each battery
        self._batteries: list[tuple[str, float, object]] = []
        for i in range(count):
            name = f"{prefix}_{i + 1:02d}"
            pct = round(random.uniform(1.0, 100.0), 1)
            pub = self.create_publisher(BatteryState, f"/batteries/{name}/state", qos)
            self._batteries.append((name, pct, pub))

        names = ", ".join(f"{n} ({p:.0f}%)" for n, p, _ in self._batteries)
        self.get_logger().info(f"Mock batteries started: {names}")

        self._timer = self.create_timer(self._interval, self._publish_all)
        # Publish initial state immediately
        self._publish_all()

    def _publish_all(self):
        drain = self._drain_rate * (self._interval / 60.0)
        updated = []
        for name, pct, pub in self._batteries:
            pct = max(0.0, pct - drain)
            updated.append((name, pct, pub))
            self._publish_one(name, pct, pub)
        self._batteries = updated

    def _publish_one(self, name: str, percentage: float, pub):
        frac = percentage / 100.0
        voltage = self._v_empty + frac * (self._v_full - self._v_empty)

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = name
        msg.voltage = voltage
        msg.current = -2.5 if percentage > 0 else 0.0
        msg.percentage = frac
        msg.charge = frac * 100.0
        msg.capacity = 100.0
        msg.design_capacity = 100.0
        msg.present = True
        msg.temperature = 25.0

        if percentage <= 0:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE

        pub.publish(msg)
        self.get_logger().debug(f"Mock battery '{name}': {percentage:.1f}%, {voltage:.1f}V")


def main(args=None):
    rclpy.init(args=args)
    node = MockBatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
