"""Combines every battery on the network into one aggregate battery topic.

Discovers all ``/batteries/...`` topics of type ``sensor_msgs/msg/BatteryState``,
subscribes to each of them, and republishes their combined state on
``/batteries/rover_main/state`` for consumers such as the LED strip.

Discovery is periodic, so batteries that appear after startup are picked up
automatically. The packs are assumed to be wired in parallel: voltage and
percentage are averaged, current and capacities summed.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import BatteryState


OUTPUT_TOPIC = "/batteries/rover_main/state"
TOPIC_PREFIX = "/batteries/"
BATTERY_TYPE = "sensor_msgs/msg/BatteryState"
DISCOVERY_PERIOD_SEC = 5.0

# Our own output would feed back into the aggregate. The gamepad is a controller
# battery, not rover traction power, so its charge does not belong in the sum.
EXCLUDED_TOPICS = frozenset([OUTPUT_TOPIC, "/batteries/gamepad/state"])


class BatteryAggregatorNode(Node):
    def __init__(self):
        super().__init__("battery_aggregator")

        # Latest sample per battery topic
        self._states: dict[str, BatteryState] = {}
        self._subs: dict[str, object] = {}

        # QoS
        self._qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(BatteryState, OUTPUT_TOPIC, self._qos)

        self._discover()
        self._timer = self.create_timer(DISCOVERY_PERIOD_SEC, self._discover)

        self.get_logger().info(f"Battery aggregator started, publishing {OUTPUT_TOPIC}")

    def _discover(self):
        """Subscribe to any battery topic that appeared since the last scan."""
        for topic, types in self.get_topic_names_and_types():
            if topic in self._subs or topic in EXCLUDED_TOPICS:
                continue
            if not topic.startswith(TOPIC_PREFIX) or BATTERY_TYPE not in types:
                continue

            self._subs[topic] = self.create_subscription(
                BatteryState,
                topic,
                lambda msg, t=topic: self._on_battery(t, msg),
                self._qos,
            )
            self.get_logger().info(f"Aggregating battery topic: {topic}")

    def _on_battery(self, topic: str, msg: BatteryState):
        """Store the latest sample and republish the aggregate."""
        self._states[topic] = msg
        self._pub.publish(self._combine(list(self._states.values())))

    def _combine(self, states: list[BatteryState]) -> BatteryState:
        """Combine multiple battery states into one (parallel battery config)."""
        n = len(states)
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "rover_main"

        # Parallel batteries: average voltage, sum current/capacity/charge
        msg.voltage = sum(s.voltage for s in states) / n
        msg.current = sum(s.current for s in states)
        msg.percentage = sum(s.percentage for s in states) / n
        msg.capacity = sum(s.capacity for s in states)
        msg.charge = sum(s.charge for s in states)
        msg.design_capacity = sum(s.design_capacity for s in states)
        msg.present = all(s.present for s in states)

        # Temperature: take the max (worst case)
        temps = [s.temperature for s in states if s.temperature == s.temperature]
        msg.temperature = max(temps) if temps else float("nan")

        # Status: if any is charging → charging, else if any discharging → discharging
        statuses = [s.power_supply_status for s in states]
        if BatteryState.POWER_SUPPLY_STATUS_CHARGING in statuses:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif BatteryState.POWER_SUPPLY_STATUS_DISCHARGING in statuses:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE

        # Merge cell voltages from all batteries
        for s in states:
            msg.cell_voltage.extend(s.cell_voltage)
            msg.cell_temperature.extend(s.cell_temperature)

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = BatteryAggregatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
