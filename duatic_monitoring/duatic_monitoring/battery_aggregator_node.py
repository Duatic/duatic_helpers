"""Aggregates selected battery states into a combined rover battery topic.

Subscribes to all /batteries/*/state topics, filters by the active_batteries
parameter, and publishes a combined BatteryState on /batteries/rover_main/state.

The active battery selection is persisted to a YAML config file so it survives
restarts. The UI can update the selection at runtime via the set_parameters service.
"""

import os
from pathlib import Path
from typing import Optional

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import BatteryState


DEFAULT_CONFIG_PATH = "~/.config/duatic/battery_config.yaml"
BATTERY_TOPIC_PREFIX = "/batteries/"
BATTERY_TOPIC_SUFFIX = "/state"
EXCLUDED_TOPICS = frozenset(["rover_main", "gamepad"])


class BatteryAggregatorNode(Node):
    def __init__(self):
        super().__init__("battery_aggregator")

        self.declare_parameter("config_file", DEFAULT_CONFIG_PATH)
        self.declare_parameter("active_batteries", rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter("poll_interval_sec", 5.0)

        self._config_path = Path(os.path.expanduser(self.get_parameter("config_file").value))
        self._interval = self.get_parameter("poll_interval_sec").value

        # Load active batteries: parameter > config file > empty (auto-select)
        self._active: list[str] = self._load_active_batteries()

        # Discovered battery states keyed by battery name
        self._battery_states: dict[str, BatteryState] = {}
        self._battery_subs: dict[str, object] = {}

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(BatteryState, "/batteries/rover_main/state", qos)

        # Timer for topic discovery and publishing
        self._timer = self.create_timer(self._interval, self._tick)

        # Initial discovery after a short delay (let publishers register first)
        self._init_timer = self.create_timer(2.0, self._tick_once)

        # React to parameter changes from UI
        self.add_on_set_parameters_callback(self._on_param_change)

        active_str = ", ".join(self._active) if self._active else "auto-select"
        self.get_logger().info(
            f"Battery aggregator started. Active: [{active_str}], " f"config: {self._config_path}"
        )

    def _load_active_batteries(self) -> list[str]:
        """Load active batteries from parameter or config file."""
        try:
            param_val = self.get_parameter("active_batteries").value
            if param_val:
                return list(param_val)
        except rclpy.exceptions.ParameterUninitializedException:
            pass

        if self._config_path.exists():
            try:
                data = yaml.safe_load(self._config_path.read_text())
                if data and "active_batteries" in data:
                    batteries = data["active_batteries"]
                    self.get_logger().info(f"Loaded active batteries from config: {batteries}")
                    return list(batteries)
            except Exception as e:
                self.get_logger().warn(f"Failed to read config file: {e}")

        return []

    def _save_config(self):
        """Persist current active battery selection to config file."""
        try:
            self._config_path.parent.mkdir(parents=True, exist_ok=True)
            data = {"active_batteries": self._active}
            self._config_path.write_text(yaml.dump(data, default_flow_style=False))
            self.get_logger().info(f"Saved active batteries to {self._config_path}: {self._active}")
        except Exception as e:
            self.get_logger().warn(f"Failed to save config: {e}")

    def _on_param_change(self, params) -> SetParametersResult:
        """Handle dynamic parameter changes from UI."""
        for param in params:
            if param.name == "active_batteries":
                self._active = list(param.value) if param.value else []
                active_str = ", ".join(self._active) if self._active else "auto-select"
                self.get_logger().info(f"Active batteries updated: [{active_str}]")
                self._save_config()
                self._publish_combined()
        return SetParametersResult(successful=True)

    def _tick_once(self):
        """One-shot early discovery, then cancel itself."""
        self._tick()
        self.destroy_timer(self._init_timer)

    def _tick(self):
        """Discover new battery topics and publish combined state."""
        self._discover_topics()
        self._publish_combined()

    def _discover_topics(self):
        """Find and subscribe to new /batteries/*/state topics."""
        topic_list = self.get_topic_names_and_types()
        for topic_name, types in topic_list:
            if not topic_name.startswith(BATTERY_TOPIC_PREFIX):
                continue
            if not topic_name.endswith(BATTERY_TOPIC_SUFFIX):
                continue
            if "sensor_msgs/msg/BatteryState" not in types:
                continue

            # Extract battery name from topic: /batteries/<name>/state
            parts = topic_name.split("/")
            if len(parts) != 4:
                continue
            battery_name = parts[2]

            if battery_name in EXCLUDED_TOPICS:
                continue
            if battery_name in self._battery_subs:
                continue

            self._subscribe_battery(battery_name, topic_name)

    def _subscribe_battery(self, name: str, topic: str):
        """Subscribe to an individual battery topic."""

        def callback(msg: BatteryState, battery_name=name):
            self._battery_states[battery_name] = msg

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        sub = self.create_subscription(BatteryState, topic, callback, qos)
        self._battery_subs[name] = sub
        self.get_logger().info(f"Discovered battery: {name} ({topic})")

    def _get_active_states(self) -> list[tuple[str, BatteryState]]:
        """Get the battery states for the active selection."""
        if self._active:
            return [
                (name, state)
                for name, state in self._battery_states.items()
                if name in self._active
            ]

        # Auto-select: use all discovered non-excluded batteries
        return list(self._battery_states.items())

    def _publish_combined(self):
        """Compute and publish combined battery state."""
        active = self._get_active_states()
        if not active:
            return

        msg = self._combine_states(active)
        if msg is not None:
            self._pub.publish(msg)

    def _combine_states(self, states: list[tuple[str, BatteryState]]) -> Optional[BatteryState]:
        """Combine multiple battery states into one (parallel battery config)."""
        if not states:
            return None

        n = len(states)
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "rover_main"

        # Parallel batteries: average voltage, sum current/capacity/charge
        msg.voltage = sum(s.voltage for _, s in states) / n
        msg.current = sum(s.current for _, s in states)
        msg.percentage = sum(s.percentage for _, s in states) / n
        msg.capacity = sum(s.capacity for _, s in states)
        msg.charge = sum(s.charge for _, s in states)
        msg.design_capacity = sum(s.design_capacity for _, s in states)
        msg.present = all(s.present for _, s in states)

        # Temperature: take the max (worst case)
        temps = [s.temperature for _, s in states if s.temperature == s.temperature]
        msg.temperature = max(temps) if temps else float("nan")

        # Status: if any is charging → charging, else if any discharging → discharging
        statuses = [s.power_supply_status for _, s in states]
        if BatteryState.POWER_SUPPLY_STATUS_CHARGING in statuses:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif BatteryState.POWER_SUPPLY_STATUS_DISCHARGING in statuses:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE

        # Merge cell voltages from all batteries
        for _, s in states:
            msg.cell_voltage.extend(s.cell_voltage)
            msg.cell_temperature.extend(s.cell_temperature)

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = BatteryAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
