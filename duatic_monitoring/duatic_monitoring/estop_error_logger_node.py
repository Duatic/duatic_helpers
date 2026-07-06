# Copyright 2026 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
@file
@brief Monitors for emergency stop events and dumps diagnostic data to a rosbag.

Auto-discovers all visible topics and maintains a rolling buffer of their raw
serialized messages. Monitors duatic_controller_msgs/DriveStateCollection specifically to detect
frozen data states (indicating the e-stop was pressed and motors lost power).
On detection, dumps the buffered history of ALL topics to a timestamped MCAP rosbag.
"""

import os
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from functools import partial
from pathlib import Path
import shutil
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rcl_interfaces.msg import Log
import rosbag2_py

from rosidl_runtime_py.utilities import get_message
from duatic_controller_msgs.msg import DriveStateCollection

STATE_MSG_TYPE = "duatic_controller_msgs/msg/DriveStateCollection"
LOG_SEVERITY = {1: "DEBUG", 2: "INFO", 4: "WARN", 8: "ERROR", 16: "FATAL"}


@dataclass
class DriveStateCollectionMonitor:
    """
    @brief Data class representing the monitoring state of an DriveStateCollection topic.
    """

    topic_name: str
    last_changed_ns: Optional[int] = None
    last_message: Optional[DriveStateCollection] = None
    was_active: bool = False
    has_seen_dynamic_data: bool = False
    frozen: bool = False


class EStopErrorLoggerNode(Node):
    """
    @brief ROS 2 Node that logs system state upon detecting a hardware emergency stop.

    This node dynamically discovers topics, buffers their serialized data, and
    evaluates incoming DriveStateCollection messages to detect if sensor data has frozen
    (indicative of a motor power loss via E-Stop). Upon detection, it writes
    the buffered messages into an MCAP rosbag2 format.
    """

    def __init__(self):
        """
        @brief Initializes the EStopErrorLoggerNode, declares parameters, and sets up timers.

        @details By default, it maintains a 15.0-second rolling buffer (`buffer_duration_sec`),
                 effectively saving the system state 10 seconds before the hardware freeze event
                 and the 5 seconds during the timeout threshold (`timeout_threshold_sec`).
        """
        super().__init__("estop_error_logger")

        self.declare_parameter("buffer_duration_sec", 15.0)
        self.declare_parameter("timeout_threshold_sec", 5.0)
        self.declare_parameter("output_dir", "/ros2_ws/estop_logs")

        self._buffer_duration = self.get_parameter("buffer_duration_sec").value
        self._timeout_threshold_ns = int(self.get_parameter("timeout_threshold_sec").value * 1e9)
        self._output_dir = Path(os.path.expanduser(self.get_parameter("output_dir").value))

        self._generic_buffers: dict[str, deque] = {}
        self._topic_types: dict[str, str] = {}
        self._generic_subscriptions: dict[str, object] = {}
        self._arm_state_monitors: dict[str, DriveStateCollectionMonitor] = {}

        self._rosout_buffer: deque = deque(maxlen=5000)
        self._rosout_sub = self.create_subscription(Log, "/rosout", self._rosout_callback, 100)

        self._last_dump_ns: Optional[int] = None

        self._discovery_timer = self.create_timer(2.0, self._discover_topics)
        self._watchdog_timer = self.create_timer(0.02, self._watchdog_check)

        self.get_logger().debug(
            f"E-stop error logger started. Output: {self._output_dir}, "
            f"data freeze threshold: {self._timeout_threshold_ns / 1e9:.1f}s, "
            f"buffer duration: {self._buffer_duration}s"
        )

    def _discover_topics(self) -> None:
        """
        @brief Polls the ROS 2 graph for new topics and initiates subscriptions.
        """
        topic_list = self.get_topic_names_and_types()
        for topic_name, types in topic_list:
            if not types:
                continue

            topic_type = types[0]

            if topic_name not in self._generic_buffers:
                self._subscribe_generic_buffer(topic_name, topic_type)

    def _subscribe_generic_buffer(self, topic_name: str, topic_type: str) -> None:
        """
        @brief Creates a raw best-effort subscription to a topic to buffer its serialized bytes.

        @param topic_name The name of the ROS 2 topic.
        @param topic_type The ROS 2 message type string.
        """
        try:
            msg_class = get_message(topic_type)
        except Exception as e:
            self.get_logger().debug(f"Could not import {topic_type} for {topic_name}: {e}")
            return

        self._generic_buffers[topic_name] = deque(maxlen=10000)
        self._topic_types[topic_name] = topic_type

        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        cb = partial(self._generic_callback, topic_name=topic_name, topic_type=topic_type)
        sub = self.create_subscription(msg_class, topic_name, cb, qos, raw=True)

        self._generic_subscriptions[topic_name] = sub
        self.get_logger().debug(f"Buffering topic for rosbag: {topic_name} ({topic_type})")

    def _generic_callback(self, msg_bytes: bytes, topic_name: str, topic_type: str) -> None:
        """
        @brief Appends incoming serialized messages to the rolling buffer and manages eviction.

        @param msg_bytes The raw serialized CDR bytes of the message.
        @param topic_name The name of the ROS 2 topic.
        @param topic_type The ROS 2 message type string.
        """
        now_ns = self.get_clock().now().nanoseconds
        q = self._generic_buffers[topic_name]

        q.append((now_ns, msg_bytes))

        cutoff = now_ns - int(self._buffer_duration * 1e9)
        while q and q[0][0] < cutoff:
            q.popleft()

        if topic_type == STATE_MSG_TYPE:
            msg = deserialize_message(msg_bytes, DriveStateCollection)
            self._update_arm_state_monitor(topic_name, msg, now_ns)

    def _update_arm_state_monitor(
        self, topic_name: str, msg: DriveStateCollection, now_ns: int
    ) -> None:
        """
        @brief Evaluates deserialized DriveStateCollection messages to update the hardware freeze monitor.

        @param topic_name The name of the DriveStateCollection topic.
        @param msg The deserialized DriveStateCollection message.
        @param now_ns The current node time in nanoseconds.
        """
        monitor = self._arm_state_monitors.setdefault(
            topic_name, DriveStateCollectionMonitor(topic_name=topic_name)
        )
        monitor.was_active = True

        if monitor.last_message is None:
            # First message ever received. Establish baseline, but do not arm the freeze detector yet.
            monitor.last_changed_ns = now_ns
            monitor.last_message = msg
            return

        if self._has_state_changed(monitor.last_message, msg):
            monitor.has_seen_dynamic_data = True
            monitor.last_changed_ns = now_ns
            monitor.frozen = False

        monitor.last_message = msg

    def _has_state_changed(
        self, old_msg: DriveStateCollection, new_msg: DriveStateCollection
    ) -> bool:
        """
        @brief Compares specific kinematic fields between two DriveStateCollection messages to detect a data freeze.

        @param old_msg The previously received DriveStateCollection message.
        @param new_msg The newly received DriveStateCollection message.
        @return True if the kinematic states have changed, False otherwise.
        """
        if len(old_msg.states) != len(new_msg.states):
            return True

        for o_state, n_state in zip(old_msg.states, new_msg.states):
            if o_state.joint_position != n_state.joint_position:
                return True
            if o_state.joint_velocity != n_state.joint_velocity:
                return True
            if o_state.joint_effort != n_state.joint_effort:
                return True
        return False

    def _rosout_callback(self, msg: Log) -> None:
        """
        @brief Buffers system logs for diagnostic summary generation.

        @param msg The rcl_interfaces/Log message.
        """
        now_ns = self.get_clock().now().nanoseconds
        self._rosout_buffer.append((now_ns, msg))

    def _watchdog_check(self) -> None:
        """
        @brief Evaluates monitored topics against the timeout threshold to trigger a dump.
        """
        now_ns = self.get_clock().now().nanoseconds

        frozen_topics = []
        for topic_name, monitor in self._arm_state_monitors.items():
            if not monitor.was_active or monitor.frozen or monitor.last_changed_ns is None:
                continue

            # Only trigger a timeout if the topic has previously proven to be streaming real data
            if not monitor.has_seen_dynamic_data:
                continue

            if (now_ns - monitor.last_changed_ns) > self._timeout_threshold_ns:
                frozen_topics.append(topic_name)
                monitor.frozen = True

        if frozen_topics:
            self._trigger_dump(frozen_topics, now_ns)

    def _trigger_dump(self, frozen_topics: list[str], now_ns: int) -> None:
        """
        @brief Coordinates the execution of diagnostic artifact generation.

        @param frozen_topics List of topic names that triggered the e-stop detection.
        @param now_ns The timestamp in nanoseconds when the dump was triggered.
        """
        self._last_dump_ns = now_ns

        self.get_logger().error(
            f"HARDWARE E-STOP DETECTED (Data Frozen)! Topics frozen: {frozen_topics}. "
            f"Dumping all topics to rosbag..."
        )

        timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        dump_dir = self._output_dir / timestamp_str
        dump_dir.mkdir(parents=True, exist_ok=True)

        self._write_summary(dump_dir, frozen_topics, now_ns)
        self._write_terminal_logs(dump_dir)
        self._write_rosbag(dump_dir)
        self._write_session_logs(dump_dir)

        self.get_logger().info(f"Diagnostics completely dumped to: {dump_dir}. Shutting down.")

        raise SystemExit(0)

    def _write_summary(self, dump_dir: Path, frozen_topics: list[str], now_ns: int) -> None:
        """
        @brief Writes a human-readable text summary of the E-Stop event and system status.

        @param dump_dir The directory where the summary file will be saved.
        @param frozen_topics List of topic names that triggered the detection.
        @param now_ns Trigger timestamp in nanoseconds.
        """
        lines = [
            "E-STOP EVENT DETECTED (DATA FROZEN)",
            f"Timestamp: {datetime.now().isoformat()}",
            f"ROS time (ns): {now_ns}",
            "",
            "FROZEN TOPICS:",
        ]
        for topic in frozen_topics:
            monitor = self._arm_state_monitors[topic]
            gap_ms = (now_ns - monitor.last_changed_ns) / 1e6 if monitor.last_changed_ns else 0
            lines.append(f"  {topic} (data unchanged for {gap_ms:.1f}ms)")

        lines.append("")
        lines.append(f"ALL MONITORED TOPICS CAPTURED IN ROSBAG ({len(self._generic_buffers)}):")
        for topic, buf in self._generic_buffers.items():
            lines.append(f"  {topic}: buffer size {len(buf)} msgs")

        (dump_dir / "summary.txt").write_text("\n".join(lines) + "\n")

    def _write_terminal_logs(self, dump_dir: Path) -> None:
        """
        @brief Dumps the buffered /rosout terminal logs to a plain text file.

        @param dump_dir The directory to save the terminal_logs.txt file.
        """
        lines = []
        for ts_ns, msg in self._rosout_buffer:
            severity = LOG_SEVERITY.get(msg.level, f"LVL{msg.level}")
            lines.append(f"[{ts_ns}] [{severity}] [{msg.name}] {msg.msg.strip()}")

        (dump_dir / "terminal_logs.txt").write_text("\n".join(lines) + "\n")

    def _write_rosbag(self, dump_dir: Path) -> None:
        """
        @brief Writes all raw buffered messages into an MCAP rosbag2 format.

        @param dump_dir The directory where the rosbag subdirectory will be created.
        """
        bag_dir = str(dump_dir / "estop_bag")

        # Set max_cache_size=0 to force immediate, synchronous writes to disk.
        # This completely bypasses the Python GC / pybind11 flushing delay.
        storage_options = rosbag2_py.StorageOptions(
            uri=bag_dir, storage_id="mcap", max_cache_size=0
        )

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )

        writer = rosbag2_py.SequentialWriter()
        writer.open(storage_options, converter_options)

        for topic_name, type_str in self._topic_types.items():
            bag_type_str = type_str.replace("/msg/", "/")

            topic_meta = rosbag2_py.TopicMetadata(
                id=0,
                name=topic_name,
                type=bag_type_str,
                serialization_format="cdr",
            )
            writer.create_topic(topic_meta)

        self.get_logger().info("Writing raw buffers to MCAP storage...")
        for topic_name, buf in self._generic_buffers.items():
            for ts_ns, msg_bytes in buf:
                writer.write(topic_name, msg_bytes, ts_ns)

        # Explicitly close the writer to force the OS to flush the file descriptor.
        # Fall back to 'del' for safety if the Python API version lacks .close()
        if hasattr(writer, "close"):
            writer.close()
        del writer

        self.get_logger().info(f"Rosbag successfully written to: {bag_dir}")

    def _write_session_logs(self, dump_dir: Path) -> None:
        """
        @brief Copies the raw standard output/error logs from the current ROS 2 launch session.

        @param dump_dir The directory where the raw_terminal_logs subdirectory will be created.
        """
        base_log_dir = Path(os.environ.get("ROS_LOG_DIR", os.path.expanduser("~/.ros/log")))

        if not base_log_dir.exists():
            self.get_logger().warn(f"ROS log directory not found at {base_log_dir}. Skipping.")
            return

        try:
            latest_link = base_log_dir / "latest"
            if latest_link.is_symlink():
                latest_session_dir = latest_link.resolve()
            else:
                log_dirs = [d for d in base_log_dir.iterdir() if d.is_dir() and not d.is_symlink()]
                if not log_dirs:
                    return
                latest_session_dir = max(log_dirs, key=os.path.getmtime)

            dest_dir = dump_dir / "raw_session_logs"
            shutil.copytree(latest_session_dir, dest_dir, dirs_exist_ok=True)

        except Exception as e:
            self.get_logger().error(f"Failed to copy raw session logs: {e}")


def main(args=None):
    """
    @brief Node entry point.
    """
    rclpy.init(args=args)
    node = EStopErrorLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
