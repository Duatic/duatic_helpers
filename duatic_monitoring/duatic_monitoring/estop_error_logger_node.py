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

"""Detects a hardware emergency stop and dumps the surrounding system state to a rosbag.

Auto-discovers the topics on the graph and keeps a rolling buffer of their raw serialized
messages. DriveStateCollection topics are additionally deserialized and watched for frozen
data: when the e-stop cuts motor power the drives keep publishing, but their readings stop
changing. On detection, the buffered history of every topic is written to a timestamped MCAP
rosbag alongside plain-text summaries.

Latched (transient-local) topics such as /tf_static and /robot_description are special-cased:
they are published once at startup and are exempt from the rolling window, so the bag always
carries the static data needed to interpret the dynamic data. See BufferedTopic.trim and
EStopErrorLoggerNode._write_rosbag.

The node captures a single event and then exits (see EStopErrorLoggerNode).
"""

import os
import re
import shutil
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from functools import partial
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rcl_interfaces.msg import Log
import rosbag2_py
# rosbag2_py does not re-export the storage QoS type at package level; the pybind module is
# the only way to reach it, and it is what TopicMetadata.offered_qos_profiles expects.
from rosbag2_py._storage import QoS as BagQoS
from rosidl_runtime_py.utilities import get_message
from duatic_controller_msgs.msg import DriveStateCollection

DRIVE_STATE_TYPE = "duatic_controller_msgs/msg/DriveStateCollection"

# rcl_interfaces/Log severity values. Note that ROS 1 used 1/2/4/8/16 for these; taking the
# values from the message itself keeps the mapping from silently drifting again.
SEVERITY_NAMES = {
    Log.DEBUG: "DEBUG",
    Log.INFO: "INFO",
    Log.WARN: "WARN",
    Log.ERROR: "ERROR",
    Log.FATAL: "FATAL",
}

NS_PER_SEC = 1e9
NS_PER_MS = 1e6

DISCOVERY_PERIOD_SEC = 2.0
WATCHDOG_PERIOD_SEC = 0.1

SUBSCRIPTION_QUEUE_DEPTH = 100
ROSOUT_BUFFER_SIZE = 5000

# Buffered separately (/rosout) or of no diagnostic value in the bag (/parameter_events).
IGNORED_TOPICS = frozenset(["/rosout", "/parameter_events"])

# Message types too large to hold in a rolling buffer. Matched as regexes against the type name.
DEFAULT_EXCLUDED_TYPES = [
    r"sensor_msgs/msg/Image",
    r"sensor_msgs/msg/CompressedImage",
    r"sensor_msgs/msg/PointCloud2",
]

# Per-topic ceiling on buffered payload bytes, enforced on top of buffer_duration_sec so that
# a fast or fat topic cannot grow the buffer without bound.
DEFAULT_TOPIC_BUDGET_BYTES = 32 * 1024 * 1024

# Recorded against latched topics so that `ros2 bag play` republishes them transient-local.
# Replaying them volatile would fire the sample once at the very start of the bag, and any
# subscriber that joins a moment later (RViz, a TF listener) would never see it.
LATCHED_REPLAY_QOS = BagQoS(SUBSCRIPTION_QUEUE_DEPTH).reliable().transient_local()


@dataclass
class BufferedTopic:
    """Rolling buffer of the raw serialized messages seen on one discovered topic."""

    type_name: str
    subscription: object
    is_latched: bool = False
    messages: deque = field(default_factory=deque)  # (timestamp_ns, payload) pairs
    buffered_bytes: int = 0

    def append(self, timestamp_ns: int, payload: bytes) -> None:
        """Add a serialized message to the buffer."""
        self.messages.append((timestamp_ns, payload))
        self.buffered_bytes += len(payload)

    def trim(self, cutoff_ns: int, budget_bytes: int) -> None:
        """Evict messages older than the cutoff, then any excess over the byte budget.

        Latched topics ignore the cutoff. Their sample is published once at startup and never
        refreshed, so it is always older than the window, yet it is the only copy of the static
        data (/tf_static, /robot_description) that the dynamic data has to be interpreted
        against. The byte budget still applies to them.
        """
        while self.messages and (
            (not self.is_latched and self.messages[0][0] < cutoff_ns)
            or self.buffered_bytes > budget_bytes
        ):
            _, payload = self.messages.popleft()
            self.buffered_bytes -= len(payload)


@dataclass
class DriveStateMonitor:
    """Freeze-detection state for one DriveStateCollection topic."""

    topic_name: str
    last_message: Optional[DriveStateCollection] = None
    last_changed_ns: Optional[int] = None
    has_seen_dynamic_data: bool = False


class EStopErrorLoggerNode(Node):
    """Logs the system state when a hardware e-stop freezes the drive data.

    Discovers topics, buffers their serialized payloads, and watches DriveStateCollection
    messages for readings that stop changing. On detection it writes the buffers to an MCAP
    rosbag2 bag plus text summaries.

    This is a single-shot node: it captures the first event it sees and then exits, so a
    freeze cannot spend the rest of the session re-triggering itself. Relaunch to re-arm.

    The default 15 s buffer (buffer_duration_sec) captures roughly the 10 s before the freeze
    plus the 5 s the detector spends confirming it (timeout_threshold_sec). Latched topics are
    held outside that window and restamped to its start, so the bag is exactly the window long
    and still self-contained.
    """

    def __init__(self):
        super().__init__("estop_error_logger")

        self.declare_parameter("buffer_duration_sec", 15.0)
        self.declare_parameter("timeout_threshold_sec", 5.0)
        self.declare_parameter("output_dir", "/ros2_ws/estop_logs")
        self.declare_parameter("topic_budget_bytes", DEFAULT_TOPIC_BUDGET_BYTES)
        self.declare_parameter("excluded_types", DEFAULT_EXCLUDED_TYPES)

        self._buffer_duration_ns = int(self.get_parameter("buffer_duration_sec").value * NS_PER_SEC)
        self._timeout_threshold_ns = int(
            self.get_parameter("timeout_threshold_sec").value * NS_PER_SEC
        )
        self._output_dir = Path(os.path.expanduser(self.get_parameter("output_dir").value))
        self._topic_budget_bytes = self.get_parameter("topic_budget_bytes").value
        self._excluded_types = [
            re.compile(pattern) for pattern in self.get_parameter("excluded_types").value
        ]

        self._topics: dict[str, BufferedTopic] = {}
        self._skipped_topics: set[str] = set()
        self._drive_state_monitors: dict[str, DriveStateMonitor] = {}

        self._rosout_buffer: deque = deque(maxlen=ROSOUT_BUFFER_SIZE)
        self._rosout_sub = self.create_subscription(
            Log, "/rosout", self._rosout_callback, SUBSCRIPTION_QUEUE_DEPTH
        )

        self._discovery_timer = self.create_timer(DISCOVERY_PERIOD_SEC, self._discover_topics)
        self._watchdog_timer = self.create_timer(WATCHDOG_PERIOD_SEC, self._watchdog_check)

        self.get_logger().debug(
            f"E-stop error logger started. Output: {self._output_dir}, "
            f"data freeze threshold: {self._timeout_threshold_ns / NS_PER_SEC:.1f}s, "
            f"buffer duration: {self._buffer_duration_ns / NS_PER_SEC:.1f}s"
        )

    def _discover_topics(self) -> None:
        """Subscribe to any topic that appeared on the graph since the last poll."""
        for topic_name, types in self.get_topic_names_and_types():
            if topic_name in self._topics or topic_name in self._skipped_topics:
                continue

            if topic_name in IGNORED_TOPICS:
                self._skipped_topics.add(topic_name)
                continue

            if not types:
                continue

            # Wait for a publisher before subscribing: the subscription QoS has to be built
            # from the offered QoS, and guessing volatile now would permanently miss the
            # sample of a latched publisher that shows up on a later poll.
            publishers = self.get_publishers_info_by_topic(topic_name)
            if not publishers:
                continue

            if len(types) > 1:
                self.get_logger().debug(f"{topic_name} has types {types}; buffering {types[0]}")

            self._subscribe_topic(topic_name, types[0], publishers)

    def _subscribe_topic(self, topic_name: str, type_name: str, publishers: list) -> None:
        """Create a raw subscription that buffers the topic's serialized bytes."""
        if any(pattern.search(type_name) for pattern in self._excluded_types):
            self._skipped_topics.add(topic_name)
            self.get_logger().debug(f"Not buffering {topic_name}: type {type_name} is excluded")
            return

        try:
            msg_class = get_message(type_name)
        except Exception as e:
            # Record the failure, otherwise the topic is retried on every discovery poll.
            self._skipped_topics.add(topic_name)
            self.get_logger().debug(f"Could not import {type_name} for {topic_name}: {e}")
            return

        is_latched = self._is_latched(publishers)
        if is_latched:
            # Only a transient-local subscription is handed the sample a latched publisher
            # retained from before this node existed.
            qos = QoSProfile(
                depth=SUBSCRIPTION_QUEUE_DEPTH,
                reliability=self._compatible_reliability(publishers),
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
        else:
            qos = QoSProfile(
                depth=SUBSCRIPTION_QUEUE_DEPTH,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )

        callback = partial(self._buffer_callback, topic_name=topic_name)
        subscription = self.create_subscription(msg_class, topic_name, callback, qos, raw=True)

        self._topics[topic_name] = BufferedTopic(
            type_name=type_name, subscription=subscription, is_latched=is_latched
        )
        suffix = " [latched]" if is_latched else ""
        self.get_logger().debug(f"Buffering topic for rosbag: {topic_name} ({type_name}){suffix}")

    @staticmethod
    def _is_latched(publishers: list) -> bool:
        """Report whether every publisher offers the topic transient-local.

        A volatile subscription only ever sees messages published after it matched, so static
        data published once at startup (/tf_static, /robot_description) never reaches it and
        used to land in the bag as an empty topic. Only a transient-local subscription is given
        the retained sample.

        The check is deliberately all() rather than any(): a transient-local subscription does
        not match a volatile publisher, so on a topic with mixed publishers, asking for the
        retained sample would cost us the live data from the volatile ones.
        """
        return all(
            info.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
            for info in publishers
        )

    @staticmethod
    def _compatible_reliability(publishers: list) -> ReliabilityPolicy:
        """Pick the strongest reliability that still matches every publisher.

        A reliable subscription does not match a best-effort publisher, so it can only be
        requested when all of them offer reliable. Best-effort matches either.
        """
        if all(
            info.qos_profile.reliability == ReliabilityPolicy.RELIABLE for info in publishers
        ):
            return ReliabilityPolicy.RELIABLE
        return ReliabilityPolicy.BEST_EFFORT

    def _buffer_callback(self, payload: bytes, topic_name: str) -> None:
        """Append a serialized message to the rolling buffer.

        Eviction is not done here: it is driven by the clock in _trim_buffers instead.
        """
        now_ns = self.get_clock().now().nanoseconds
        topic = self._topics[topic_name]

        topic.append(now_ns, payload)

        if topic.type_name == DRIVE_STATE_TYPE:
            msg = deserialize_message(payload, DriveStateCollection)
            self._update_drive_state_monitor(topic_name, msg, now_ns)

    def _update_drive_state_monitor(
        self, topic_name: str, msg: DriveStateCollection, now_ns: int
    ) -> None:
        """Track when a drive-state topic last published readings that actually changed."""
        monitor = self._drive_state_monitors.setdefault(
            topic_name, DriveStateMonitor(topic_name=topic_name)
        )

        if monitor.last_message is None:
            # First message: establish a baseline without arming the freeze detector yet.
            monitor.last_message = msg
            monitor.last_changed_ns = now_ns
            return

        if self._has_state_changed(monitor.last_message, msg):
            monitor.has_seen_dynamic_data = True
            monitor.last_changed_ns = now_ns

        monitor.last_message = msg

    @staticmethod
    def _has_state_changed(old_msg: DriveStateCollection, new_msg: DriveStateCollection) -> bool:
        """Report whether the kinematic readings differ between two drive-state collections.

        The exact float comparison is deliberate: an unpowered drive republishes bit-identical
        values, while a live one always jitters at least in the last encoder or torque bits.
        Any difference at all therefore counts as the drive still being alive.
        """
        if len(old_msg.states) != len(new_msg.states):
            return True

        for old_state, new_state in zip(old_msg.states, new_msg.states):
            if (
                old_state.joint_position != new_state.joint_position
                or old_state.joint_velocity != new_state.joint_velocity
                or old_state.joint_effort != new_state.joint_effort
            ):
                return True
        return False

    def _rosout_callback(self, msg: Log) -> None:
        """Buffer system logs for the terminal log dump."""
        self._rosout_buffer.append((self.get_clock().now().nanoseconds, msg))

    def _trim_buffers(self, now_ns: int) -> None:
        """Enforce the rolling window on every buffer, including topics that fell silent.

        Retention has to be driven by the clock rather than by arriving messages. Trimming a
        buffer only from its own subscription callback leaves a topic that stops publishing
        (a lifecycle transition_event, a one-shot statistics/names) holding its startup
        messages forever: they age out of the window but nothing ever runs to evict them, and
        they drag the dumped bag's start time back to node startup. The bag then replays as
        minutes of near-silence followed by the window that was actually wanted.
        """
        cutoff_ns = now_ns - self._buffer_duration_ns
        for topic in self._topics.values():
            topic.trim(cutoff_ns, self._topic_budget_bytes)

    def _watchdog_check(self) -> None:
        """Dump diagnostics once a monitored drive-state topic has stopped changing."""
        now_ns = self.get_clock().now().nanoseconds
        self._trim_buffers(now_ns)

        frozen_topics = [
            topic_name
            for topic_name, monitor in self._drive_state_monitors.items()
            # Only a topic that has proven it streams live data can be considered frozen.
            if monitor.has_seen_dynamic_data
            and (now_ns - monitor.last_changed_ns) > self._timeout_threshold_ns
        ]
        if not frozen_topics:
            return

        dumped = self._trigger_dump(frozen_topics, now_ns)

        # Single-shot: capture one event, then leave. SystemExit unwinds rclpy.spin() in main().
        raise SystemExit(0 if dumped else 1)

    def _trigger_dump(self, frozen_topics: list[str], now_ns: int) -> bool:
        """Write every diagnostic artifact for the event, reporting whether the dump happened.

        The artifacts are written independently: a failure on one of them (a full disk while
        streaming out the bag, say) must not cost us the others.
        """
        self.get_logger().error(
            f"HARDWARE E-STOP DETECTED (data frozen)! Frozen topics: {frozen_topics}. "
            f"Dumping all buffered topics to a rosbag..."
        )

        dump_dir = self._output_dir / datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        try:
            dump_dir.mkdir(parents=True, exist_ok=True)
        except OSError as e:
            self.get_logger().error(f"Could not create dump directory {dump_dir}: {e}")
            return False

        # The buffers were trimmed against this same instant, so nothing dynamic predates it.
        window_start_ns = now_ns - self._buffer_duration_ns

        artifacts = (
            ("summary", partial(self._write_summary, dump_dir, frozen_topics, now_ns)),
            ("terminal logs", partial(self._write_terminal_logs, dump_dir)),
            ("rosbag", partial(self._write_rosbag, dump_dir, window_start_ns)),
            ("session logs", partial(self._write_session_logs, dump_dir)),
        )
        for name, write_artifact in artifacts:
            try:
                write_artifact()
            except Exception as e:
                self.get_logger().error(f"Failed to write {name}: {e}")

        self.get_logger().info(f"Diagnostics dumped to: {dump_dir}. Shutting down.")
        return True

    def _write_summary(self, dump_dir: Path, frozen_topics: list[str], now_ns: int) -> None:
        """Write a human-readable summary of the e-stop event and what was captured."""
        lines = [
            "E-STOP EVENT DETECTED (DATA FROZEN)",
            f"Timestamp: {datetime.now().isoformat(timespec='milliseconds')}",
            f"ROS time (ns): {now_ns}",
            "",
            "FROZEN TOPICS:",
        ]
        for topic_name in frozen_topics:
            monitor = self._drive_state_monitors[topic_name]
            gap_ms = (now_ns - monitor.last_changed_ns) / NS_PER_MS
            lines.append(f"  {topic_name} (data unchanged for {gap_ms:.1f} ms)")

        lines += ["", f"TOPICS CAPTURED IN ROSBAG ({len(self._topics)}):"]
        for topic_name, topic in sorted(self._topics.items()):
            size_kib = topic.buffered_bytes / 1024
            suffix = " [latched]" if topic.is_latched else ""
            lines.append(f"  {topic_name}: {len(topic.messages)} msgs, {size_kib:.1f} KiB{suffix}")

        (dump_dir / "summary.txt").write_text("\n".join(lines) + "\n")

    def _write_terminal_logs(self, dump_dir: Path) -> None:
        """Write the buffered /rosout messages as plain text."""
        lines = []
        for timestamp_ns, msg in self._rosout_buffer:
            stamp = datetime.fromtimestamp(timestamp_ns / NS_PER_SEC).strftime("%H:%M:%S.%f")[:-3]
            severity = SEVERITY_NAMES.get(msg.level, f"LVL{msg.level}")
            lines.append(f"[{stamp}] [{severity}] [{msg.name}] {msg.msg.strip()}")

        (dump_dir / "terminal_logs.txt").write_text("\n".join(lines) + "\n")

    def _write_rosbag(self, dump_dir: Path, window_start_ns: int) -> None:
        """Write all buffered payloads into an MCAP rosbag2 bag.

        Latched samples are older than the window (often by minutes) and are restamped to
        window_start_ns, so the bag spans exactly buffer_duration_sec and replays the static
        data first, ahead of the dynamic data that is expressed against it.
        """
        bag_dir = str(dump_dir / "estop_bag")

        # max_cache_size=0 writes straight through to disk rather than buffering inside the
        # writer, so the bag stays complete even if the process dies right after the dump.
        storage_options = rosbag2_py.StorageOptions(
            uri=bag_dir, storage_id="mcap", max_cache_size=0
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )

        writer = rosbag2_py.SequentialWriter()
        writer.open(storage_options, converter_options)
        try:
            for topic_name, topic in self._topics.items():
                writer.create_topic(
                    rosbag2_py.TopicMetadata(
                        id=0,
                        name=topic_name,
                        type=topic.type_name,
                        serialization_format="cdr",
                        offered_qos_profiles=[LATCHED_REPLAY_QOS] if topic.is_latched else [],
                    )
                )

            self.get_logger().info("Writing raw buffers to MCAP storage...")
            for topic_name, topic in self._topics.items():
                for timestamp_ns, payload in topic.messages:
                    # Only latched messages are older than the window; a max() lifts them to
                    # its start while leaving anything republished inside it at its real time.
                    writer.write(topic_name, payload, max(timestamp_ns, window_start_ns))
        finally:
            writer.close()

        self.get_logger().info(f"Rosbag successfully written to: {bag_dir}")

    def _write_session_logs(self, dump_dir: Path) -> None:
        """Copy the raw stdout/stderr logs of the current ROS 2 launch session."""
        base_log_dir = Path(os.environ.get("ROS_LOG_DIR", os.path.expanduser("~/.ros/log")))
        if not base_log_dir.exists():
            self.get_logger().warn(f"ROS log directory not found at {base_log_dir}. Skipping.")
            return

        latest_link = base_log_dir / "latest"
        if latest_link.is_symlink():
            session_dir = latest_link.resolve()
        else:
            session_dirs = [d for d in base_log_dir.iterdir() if d.is_dir() and not d.is_symlink()]
            if not session_dirs:
                self.get_logger().warn(f"No log session found in {base_log_dir}. Skipping.")
                return
            session_dir = max(session_dirs, key=os.path.getmtime)

        shutil.copytree(session_dir, dump_dir / "raw_session_logs", dirs_exist_ok=True)


def main(args=None):
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
