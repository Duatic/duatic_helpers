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

"""Requests a failure snapshot when a hardware emergency stop freezes the drive data.

When the e-stop cuts motor power the drives keep publishing, but their readings stop
changing. The node watches every DriveStateCollection topic for that and, on detection,
calls the failure_snapshot node's trigger service. It re-arms once the readings move
again. A topic that stops publishing is treated the same way.
"""

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from duatic_controller_msgs.msg import DriveStateCollection
from duatic_helper_msgs.srv import TriggerSnapshot

DRIVE_STATE_TYPE = "duatic_controller_msgs/msg/DriveStateCollection"

NS_PER_SEC = 1e9

DISCOVERY_PERIOD_SEC = 2.0
WATCHDOG_PERIOD_SEC = 0.1


def _differs(old: float, new: float) -> bool:
    """Compare two readings, treating a reading that stays NaN as unchanged."""
    return old != new and not (math.isnan(old) and math.isnan(new))


def has_state_changed(old_msg: DriveStateCollection, new_msg: DriveStateCollection) -> bool:
    """Report whether the kinematic readings differ between two drive-state collections.

    The exact float comparison is deliberate: an unpowered drive republishes bit-identical
    values, while a live one always jitters at least in the last encoder or torque bits.
    """
    if len(old_msg.states) != len(new_msg.states):
        return True

    for old_state, new_state in zip(old_msg.states, new_msg.states):
        if (
            _differs(old_state.joint_position, new_state.joint_position)
            or _differs(old_state.joint_velocity, new_state.joint_velocity)
            or _differs(old_state.joint_effort, new_state.joint_effort)
        ):
            return True
    return False


@dataclass
class FreezeDetector:
    """Freeze-detection state for one DriveStateCollection topic."""

    timeout_ns: int
    last_message: Optional[DriveStateCollection] = None
    last_changed_ns: Optional[int] = None
    last_sample_ns: Optional[int] = None
    # Only a topic that has proven it streams live data can be considered frozen.
    armed: bool = False

    def update(self, msg: DriveStateCollection, now_ns: int) -> None:
        """Record a sample of the topic."""
        if self.last_message is None:
            self.last_changed_ns = now_ns
        elif has_state_changed(self.last_message, msg):
            self.armed = True
            self.last_changed_ns = now_ns
        self.last_message = msg
        self.last_sample_ns = now_ns

    def is_frozen(self, now_ns: int) -> bool:
        """Report whether live data has stopped changing for longer than the timeout."""
        return self.armed and (now_ns - self.last_changed_ns) > self.timeout_ns

    def is_silent(self, now_ns: int) -> bool:
        """Report whether the topic has stopped publishing for longer than the timeout."""
        return (now_ns - self.last_sample_ns) > self.timeout_ns

    def disarm(self) -> None:
        """Ignore the topic until its readings move again."""
        self.armed = False


def is_new_event(last_changed_ns: list[int], last_trigger_ns: Optional[int]) -> bool:
    """Report whether newly stopped topics call for a snapshot of their own.

    A topic that stopped moving before the last snapshot was requested is part of what
    that snapshot recorded. One that moved since then has stopped again.
    """
    return last_trigger_ns is None or any(ns > last_trigger_ns for ns in last_changed_ns)


class EstopMonitorNode(Node):
    """Requests a failure snapshot when a hardware e-stop freezes the drive data."""

    def __init__(self):
        super().__init__("estop_monitor")

        self.declare_parameter("timeout_threshold_sec", 2.0)
        self.declare_parameter("trigger_service", "failure_snapshot/trigger")

        self._timeout_ns = int(self.get_parameter("timeout_threshold_sec").value * NS_PER_SEC)
        self._last_trigger_ns: Optional[int] = None
        self._trigger_client = self.create_client(
            TriggerSnapshot, self.get_parameter("trigger_service").value
        )

        self._subscriptions_by_topic: dict[str, object] = {}
        self._detectors: dict[str, FreezeDetector] = {}
        # Never spun: its subscriptions are polled at the watchdog rate, so the drive data
        # costs one take per period instead of a callback per message.
        self._poll_node = rclpy.create_node(
            f"{self.get_name()}_poll",
            namespace=self.get_namespace(),
            use_global_arguments=False,
            enable_rosout=False,
            start_parameter_services=False,
        )

        self._discovery_timer = self.create_timer(DISCOVERY_PERIOD_SEC, self._discover_topics)
        self._watchdog_timer = self.create_timer(WATCHDOG_PERIOD_SEC, self._watchdog_check)

    def _discover_topics(self) -> None:
        """Subscribe to any DriveStateCollection topic that appeared in this namespace."""
        prefix = self.get_namespace().rstrip("/") + "/"
        for topic_name, types in self.get_topic_names_and_types():
            if (
                topic_name in self._subscriptions_by_topic
                or DRIVE_STATE_TYPE not in types
                or not topic_name.startswith(prefix)
            ):
                continue
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self._subscriptions_by_topic[topic_name] = self._poll_node.create_subscription(
                DriveStateCollection, topic_name, lambda msg: None, qos
            )
            self._detectors[topic_name] = FreezeDetector(timeout_ns=self._timeout_ns)
            self.get_logger().debug(f"Watching {topic_name}")

    def _watchdog_check(self) -> None:
        """Sample the watched topics and request a snapshot once one has stopped."""
        now_ns = self.get_clock().now().nanoseconds
        for topic_name, subscription in self._subscriptions_by_topic.items():
            with subscription.handle:
                taken = subscription.handle.take_message(DriveStateCollection, False)
            if taken is not None:
                self._detectors[topic_name].update(taken[0], now_ns)

        frozen = {
            topic_name: "no data" if detector.is_silent(now_ns) else "data frozen"
            for topic_name, detector in self._detectors.items()
            if detector.is_frozen(now_ns)
        }
        if not frozen:
            return
        for topic_name in frozen:
            self._detectors[topic_name].disarm()
        description = ", ".join(f"{state} on {name}" for name, state in frozen.items())

        last_changed = [self._detectors[name].last_changed_ns for name in frozen]
        if not is_new_event(last_changed, self._last_trigger_ns):
            self.get_logger().info(f"Part of the same event: {description}")
            return
        if self._request_snapshot(description):
            self._last_trigger_ns = now_ns

    def _request_snapshot(self, description: str) -> bool:
        """Call the trigger service, if it is there."""
        self.get_logger().error(
            f"Drive data stopped, a hardware e-stop or a dead driver: {description}"
        )
        if not self._trigger_client.service_is_ready():
            self.get_logger().error(
                f"Snapshot service {self._trigger_client.srv_name} is not available"
            )
            return False
        request = TriggerSnapshot.Request(reason=f"Drive data stopped: {description}")
        self._trigger_client.call_async(request).add_done_callback(self._on_snapshot_done)
        return True

    def _on_snapshot_done(self, future) -> None:
        """Report a snapshot that was not written."""
        if future.exception() is not None or not future.result().success:
            message = future.exception() or future.result().message
            self.get_logger().error(f"No snapshot written: {message}")

    def destroy_node(self):
        self._poll_node.destroy_node()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EstopMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
