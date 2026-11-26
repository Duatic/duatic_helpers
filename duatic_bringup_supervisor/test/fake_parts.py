# Copyright 2026 Duatic AG
# SPDX-License-Identifier: BSD-3-Clause

"""Stand-ins for the parts of a robot in the supervisor launch tests.

``fake_parts.py lifecycle --ros-args -r __node:=<name>`` runs a lifecycle node that reports every
transition on ``transitions`` and fails to activate when its ``fail_on_activate`` parameter is
true. ``fake_parts.py publisher --ros-args -r __node:=<name>`` runs a plain node that publishes on
``data`` until its ``stop_after`` parameter (seconds, 0 for never) has passed.
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class FakeLifecycleNode(LifecycleNode):
    """A lifecycle node that reports its transitions."""

    def __init__(self):
        """Create the transition reporter."""
        super().__init__("fake_lifecycle")
        self.declare_parameter("fail_on_activate", False)
        qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reporter = self.create_publisher(String, "transitions", qos)

    def _report(self, transition: str) -> TransitionCallbackReturn:
        """Publish ``<node>:<transition>`` and accept the transition."""
        self.reporter.publish(String(data=f"{self.get_name()}:{transition}"))
        return TransitionCallbackReturn.SUCCESS

    def on_configure(self, state):
        """Report the configure transition."""
        return self._report("configure")

    def on_activate(self, state):
        """Report the activate transition, or fail it when asked to."""
        if self.get_parameter("fail_on_activate").value:
            self._report("activate_failed")
            return TransitionCallbackReturn.FAILURE
        self._report("activate")
        return super().on_activate(state)

    def on_deactivate(self, state):
        """Report the deactivate transition."""
        self._report("deactivate")
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        """Report the cleanup transition."""
        return self._report("cleanup")


class FakePublisher(Node):
    """A plain node that publishes on ``data`` at 10 Hz, and stops after ``stop_after`` s if set."""

    def __init__(self):
        """Start publishing."""
        super().__init__("fake_publisher")
        self.stop_after = self.declare_parameter("stop_after", 0.0).value
        self.started = self.get_clock().now()
        self.publisher = self.create_publisher(String, "data", 10)
        self.timer = self.create_timer(0.1, self._tick)

    def _tick(self):
        """Publish one message, or stop publishing once ``stop_after`` has passed."""
        elapsed = (self.get_clock().now() - self.started).nanoseconds / 1e9
        if self.stop_after and elapsed > self.stop_after:
            self.timer.cancel()
            return
        self.publisher.publish(String(data="tick"))


def main() -> None:
    """Run the fake part named by the first argument."""
    kind = sys.argv[1]
    rclpy.init(args=sys.argv)
    node = FakeLifecycleNode() if kind == "lifecycle" else FakePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
