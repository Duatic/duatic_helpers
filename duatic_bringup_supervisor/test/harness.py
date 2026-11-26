# Copyright 2026 Duatic AG
# SPDX-License-Identifier: BSD-3-Clause

"""Shared pieces of the supervisor launch tests: fake parts, the supervisor and an observer."""

import os
import sys
import tempfile
import textwrap
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

FAKE_PARTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fake_parts.py")


def fake(kind: str, name: str, namespace: str, **parameters) -> ExecuteProcess:
    """A fake lifecycle node or publisher process."""
    args = [sys.executable, FAKE_PARTS, kind, "--ros-args", "-r", f"__node:={name}"]
    args += ["-r", f"__ns:=/{namespace}"]
    for key, value in parameters.items():
        args += ["-p", f"{key}:={value}"]
    return ExecuteProcess(cmd=args, name=name, output="screen")


def supervisor(namespace: str, stages: str) -> Node:
    """The supervisor process, with ``stages`` as the base file of its stage directory."""
    directory = tempfile.mkdtemp()
    with open(os.path.join(directory, "stages.yaml"), "w") as f:
        f.write(textwrap.dedent(stages))
    return Node(
        package="duatic_bringup_supervisor",
        executable="supervisor",
        name="bringup_supervisor",
        namespace=namespace,
        output="screen",
        parameters=[{"directory": directory, "watch_period": 0.5}],
    )


class Observer:
    """Collects the supervisor's stage states and the fake nodes' transitions."""

    def __init__(self, node, namespace: str):
        """Subscribe to the diagnostics and transition reports."""
        self.node = node
        self.namespace = namespace
        self.states: dict[str, str] = {}
        self.transitions: list[str] = []
        self.lock = threading.Lock()
        node.create_subscription(
            DiagnosticArray, f"/{namespace}/diagnostics", self._on_diagnostics, 10
        )
        qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, f"/{namespace}/transitions", self._on_transition, qos)

    def _on_diagnostics(self, message: DiagnosticArray) -> None:
        """Remember the latest state of every stage."""
        with self.lock:
            for status in message.status:
                stage = status.name.rsplit(": ", 1)[-1]
                self.states[stage] = next(v.value for v in status.values if v.key == "state")

    def _on_transition(self, message: String) -> None:
        """Remember every reported transition in arrival order."""
        with self.lock:
            self.transitions.append(message.data)

    def wait_for(self, executor, predicate, timeout: float) -> bool:
        """Spin until ``predicate(observer)`` holds or the timeout passes."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            with self.lock:
                if predicate(self):
                    return True
        return False

    def request_shutdown(self, executor) -> bool:
        """Call the supervisor's shutdown service."""
        client = self.node.create_client(Trigger, f"/{self.namespace}/bringup_supervisor/shutdown")
        if not client.wait_for_service(timeout_sec=10.0):
            return False
        future = client.call_async(Trigger.Request())
        executor.spin_until_future_complete(future, timeout_sec=10.0)
        return future.done() and future.result().success

    def before(self, first: str, second: str) -> bool:
        """Whether transition ``first`` was reported before ``second``."""
        return (
            first in self.transitions
            and second in self.transitions
            and self.transitions.index(first) < self.transitions.index(second)
        )
