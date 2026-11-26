# Copyright 2026 Duatic AG
# SPDX-License-Identifier: BSD-3-Clause

"""Helpers for launch tests of robots that start through the bringup supervisor."""

import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray

from duatic_bringup_supervisor.supervisor import stage_of_status


def wait_for_stages(node, namespace: str, timeout: float) -> dict[str, str]:
    """Wait until the supervisor in ``namespace`` reports every stage ready, or the timeout.

    :param node: A node of the test, spun by this function.
    :param namespace: The robot's namespace, empty for none.
    :param timeout: Seconds to wait.
    :returns: The last reported state of every stage, by stage name.
    """
    states: dict[str, str] = {}

    def on_diagnostics(message: DiagnosticArray) -> None:
        """Remember the state of each stage the supervisor reports."""
        for status in message.status:
            states[stage_of_status(status.name)] = next(
                (v.value for v in status.values if v.key == "state"), ""
            )

    prefix = f"/{namespace}" if namespace else ""
    subscription = node.create_subscription(
        DiagnosticArray, f"{prefix}/diagnostics", on_diagnostics, 10
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not (states and set(states.values()) == {"ready"}):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(subscription)
    return dict(states)
