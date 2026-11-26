# Copyright 2026 Duatic AG
# SPDX-License-Identifier: BSD-3-Clause

"""A stage that fails after startup stops its dependents, and a failed required stage stops the
whole robot and makes the supervisor exit with code 1."""

import os
import sys
import unittest

import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from launch import LaunchDescription
from launch_testing.actions import ReadyToTest
from rclpy.executors import SingleThreadedExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from harness import Observer, fake, supervisor  # noqa: E402

NAMESPACE = "failures_test"
# The publisher stops after a few seconds: 'plain' fails after startup and takes 'consumer' with
# it. 'late' only starts once 'consumer' is ready and is required, but never comes up.
STAGES = """
stages:
  sensor:
    timeout: 20
    lifecycle_nodes: [sensor]
  plain:
    required: false
    timeout: 20
    ready:
      topics: [{name: data, max_age: 1.0}]
  consumer:
    required: false
    after: [plain]
    timeout: 20
    lifecycle_nodes: [consumer]
  late:
    after: [sensor]
    timeout: 12
    lifecycle_nodes: [missing_node]
"""
SUPERVISOR = supervisor(NAMESPACE, STAGES)


@pytest.mark.launch_test
def generate_test_description():
    """Fake parts plus the supervisor."""
    return LaunchDescription(
        [
            fake("lifecycle", "sensor", NAMESPACE),
            fake("lifecycle", "consumer", NAMESPACE),
            fake("publisher", "publisher", NAMESPACE, stop_after="4.0"),
            SUPERVISOR,
            ReadyToTest(),
        ]
    )


class TestFailures(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("failures_observer")
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.observer = Observer(cls.node, NAMESPACE)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_1_runtime_failure_stops_dependents(self):
        reached = self.observer.wait_for(
            self.executor,
            lambda o: o.states.get("plain") == "failed" and o.states.get("consumer") == "stopped",
            timeout=20.0,
        )
        self.assertTrue(reached, f"stage states: {self.observer.states}")
        self.assertIn("consumer:deactivate", self.observer.transitions)
        self.assertNotIn("sensor:deactivate", self.observer.transitions)

    def test_2_required_failure_stops_everything(self, proc_info):
        proc_info.assertWaitForShutdown(process=SUPERVISOR, timeout=30.0)
        stopped = self.observer.wait_for(
            self.executor,
            lambda o: "sensor:cleanup" in o.transitions and o.states.get("late") == "failed",
            timeout=5.0,
        )
        self.assertTrue(stopped, f"{self.observer.states} {self.observer.transitions}")


@launch_testing.post_shutdown_test()
class TestExit(unittest.TestCase):
    def test_supervisor_exits_with_code_1(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[1], process=SUPERVISOR
        )
