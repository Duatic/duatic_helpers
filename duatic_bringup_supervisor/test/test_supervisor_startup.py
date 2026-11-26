# Copyright 2026 Duatic AG
# SPDX-License-Identifier: BSD-3-Clause

"""The supervisor starts stages in dependency order, skips what an optional failure blocks, and
stops everything in reverse order on request."""

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

NAMESPACE = "startup_test"
STAGES = """
stages:
  sensor:
    timeout: 20
    lifecycle_nodes: [sensor]
  plain:
    timeout: 20
    ready:
      topics: [data]
  consumer:
    after: [sensor, plain]
    timeout: 20
    lifecycle_nodes: [consumer]
  broken:
    required: false
    timeout: 20
    lifecycle_nodes: [broken]
  needs_broken:
    after: [broken]
    timeout: 20
    ready:
      topics: [data]
"""
SUPERVISOR = supervisor(NAMESPACE, STAGES)


@pytest.mark.launch_test
def generate_test_description():
    """Fake parts plus the supervisor."""
    return LaunchDescription(
        [
            fake("lifecycle", "sensor", NAMESPACE),
            fake("lifecycle", "consumer", NAMESPACE),
            fake("lifecycle", "broken", NAMESPACE, fail_on_activate="true"),
            fake("publisher", "publisher", NAMESPACE),
            SUPERVISOR,
            ReadyToTest(),
        ]
    )


class TestStartup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("startup_observer")
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.observer = Observer(cls.node, NAMESPACE)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_1_stages_come_up_or_are_skipped(self):
        expected = {
            "sensor": "ready",
            "plain": "ready",
            "consumer": "ready",
            "broken": "failed",
            "needs_broken": "skipped",
        }
        reached = self.observer.wait_for(
            self.executor, lambda o: o.states == expected, timeout=40.0
        )
        self.assertTrue(reached, f"stage states: {self.observer.states}")

    def test_2_dependents_start_after_their_dependencies(self):
        self.assertTrue(
            self.observer.before("sensor:activate", "consumer:configure"),
            self.observer.transitions,
        )
        # The failed optional stage is cleaned up, so it can be started again later
        self.assertIn("broken:cleanup", self.observer.transitions)

    def test_3_shutdown_stops_in_reverse_order(self, proc_info):
        self.assertTrue(self.observer.request_shutdown(self.executor))
        proc_info.assertWaitForShutdown(process=SUPERVISOR, timeout=20.0)
        stopped = self.observer.wait_for(
            self.executor,
            lambda o: "sensor:cleanup" in o.transitions and "consumer:cleanup" in o.transitions,
            timeout=5.0,
        )
        self.assertTrue(stopped, self.observer.transitions)
        self.assertTrue(
            self.observer.before("consumer:deactivate", "sensor:deactivate"),
            self.observer.transitions,
        )


@launch_testing.post_shutdown_test()
class TestExit(unittest.TestCase):
    def test_supervisor_exits_cleanly_after_shutdown(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, process=SUPERVISOR)
