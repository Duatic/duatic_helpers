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

"""Waiting on two nodes of one process from two threads at the same time."""

import threading
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future

from duatic_helpers.duatic_spin_helper import spin_once, spin_until_future_complete


class TestSpinHelper(unittest.TestCase):

    def setUp(self) -> None:
        rclpy.init()
        self.nodes = [rclpy.create_node(f"waiter_{i}") for i in range(2)]

    def tearDown(self) -> None:
        for node in self.nodes:
            node.destroy_node()
        rclpy.shutdown()

    def test_two_threads_wait_at_once(self) -> None:
        errors = []

        def wait(node) -> None:
            future = Future()
            # Completes only while the node is spun, so both waits overlap.
            node.create_timer(0.2, lambda: future.done() or future.set_result(True))
            try:
                spin_until_future_complete(node, future, timeout_sec=2.0)
            except Exception as error:  # noqa: BLE001
                errors.append(f"{node.get_name()}: {error!r}")
                return
            if not future.done():
                errors.append(f"{node.get_name()}: timed out")

        threads = [threading.Thread(target=wait, args=(node,)) for node in self.nodes]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(errors, [])

    def test_a_wait_leaves_the_node_free_for_another_executor(self) -> None:
        spin_once(self.nodes[0], timeout_sec=0.01)
        executor = SingleThreadedExecutor()
        self.assertTrue(executor.add_node(self.nodes[0]))
        executor.shutdown()


if __name__ == "__main__":
    unittest.main()
