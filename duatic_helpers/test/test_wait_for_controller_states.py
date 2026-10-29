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

import threading
import unittest

import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from rclpy.executors import SingleThreadedExecutor

from duatic_helpers import wait_for_controller_states

STATES = {"joint_state_broadcaster": "active", "freeze_controller": "inactive"}


class TestWaitForControllerStates(unittest.TestCase):

    def setUp(self) -> None:
        rclpy.init()
        self.server_node = rclpy.create_node("fake_controller_manager")
        self.server_node.create_service(
            ListControllers, "/test_cm/list_controllers", self.list_controllers
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.server_node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.node = rclpy.create_node("controller_state_checker")

    def tearDown(self) -> None:
        self.executor.shutdown()
        self.spin_thread.join()
        self.node.destroy_node()
        self.server_node.destroy_node()
        rclpy.shutdown()

    def list_controllers(
        self, _request: ListControllers.Request, response: ListControllers.Response
    ) -> ListControllers.Response:
        """
        Reports the controllers of STATES with their states.

        :param _request: Unused request.
        :param response: Response to fill.

        :returns: The response listing every controller of STATES.
        """
        response.controller = [ControllerState(name=n, state=s) for n, s in STATES.items()]
        return response

    def test_matching_states_pass(self) -> None:
        wait_for_controller_states(self.node, STATES, controller_manager="/test_cm", timeout=5.0)

    def test_mismatch_names_controller_and_actual_state(self) -> None:
        expected = {"joint_state_broadcaster": "active", "freeze_controller": "active"}
        with self.assertRaises(AssertionError) as ctx:
            wait_for_controller_states(
                self.node, expected, controller_manager="/test_cm", timeout=1.0
            )
        self.assertIn("freeze_controller is inactive (expected active)", str(ctx.exception))
        self.assertNotIn("joint_state_broadcaster", str(ctx.exception))

    def test_missing_controller_is_reported(self) -> None:
        with self.assertRaises(AssertionError) as ctx:
            wait_for_controller_states(
                self.node, {"gripper": "active"}, controller_manager="/test_cm", timeout=1.0
            )
        self.assertIn("gripper is not listed (expected active)", str(ctx.exception))
