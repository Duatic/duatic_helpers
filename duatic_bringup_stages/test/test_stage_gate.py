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

"""Stage gates against a controller manager with mock hardware, a lifecycle node and published drive states."""

import importlib.util
import sys
import time
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node, SetParameter

from duatic_bringup_stages.stages import gate

URDF = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <ros2_control name="MockSystem" type="system">
    <hardware><plugin>mock_components/GenericSystem</plugin></hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>
"""

CONTROLLERS = """
/**:
  controller_manager:
    ros__parameters:
      update_rate: 50
      joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster
"""

WRONG_STATE = CONTROLLERS + "        state: inactive\n"

MISSING_CONTROLLER = CONTROLLERS.replace(
    "      joint_state_broadcaster:",
    "      missing_controller:\n"
    "        type: joint_state_broadcaster/JointStateBroadcaster\n"
    "        state: inactive\n"
    "      joint_state_broadcaster:",
)

LIFECYCLE_NODE = """
import rclpy
from rclpy.lifecycle import LifecycleNode
rclpy.init()
node = LifecycleNode("test_lifecycle_node")
node.trigger_configure()
node.trigger_activate()
rclpy.spin(node)
"""


def test_gates_pass_and_fail_on_the_right_checks(tmp_path):
    controllers = tmp_path / "controllers.yaml"
    controllers.write_text(CONTROLLERS)
    missing_controller = tmp_path / "missing_controller.yaml"
    missing_controller.write_text(MISSING_CONTROLLER)
    wrong_state = tmp_path / "wrong_state.yaml"
    wrong_state.write_text(WRONG_STATE)

    gates = {
        "controllers_ready": gate(
            "controllers_ready",
            30,
            controllers_config=str(controllers),
            hardware_components_active=True,
        ),
        "controller_missing": gate(
            "controller_missing", 8, controllers_config=str(missing_controller)
        ),
        "controller_wrong_state": gate(
            "controller_wrong_state", 8, controllers_config=str(wrong_state)
        ),
        "topic_ready": gate("topic_ready", 30, topics=["/joint_states"]),
        "topic_missing": gate("topic_missing", 5, topics=["/joint_states", "/never_published"]),
        "lifecycle_ready": gate("lifecycle_ready", 30, lifecycle_nodes=["/test_lifecycle_node"]),
        "lifecycle_missing": gate(
            "lifecycle_missing", 5, lifecycle_nodes=["/missing_lifecycle_node"]
        ),
        "hardware_inactive": gate(
            "hardware_inactive",
            8,
            hardware_components_active=True,
            controller_manager="/inactive_hardware/controller_manager",
        ),
    }
    scaled_gate = gate("scaled_timeout", 2, topics=["/never_published"])
    gates["scaled_timeout"] = scaled_gate
    sim_time_gate = gate("sim_time_without_clock", 2, topics=["/never_published"])
    gates["sim_time_without_clock"] = sim_time_gate
    exit_codes = {}
    exit_times = {}
    started = time.monotonic()

    def record(name):
        def on_exit(event, _context):
            exit_codes[name] = event.returncode
            exit_times[name] = time.monotonic() - started
            if len(exit_codes) == len(gates):
                return [EmitEvent(event=Shutdown(reason="all gates exited"))]
            return None

        return RegisterEventHandler(OnProcessExit(target_action=gates[name], on_exit=on_exit))

    python = [sys.executable, "-c"]
    description = LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": URDF}],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[str(controllers)],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
            ),
            ExecuteProcess(cmd=[*python, LIFECYCLE_NODE]),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace="inactive_hardware",
                parameters=[
                    {
                        "update_rate": 50,
                        "hardware_components_initial_state.inactive": ["MockSystem"],
                    }
                ],
                remappings=[("robot_description", "/robot_description")],
            ),
            *[record(name) for name in gates],
            *[
                gate_action
                for name, gate_action in gates.items()
                if name not in ("scaled_timeout", "sim_time_without_clock")
            ],
            GroupAction([SetLaunchConfiguration("stage_timeout_scale", "4"), scaled_gate]),
            GroupAction([SetParameter(name="use_sim_time", value=True), sim_time_gate]),
        ]
    )
    service = LaunchService()
    service.include_launch_description(description)
    service.run()

    assert exit_codes == {
        "controllers_ready": 0,
        "controller_missing": 1,
        "controller_wrong_state": 1,
        "topic_ready": 0,
        "topic_missing": 1,
        "lifecycle_ready": 0,
        "lifecycle_missing": 1,
        "hardware_inactive": 1,
        "scaled_timeout": 1,
        "sim_time_without_clock": 1,
    }
    # 2 s scaled by 4
    assert exit_times["scaled_timeout"] > 7.5


def test_states_other_than_inactive_are_expected_active(tmp_path):
    spec = importlib.util.spec_from_file_location(
        "stage_gate_script", Path(__file__).resolve().parent.parent / "scripts" / "stage_gate.py"
    )
    script = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(script)
    config = tmp_path / "controllers.yaml"
    config.write_text(
        CONTROLLERS + "        state: unconfigured\n"
        "      second_broadcaster:\n"
        "        type: joint_state_broadcaster/JointStateBroadcaster\n"
        "        state: INACTIVE\n"
    )
    assert script.expected_controller_states(str(config)) == {
        "joint_state_broadcaster": "active",
        "second_broadcaster": "inactive",
    }
