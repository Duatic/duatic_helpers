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

from __future__ import annotations

import re
import time
import xml.etree.ElementTree as ET
from typing import TYPE_CHECKING

import rclpy

from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListHardwareComponents
from duatic_helpers.duatic_param_helper import DuaticParamHelper

if TYPE_CHECKING:
    from typing import Any

    from rclpy.node import Node


class DuaticRobotsHelper:
    """
    Helper class that tracks the joint states of the robot and detects its structure.

    Identifies the robot morphology from the URDF, groups the joints seen on 'joint_states'
    into robots and components (arms, platform, ...) by their names, and provides access to
    them.
    """

    def __init__(self, node: Node) -> None:
        """
        Construct the helper, subscribe to 'joint_states' and read the URDF.

        :param node: Node used for the subscription, service client and logging.
        """
        self.node = node

        # Joint name to position
        self._joint_states: dict[str, float] = {}
        # Robot id to component name to {"type": component type, "joints": joint names}
        self._robot: dict[str, dict[str, dict[str, Any]]] = {}
        self._robot_count = 0
        self._expected_joints: set[str] = set()

        self._end_effector_keywords = (
            "finger",
            "gripper",
            "claw",
            "tool",
            "effector",
            "hand",
            "pinch",
        )

        self._joint_states_subscription = self.node.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 10
        )

        self.hardware_components_client = self.node.create_client(
            ListHardwareComponents, "controller_manager/list_hardware_components"
        )

        self.param_helper = DuaticParamHelper(self.node)
        self.component_type_indicators = {
            "arm": ["shoulder", "elbow", "forearm", "wrist"],
            "hip": ["hip"],
            "platform": ["wheel"],
            "head": ["head"],
            "end_effector": list(self._end_effector_keywords),
        }

        self.robot_structure = "unknown"
        self._initialize_from_urdf()

    def _initialize_from_urdf(self) -> None:
        """
        Try to identify robot structure from URDF.

        Sets the robot structure to one of 'mobile_manipulator', 'multi_arm', 'mobile_base',
        'single_arm' or 'generic', and stores the joints expected on 'joint_states'. Both are
        left unchanged if the URDF cannot be read.
        """
        urdf_raw = self.param_helper.get_urdf()
        if urdf_raw:
            self._expected_joints = self._movable_joint_names(urdf_raw)
            # Strip XML comments to ensure keyword matching only hits joint/link names
            urdf = re.sub(r"<!--.*?-->", "", urdf_raw, flags=re.DOTALL).lower()
            # Priority based identification to avoid mis-detecting rovers with arm metadata
            # Morphology-based identification
            has_wheels = "wheel" in urdf
            has_arm = "shoulder" in urdf or "elbow" in urdf
            if has_wheels and has_arm:
                self.robot_structure = "mobile_manipulator"
                self.node.get_logger().info(
                    "Identified robot structure: Mobile Manipulator (Base + Arm/Torso)"
                )
            elif "hip" in urdf:
                self.robot_structure = "multi_arm"
                self.node.get_logger().info("Identified robot structure: Multi-Arm (Torso)")
            elif "wheel" in urdf:
                self.robot_structure = "mobile_base"
                self.node.get_logger().info("Identified robot structure: Mobile Base")
            elif "shoulder" in urdf or "elbow" in urdf:
                self.robot_structure = "single_arm"
                self.node.get_logger().info("Identified robot structure: Single-Arm")
            else:
                self.robot_structure = "generic"
                self.node.get_logger().info("Identified robot structure: Generic")
        else:
            self.node.get_logger().warning(
                "Could not read robot_description: without it there is no expected joint "
                "set, so wait_for_robot() cannot confirm the robot is complete."
            )

    def _movable_joint_names(self, urdf_raw: str) -> set[str]:
        """
        The joint names a complete /joint_states is expected to carry.

        Listed by type rather than by exclusion: revolute, continuous and prismatic are the
        joints that report one position on the topic. Fixed has no state at all, floating and
        planar have several degrees of freedom and no single position, and a mimic joint
        is computed from the joint it follows rather than measured. An unparsable URDF
        yields an empty set, which wait_for_robot() reports as a check it could not run.

        :param urdf_raw: The URDF as XML string.

        :returns: Names of the joints expected on /joint_states.
        """
        try:
            root = ET.fromstring(urdf_raw)
        except ET.ParseError as e:
            self.node.get_logger().warning(f"Could not parse the URDF to list its joints: {e}")
            return set()

        # Direct children only: <transmission> and <ros2_control> blocks nest their own
        # <joint> elements, and those are wiring, not joints to wait for.
        names = set()
        for joint in root.findall("joint"):
            name = joint.get("name")
            if not name or joint.get("type") not in ("revolute", "continuous", "prismatic"):
                continue
            if joint.find("mimic") is not None:
                continue
            names.add(name)
        return names

    def _missing_joints(self) -> set[str]:
        """
        URDF joints not yet seen on /joint_states.

        :returns: Names of the expected joints that have not been received yet.
        """
        return self._expected_joints - set(self._joint_states)

    def _joint_state_callback(self, msg: JointState) -> None:
        """
        Update the JointState cache and robot type.

        The cache only grows: a publisher that goes away leaves its last values behind.

        :param msg: The received joint state. Messages without positions, or with a different
            number of names and positions, are ignored.
        """
        if not msg.position:
            # Legal per sensor_msgs/JointState: a publisher may send only velocity or
            # effort. Nothing here to merge, and nothing wrong with it either.
            return

        if len(msg.position) != len(msg.name):
            self.node.get_logger().warning(
                f"Ignoring JointState with {len(msg.name)} names and "
                f"{len(msg.position)} positions: arrays must have equal length",
                throttle_duration_sec=10.0,
            )
            return
        # Merged, not assigned: /joint_states may have several publishers and they need
        # not each carry the whole robot, so replacing the cache lets one partial message
        # hide everyone else's joints.
        #
        # Copy and rebind rather than update in place. get_joint_states() hands out this
        # very dict and callers iterate it from other callbacks — with a
        # MultiThreadedExecutor that is a mutation under an iterator. A rebind gives every
        # reader a consistent snapshot; the copy costs nothing at a few dozen joints.
        previous = self._joint_states
        merged = dict(previous)
        merged.update(zip(msg.name, msg.position))
        self._joint_states = merged

        # Redo robot type detection, on every new joint name.
        if len(merged) != len(previous):
            self._robot = self.get_robots_with_components()
            self._robot_count = len(self._robot)

    def get_joint_states(self) -> dict[str, float]:
        """
        Returns a dictionary of joint names and their positions.

        :returns: Snapshot of the joint positions by joint name. It is replaced, not modified, on
            new messages, so it is safe to iterate.
        """
        return self._joint_states

    def get_joint_value_from_states(self, joint_name: str) -> float:
        """
        Get the current position value for a specific joint from joint states.

        :param joint_name: Name of the joint.

        :returns: The joint position, or 0.0 (with a warning) if the joint has not been seen.
        """
        if joint_name in self._joint_states:
            return self._joint_states[joint_name]
        else:
            self.node.get_logger().warning(f"Joint '{joint_name}' not found in joint states")
            return 0.0

    def get_component_names(self, filter_by_type: str | None = None) -> list[str]:
        """
        Returns a list of component names, optionally filtered by type.

        :param filter_by_type: Only return components of this type (e.g. 'arm', 'platform').
            If None, all components are returned.

        :returns: The component names, empty if no robot was detected yet.
        """
        if self._robot_count <= 0:
            self.node.get_logger().error("No robots detected. Cannot get component names.")
            return []

        component_names = []
        for robot_id, components in self._robot.items():
            for component_name, component_info in components.items():
                if filter_by_type is None:
                    component_names.append(component_name)
                else:
                    found_component = component_info["type"] == filter_by_type
                    if found_component:
                        component_names.append(component_name)

        return component_names

    def get_robots_with_components(self) -> dict[str, dict[str, dict[str, Any]]]:
        """
        Returns a dictionary of robots with their detected components.

        :returns: Mapping from robot id to component name to a dictionary with the component 'type'
            and the sorted names of its 'joints'. Empty if no joint states were received.
        """
        if not self._joint_states:
            self.node.get_logger().error("No joint states received. Cannot detect robots.")
            return {}

        robots = {}

        # Analyze joint names to identify robots and their components
        for joint_name in self._joint_states.keys():
            robot_id, component_name, component_type = self._parse_joint_name(joint_name)

            # Initialize robot structure if not exists
            if robot_id not in robots:
                robots[robot_id] = {}

            # Initialize specific component if not exists
            if component_name not in robots[robot_id]:
                robots[robot_id][component_name] = {"type": component_type, "joints": []}

            # Add joint to component
            robots[robot_id][component_name]["joints"].append(joint_name)

        # Sort joints within each component for consistency
        for robot_id in robots:
            for component_name in robots[robot_id]:
                robots[robot_id][component_name]["joints"].sort()

        self.node.get_logger().debug(f"Detected robot structure: {robots}")
        return robots

    def _parse_joint_name(self, joint_name: str) -> tuple[str, str, str]:
        """
        Parse a joint name to extract robot ID, component name, and component type.

        :param joint_name: Name of the joint, e.g. 'arm_left/shoulder_rotation'.

        :returns: Tuple of the robot id, the component name and the component type.
        """
        # Handle different joint naming patterns

        # Pattern 1: arm_left/shoulder_rotation, arm_right/elbow_flexion
        if "/" in joint_name:
            prefix, joint_suffix = joint_name.split("/", 1)
            if prefix.startswith("arm_") or prefix.startswith("hand_"):
                return "robot_0", prefix, "arm"

            # Check if suffix matches any component indicator
            for comp_type, indicators in self.component_type_indicators.items():
                if any(indicator in joint_suffix for indicator in indicators):
                    return "robot_0", prefix, comp_type

        # Pattern 2: Heuristic based on joint name keywords (Single Arm or other flat structures)
        for comp_type, indicators in self.component_type_indicators.items():
            if any(indicator in joint_name for indicator in indicators):
                # For flat structures, we return "" as component name if it's an arm
                # to match '/joint_trajectory_controller/joint_trajectory'
                comp_name = "" if comp_type == "arm" else comp_type
                return "robot_0", comp_name, comp_type

        # Default: treat as miscellaneous component
        return "robot_0", "misc", "misc"

    def get_component_count(self, component_type: str = "arm") -> int:
        """
        Get the number of components detected.

        :param component_type: Type of the components to count (e.g. 'arm', 'platform').

        :returns: The number of components of this type, 0 if no robot was detected yet.
        """
        if self._robot_count <= 0:
            self.node.get_logger().error("No robots detected. Cannot get component count.")
            return 0

        component_count = 0
        for robot_id, components in self._robot.items():
            for component_name, component_info in components.items():
                if component_info["type"] == component_type:
                    component_count += 1

        return component_count

    def wait_for_robot(self, timeout_sec: float = 30.0) -> bool:
        """
        Block until every joint the URDF declares has been seen on /joint_states.

        Waiting for the first message alone is not enough: it proves one publisher is up,
        not that all the components are among them.

        :param timeout_sec: How long to wait for the full joint set, in seconds.

        :returns: True once every expected joint has arrived, False on timeout.
        """
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            if self._expected_joints and self._robot_count > 0 and not self._missing_joints():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)

        if self._robot_count <= 0:
            self.node.get_logger().error(
                f"No joint states within {timeout_sec:.1f}s: no robot detected"
            )
        elif not self._expected_joints:
            self.node.get_logger().warning(
                f"robot_description unreadable within {timeout_sec:.1f}s. "
                "Continuing with a robot that may be missing components."
            )
        else:
            self.node.get_logger().warning(
                f"Only {len(self._joint_states)} of {len(self._expected_joints)} joints within "
                f"{timeout_sec:.1f}s, continuing with a partial robot. Never seen: "
                f"{sorted(self._missing_joints())}"
            )
        return False

    def get_component_joint_names(
        self,
        robot_id: str = "robot_0",
        component_type: str = "arms",
        component_name: str | None = None,
    ) -> list[str]:
        """
        Get joints for a specific component.

        Note: _is_robot_ready() never returns True, so this currently always returns an empty
        list.

        :param robot_id: Id of the robot.
        :param component_type: Key of the component group in the robot's components.
        :param component_name: Name of the component.

        :returns: The joint names of the component, empty if it is not ready.
        """
        if not self._is_robot_ready(robot_id, component_type, component_name):
            return []

        return self._robot[robot_id][component_type][component_name]["joints"]

    def get_component_joint_states(
        self,
        robot_id: str = "robot_0",
        component_type: str = "arms",
        component_name: str | None = None,
    ) -> dict[str, float]:
        """
        Returns the joint_states for a component.

        Note: _is_robot_ready() never returns True, so this currently always returns an empty
        dictionary.

        :param robot_id: Id of the robot.
        :param component_type: Key of the component group in the robot's components.
        :param component_name: Name of the component.

        :returns: Joint positions by joint name for the component's joints, empty if it is not ready.
        """
        if not self._is_robot_ready(robot_id, component_type, component_name):
            return {}

        joint_states = self.get_joint_states()
        joints = self._robot[robot_id][component_type][component_name]["joints"]
        return {joint: joint_states[joint] for joint in joints if joint in joint_states}

    def _is_robot_ready(
        self,
        robot_id: str = "robot_0",
        component_type: str = "arms",
        component_name: str | None = None,
    ) -> bool | None:
        """
        Check if the robot is ready by verifying joint states.

        :param robot_id: Id of the robot.
        :param component_type: Key of the component group in the robot's components.
        :param component_name: Name of the component.

        :returns: False (with an error log) if the robot or component was not found. Falls through
            to None if all checks pass, so the result is falsy in every case.
        """
        if self._robot_count <= 0:
            self.node.get_logger().error("No robots detected. Cannot get component joint states.")
            return False

        if robot_id not in self._robot:
            self.node.get_logger().error(f"Robot {robot_id} not found")
            return False

        if component_type not in self._robot[robot_id]:
            self.node.get_logger().error(f"Component type {component_type} not found in {robot_id}")
            return False

        if component_name not in self._robot[robot_id][component_type]:
            self.node.get_logger().error(f"Component {component_name} not found")
            return False

    def check_simulation_mode(self) -> bool:
        """
        Detect if we're running in simulation or real hardware mode.

        Checks the plugin names of the hardware components for 'mock', 'fake' and 'gazebo'.

        :returns: True if a simulated hardware component was found, False for real hardware or if
            the hardware components could not be listed.
        """
        is_simulation = False

        if not self.hardware_components_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().debug("Hardware components service not available")
            return is_simulation

        # Call the service
        request = ListHardwareComponents.Request()
        future = self.hardware_components_client.call_async(request)

        # Wait for response
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        if future.result() is None:
            self.node.get_logger().debug("Hardware components service call failed")
            return False

        response = future.result()
        for component in response.component:
            plugin_name = component.plugin_name.lower()

            # Check plugin name for mock hardware indicators
            if "mock" in plugin_name:
                self.node.get_logger().debug(f"Mock hardware detected: {component.plugin_name}")
                is_simulation = True
            elif "fake" in plugin_name:
                self.node.get_logger().debug(f"Fake hardware detected: {component.plugin_name}")
                is_simulation = True
            elif "gazebo" in plugin_name:
                self.node.get_logger().debug(f"Gazebo hardware detected: {component.plugin_name}")
                is_simulation = True
            elif any(real_hw in plugin_name for real_hw in ["real"]):
                self.node.get_logger().debug(f"Real hardware detected: {component.plugin_name}")
                is_simulation = False

        return is_simulation

    def get_dt(self) -> float:
        """
        Get the control period, depending on whether running in simulation or on hardware.

        Also stores the result of the simulation check and the period as the attributes
        is_simulation and dt.

        :returns: The period in seconds: 0.05 in simulation, 0.001 on real hardware.
        """
        self.is_simulation = self.check_simulation_mode()

        if self.is_simulation:
            self.dt = 0.05
            self.node.get_logger().info("Using simulation timing: dt=0.05s (20Hz)")
        else:
            self.dt = 0.001
            self.node.get_logger().info("Using real hardware timing: dt=0.001s (1000Hz)")

        return self.dt  # Return the dt value
