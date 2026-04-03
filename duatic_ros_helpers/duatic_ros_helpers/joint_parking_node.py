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


import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from duatic_ros_helpers.controller_helper import DuaticControllerHelper
from duatic_ros_helpers.robots_helper import DuaticRobotsHelper
from duatic_ros_helpers.jtc_helper import DuaticJTCHelper


class JointParkingNode(Node):
    """Move robot joints to predefined parking positions (home/sleep) using a safe staged motion.

    The node moves joints in two phases to avoid collisions:
      1. Move primary joints (e.g. flexion) to target
      2. Move secondary joints (e.g. rotation) to target

    All positions and joint categories are configurable via ROS parameters,
    making this node robot-agnostic.

    Parameters:
        home_position (float[]): Joint positions for the home pose
        sleep_position (float[]): Joint positions for the sleep pose
        primary_joint_indices (int[]): Indices of joints to move first (e.g. flexion joints)
        secondary_joint_indices (int[]): Indices of joints to move second (e.g. rotation joints)
        use_mirroring (bool): Mirror positions for the second arm (dual-arm setups)
        mirrored_joint_bases (string[]): Joint name substrings that get mirrored (negated)
        target_velocity_primary (double): Velocity for primary joints in rad/s
        target_velocity_secondary (double): Velocity for secondary joints in rad/s
    """

    def __init__(self):
        super().__init__("joint_parking_node")

        # Configurable parameters (no hardcoded robot-specific values)
        self.declare_parameter("home_position", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("sleep_position", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("primary_joint_indices", rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter("secondary_joint_indices", rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter("use_mirroring", False)
        self.declare_parameter("mirrored_joint_bases", rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter("target_velocity_primary", 0.8)
        self.declare_parameter("target_velocity_secondary", 1.2)

        self.home_position = list(self.get_parameter("home_position").value)
        self.sleep_position = list(self.get_parameter("sleep_position").value)
        self.primary_joint_indices = list(self.get_parameter("primary_joint_indices").value)
        self.secondary_joint_indices = list(self.get_parameter("secondary_joint_indices").value)
        self.use_mirroring = self.get_parameter("use_mirroring").value
        self.target_velocity_primary = self.get_parameter("target_velocity_primary").value
        self.target_velocity_secondary = self.get_parameter("target_velocity_secondary").value

        mirrored_param = self.get_parameter("mirrored_joint_bases").value
        self.mirrored_joint_bases = set(mirrored_param) if mirrored_param else set()

        self.tolerance = 0.01
        self.moving_states = {}
        self.home = False
        self.sleep = False
        self.controller_active = False
        self.joint_trajectory_publishers = {}
        self.topic_to_joint_names = {}
        self.topic_to_commanded_positions = {}
        self.previous_active_controllers = []

        # Subscriptions
        self.create_subscription(Bool, "move_home", self.move_home_callback, 10)
        self.create_subscription(Bool, "move_sleep", self.move_sleep_callback, 10)

        # Helper classes
        self.duatic_controller_helper = DuaticControllerHelper(self)
        self.duatic_controller_helper.wait_for_controller_data()

        self.duatic_robots_helper = DuaticRobotsHelper(self)
        self.duatic_robots_helper.wait_for_robot()

        duatic_jtc_helper = DuaticJTCHelper(self)
        self.arms = self.duatic_robots_helper.get_component_names("arm")
        found_topics = duatic_jtc_helper.find_topics_for_controller(
            "joint_trajectory_controller", "joint_trajectory", self.arms
        )

        response = duatic_jtc_helper.process_topics_and_extract_joint_names(found_topics)
        self.topic_to_joint_names = response[0]
        self.topic_to_commanded_positions = response[1]

        # Set the dt value depending on simulation or real hardware
        self.dt = self.duatic_robots_helper.get_dt()

        # Calculate step sizes based on dt to achieve target velocities
        self.step_size_primary = self.target_velocity_primary * self.dt
        self.step_size_secondary = self.target_velocity_secondary * self.dt

        # Create publishers for each joint trajectory topic
        for topic in self.topic_to_joint_names.keys():
            self.joint_trajectory_publishers[topic] = self.create_publisher(
                JointTrajectory, topic, 10
            )
            self.get_logger().debug(f"Created publisher for topic: {topic}")

        self.create_timer(self.dt, self.control_loop)

    def reset_state(self):
        """Reset the state of the node."""
        if self.sleep or self.home:
            return

        for topic, _ in self.topic_to_joint_names.items():
            self.moving_states[topic] = "none"

    def move_home_callback(self, msg):
        self.home = msg.data
        self.sleep = False
        self.reset_state()

    def move_sleep_callback(self, msg):
        self.sleep = msg.data
        self.home = False
        self.reset_state()

    def control_loop(self):
        """Main control loop to handle movement commands."""
        if self.duatic_controller_helper.is_freeze_active():
            return

        if self.home or self.sleep:
            if not self.controller_active:
                self.controller_active = True
                self.switch_to_joint_trajectory_controllers()

            self._move_to_target()
        else:
            if (
                self.controller_active
                and "joint_trajectory_controller" not in self.previous_active_controllers
            ):
                self.controller_active = False
                self.switch_to_previous_controllers()
            else:
                self.previous_active_controllers = list(
                    self.duatic_controller_helper.get_active_controllers()
                )

    def _move_to_target(self):
        """Staged movement logic: primary joints first, then secondary joints."""
        arm_index = 0

        for topic, joint_names in self.topic_to_joint_names.items():
            if topic not in self.moving_states:
                self.moving_states[topic] = "none"

            all_joint_states = self.duatic_robots_helper.get_joint_states()
            current_joint_values = [
                all_joint_states.get(joint_name, 0.0) for joint_name in joint_names
            ]

            # Determine target position based on mirroring
            if self.use_mirroring and arm_index == 1:
                target_home = self.mirror_position(joint_names, self.home_position.copy())
                target_sleep = self.mirror_position(joint_names, self.sleep_position.copy())
            else:
                target_home = self.home_position.copy()
                target_sleep = self.sleep_position.copy()

            # Create intermediate position: home position with sleep secondary joints
            target_sleep_secondary = target_home.copy()
            for i in self.secondary_joint_indices:
                target_sleep_secondary[i] = target_sleep[i]

            # State machine logic per arm
            at_home_primary = self.joints_at_pose(
                current_joint_values, self.primary_joint_indices, target_home
            )
            at_sleep_primary = self.joints_at_pose(
                current_joint_values, self.primary_joint_indices, target_sleep
            )
            at_home_secondary = self.joints_at_pose(
                current_joint_values, self.secondary_joint_indices, target_home
            )
            at_sleep_secondary = self.joints_at_pose(
                current_joint_values, self.secondary_joint_indices, target_sleep
            )

            current_state = self.moving_states[topic]
            commanded_positions = current_joint_values.copy()

            self.get_logger().debug(
                f"Arm {arm_index}: State={current_state}, "
                f"at_home_primary={at_home_primary}, at_sleep_primary={at_sleep_primary}, "
                f"at_home_secondary={at_home_secondary}, at_sleep_secondary={at_sleep_secondary}"
            )

            if self.sleep:
                if at_sleep_primary and at_sleep_secondary:
                    self.moving_states[topic] = "at_sleep"
                elif at_sleep_secondary:
                    self.moving_states[topic] = "going_sleep_primary"
                elif at_home_primary:
                    self.moving_states[topic] = "going_sleep_secondary"
                elif not at_sleep_secondary:
                    self.moving_states[topic] = "going_home_primary"
                else:
                    self.moving_states[topic] = "error"
            elif self.home:
                if at_home_primary and at_home_secondary:
                    self.moving_states[topic] = "at_home"
                elif not at_home_primary:
                    self.moving_states[topic] = "going_home_primary"
                elif not at_home_secondary:
                    self.moving_states[topic] = "going_home_secondary"
                else:
                    self.moving_states[topic] = "error"

            current_state = self.moving_states[topic]

            if current_state in ("at_sleep", "at_home"):
                pass
            elif current_state == "going_sleep_primary":
                commanded_positions = self.move_primary_only(target_sleep, current_joint_values)
            elif current_state == "going_sleep_secondary":
                commanded_positions = self.move_secondary_only(
                    target_sleep_secondary, current_joint_values
                )
            elif current_state == "going_home_primary":
                commanded_positions = self.move_primary_only(target_home, current_joint_values)
            elif current_state == "going_home_secondary":
                commanded_positions = self.move_secondary_only(target_home, current_joint_values)
            else:
                self.get_logger().error(f"Arm {arm_index}: Invalid state '{current_state}'")
                commanded_positions = current_joint_values.copy()

            self.topic_to_commanded_positions[topic] = commanded_positions
            arm_index += 1

        # Publish trajectories for all arms
        for topic, publisher in self.joint_trajectory_publishers.items():
            if self.moving_states.get(topic) in ("at_sleep", "at_home"):
                continue

            self.publish_joint_trajectory(
                self.topic_to_commanded_positions[topic],
                publisher=publisher,
                joint_names=self.topic_to_joint_names[topic],
            )

    def mirror_position(self, joint_names, target_position):
        """Mirror the joint positions for mirrored joints based on the configuration."""
        mirrored_indices = [
            i
            for i, name in enumerate(joint_names)
            if any(base in name for base in self.mirrored_joint_bases)
        ]
        for i in mirrored_indices:
            target_position[i] = -target_position[i]
        return target_position

    def joints_at_pose(self, current, indices, target_position):
        """Check if the current joint angles are at the target position for the specified indices."""
        return all(abs(current[i] - target_position[i]) < self.tolerance for i in indices)

    def interpolate_partial(self, current, target, indices, step_size):
        """Interpolate the current joint values towards the target for specified indices."""
        next_step = current[:]
        for i in indices:
            delta = target[i] - current[i]
            if abs(delta) > step_size:
                next_step[i] = current[i] + step_size * (1 if delta > 0 else -1)
            else:
                next_step[i] = target[i]
        return next_step

    def move_primary_only(self, target_position, current_joint_values):
        return self.interpolate_partial(
            current_joint_values.copy(),
            target_position,
            self.primary_joint_indices,
            self.step_size_primary,
        )

    def move_secondary_only(self, target_position, current_joint_values):
        return self.interpolate_partial(
            current_joint_values.copy(),
            target_position,
            self.secondary_joint_indices,
            self.step_size_secondary,
        )

    def publish_joint_trajectory(self, target_positions, publisher, joint_names=None):
        """Publishes a joint trajectory message for the given positions."""
        if joint_names is None:
            joint_names = list(self.duatic_robots_helper.get_joint_states().keys())

        if not joint_names:
            self.get_logger().error("No joint names available. Cannot publish trajectory.")
            return

        if not target_positions:
            self.get_logger().error("No trajectory points available to publish.")
            return

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(joint_names)
        point.accelerations = [0.0] * len(joint_names)
        time_in_sec = self.dt
        sec = int(time_in_sec)
        nanosec = int((time_in_sec - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        trajectory_msg.points.append(point)
        publisher.publish(trajectory_msg)

    def switch_to_joint_trajectory_controllers(self):
        """Switch to all joint trajectory controllers if not already active."""
        all_controllers = self.duatic_controller_helper.get_all_controllers()
        controllers_to_activate = []
        controllers_to_deactivate = []

        for controller_type, controllers in all_controllers.items():
            for controller in controllers:
                for name, active_state in controller.items():
                    if (
                        controller_type == "joint_trajectory_controller"
                        and active_state == "inactive"
                    ):
                        controllers_to_activate.append(name)
                    elif (
                        controller_type == "joint_trajectory_controller"
                        and active_state == "active"
                    ):
                        pass
                    elif active_state == "inactive":
                        pass
                    else:
                        controllers_to_deactivate.append(name)

        self.duatic_controller_helper.switch_controller(
            controllers_to_activate, controllers_to_deactivate
        )

    def switch_to_previous_controllers(self):
        """Deactivate all joint trajectory controllers."""
        all_controllers = self.duatic_controller_helper.get_all_controllers()
        controllers_to_activate = []
        controllers_to_deactivate = []

        if not self.previous_active_controllers or len(self.previous_active_controllers) <= 0:
            self.get_logger().debug(
                "No previous controller state found. Deactivating joint trajectory controllers."
            )
            for controller_type, controllers in all_controllers.items():
                for controller in controllers:
                    for name, active_state in controller.items():
                        if "joint_trajectory_controller" in name and active_state == "active":
                            controllers_to_deactivate.append(name)
        else:
            for controller_type, controllers in all_controllers.items():
                for controller in controllers:
                    for name, active_state in controller.items():
                        if "joint_trajectory_controller" in name and active_state == "active":
                            if name not in self.previous_active_controllers:
                                controllers_to_deactivate.append(name)
                        elif name in self.previous_active_controllers:
                            controllers_to_activate.append(name)

        self.get_logger().debug(
            f"Switching to previous controllers: {controllers_to_activate} "
            f"and deactivating: {controllers_to_deactivate}"
        )
        self.duatic_controller_helper.switch_controller(
            controllers_to_activate, controllers_to_deactivate
        )


def main(args=None):

    rclpy.init(args=args)

    node = JointParkingNode()
    node.get_logger().info("Joint Parking Node is running...")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
