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

import os
import xacro
import tempfile
from typing import TYPE_CHECKING

import numpy as np
import pinocchio as pin
from duatic_helpers.duatic_param_helper import DuaticParamHelper

from geometry_msgs.msg import PoseStamped
from ament_index_python import get_package_share_directory

if TYPE_CHECKING:
    from collections.abc import Sequence
    from typing import TypeAlias

    import numpy.typing as npt
    from rclpy.node import Node

    # Joint positions, either by joint name or in the order of the model's configuration vector
    JointValues: TypeAlias = "dict[str, float] | Sequence[float] | npt.NDArray"


class DuaticPinocchioHelper:
    """
    Helper class that builds a Pinocchio model of the robot and computes its kinematics.

    The model is built from the URDF on the parameter server, with a DynaArm example URDF as
    fallback. Provides forward kinematics, an iterative inverse kinematics solver and
    conversions between ROS poses and Pinocchio types.
    """

    def __init__(self, node: Node, robot_id: str = "robot", joint_margins: float = 0.2) -> None:
        """
        Construct the helper and load the Pinocchio model.

        Args:
            node: Node used for parameter access and logging.
            robot_id: Identifier of the robot, stored as an attribute.
            joint_margins: Margin to the model's position limits that the IK solver keeps, in
                rad (or m for prismatic joints).
        """
        self.node = node
        self.robot_id = robot_id
        self.param_helper = DuaticParamHelper(self.node)

        self.model = self.get_pin_model_data()
        self.data = self.model.createData()

        self.lower = self.model.lowerPositionLimit + joint_margins
        self.upper = self.model.upperPositionLimit - joint_margins

    def get_fk(
        self,
        current_joint_values: JointValues,
        frame: str,
        target_frame: str | None = None,
    ) -> pin.SE3:
        """
        Compute forward kinematics for the specified frame.

        Args:
            current_joint_values: Joint positions, either by joint name or as a vector in the
                order of the model's configuration vector (padded with zeros if too short).
            frame: Name of the frame to compute the pose of.
            target_frame: If given, the pose is expressed in this frame instead of the model
                root frame. If the transformation fails, the pose in the model root frame is
                returned.

        Returns:
            Pose of the frame in the target frame, or in the model root frame if no target
            frame is given.
        """
        # Convert input to proper numpy array
        q = self._convert_joint_values_to_array(current_joint_values)

        # Compute FK using only the model DOFs
        pin.forwardKinematics(self.model, self.data, q[: self.model.nq])
        pin.updateFramePlacements(self.model, self.data)

        # Get FK for the given frame
        frame_id = self.model.getFrameId(frame)
        pose = self.data.oMf[frame_id]  # Pose in model root frame

        # Debug: Log the target frame pose if it's tbase
        if target_frame == "tbase":
            try:
                target_frame_id = self.model.getFrameId(target_frame)
                target_pose = self.data.oMf[target_frame_id]
            except Exception as e:
                self.node.get_logger().warn(f"Could not get tbase frame: {e}")

        # Transform to target frame if specified
        if target_frame is not None:
            try:
                target_frame_id = self.model.getFrameId(target_frame)
                target_pose = self.data.oMf[target_frame_id]  # Target frame pose in model root

                # Transform: ee_pose_in_target = target_pose.inverse() * ee_pose
                ee_pose = target_pose.inverse() * pose
                return ee_pose
            except Exception as e:
                self.node.get_logger().warn(f"Could not transform to frame '{target_frame}': {e}")
                # Fall back to model root frame

        return pose

    def get_fk_as_pose_stamped(
        self,
        current_joint_values: JointValues,
        frame: str,
        target_frame: str | None = None,
    ) -> PoseStamped:
        """
        Compute forward kinematics for the specified frame as a PoseStamped message.

        The header of the returned message is left empty.

        Args:
            current_joint_values: Joint positions, see get_fk().
            frame: Name of the frame to compute the pose of.
            target_frame: Frame to express the pose in, see get_fk().

        Returns:
            Pose of the frame as returned by get_fk().
        """
        current_pose = self.get_fk(current_joint_values, frame, target_frame)

        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = current_pose.translation[0]
        pose_stamped.pose.position.y = current_pose.translation[1]
        pose_stamped.pose.position.z = current_pose.translation[2]

        quat = pin.Quaternion(current_pose.rotation)
        pose_stamped.pose.orientation.x = quat.x
        pose_stamped.pose.orientation.y = quat.y
        pose_stamped.pose.orientation.z = quat.z
        pose_stamped.pose.orientation.w = quat.w

        return pose_stamped

    def solve_ik(
        self,
        current_joint_values: JointValues,
        target_SE3: pin.SE3,
        frame: str,
        target_frame: str | None,
        iterations: int = 100,
        threshold: float = 0.01,
    ) -> tuple[npt.NDArray[np.floating], npt.NDArray[np.floating]]:
        """
        Solve IK for the specified frame, but work with full joint configuration.

        Iterates pseudo-inverse steps on the frame Jacobian (with a gain of 0.1), clipping the
        joints to the model limits shrunk by the joint margins.

        Args:
            current_joint_values: Joint positions to start from, see get_fk().
            target_SE3: Target pose of the frame, expressed in the target frame (or in the
                model root frame if no target frame is given).
            frame: Name of the frame to move to the target pose.
            target_frame: Frame the target pose is expressed in, or None for the model root
                frame.
            iterations: Maximum number of iterations.
            threshold: Norm of the (rotation-scaled) pose error below which the solver stops.

        Returns:
            Tuple of the full joint configuration vector and the pose error (6D, rotation part
            scaled) of the last iteration.
        """
        error = np.inf

        # Convert input to proper numpy array (always full configuration)
        q = self._convert_joint_values_to_array(current_joint_values)

        frame_id = self.model.getFrameId(frame)

        # Get the actual DOF count from a test Jacobian computation
        pin.forwardKinematics(self.model, self.data, q[: self.model.nq])
        pin.updateFramePlacements(self.model, self.data)
        J_test = pin.computeFrameJacobian(
            self.model, self.data, q[: self.model.nq], frame_id, pin.LOCAL
        )
        actual_dof = J_test.shape[1]

        # Create weight matrix with correct dimensions (actual DOF, not model.nq)
        joint_weights = np.ones(actual_dof)
        W = np.diag(joint_weights)

        for i in range(iterations):
            # Update model data with current joint configuration
            pin.forwardKinematics(self.model, self.data, q[: self.model.nq])
            pin.updateFramePlacements(self.model, self.data)

            # Compute error using full configuration
            error = self.get_pose_error(q, target_SE3, frame, target_frame)
            if np.linalg.norm(error) < threshold:
                break

            # Compute Jacobian for full model
            J = pin.computeFrameJacobian(
                self.model, self.data, q[: self.model.nq], frame_id, pin.LOCAL
            )

            # Now J has shape (6, actual_dof) and W has shape (actual_dof, actual_dof)
            J_w = J @ W
            dq = -0.1 * W @ np.linalg.pinv(J_w) @ error

            # Apply the change only to the actual DOFs
            q[:actual_dof] += dq

            # Use only the model limits for clipping (but only for actual DOFs)
            if len(self.lower) >= actual_dof and len(self.upper) >= actual_dof:
                q[:actual_dof] = np.clip(
                    q[:actual_dof], self.lower[:actual_dof], self.upper[:actual_dof]
                )

        return q, error

    def get_pose_error(
        self,
        q: JointValues,
        target_SE3: pin.SE3,
        frame: str,
        target_frame: str | None = None,
    ) -> npt.NDArray[np.floating]:
        """
        Compute pose error between current and target pose.

        Args:
            q: Joint positions, see get_fk().
            target_SE3: Target pose of the frame, expressed in the target frame.
            frame: Name of the frame to compute the error of.
            target_frame: Frame the poses are expressed in, or None for the model root frame.

        Returns:
            Pose error as 6D vector (translation, then rotation scaled by 0.2 to balance the
            two parts), as the logarithm of the transform from target to current pose.
        """
        current_SE3 = self.get_fk(q, frame, target_frame)
        error = pin.log(target_SE3.inverse() * current_SE3).vector
        # Scale rotation part (last 3 elements) to balance translation/rotation error
        rot_scale = 0.2  # Try 0.1–0.2 for typical arms
        error[3:] *= rot_scale
        return error

    def convert_pose_stamped_to_se3(self, pose_stamped: PoseStamped) -> pin.SE3:
        """
        Convert a PoseStamped message to a Pinocchio SE3 object.

        Args:
            pose_stamped: Pose to convert. Its header is ignored.

        Returns:
            The pose as SE3.
        """
        pos = pose_stamped.pose.position
        ori = pose_stamped.pose.orientation
        translation = np.array([pos.x, pos.y, pos.z])
        quat = np.array([ori.w, ori.x, ori.y, ori.z])
        return pin.SE3(pin.Quaternion(*quat).matrix(), translation)

    def get_pin_model_data(self) -> pin.Model:
        """
        Build the Pinocchio model from the robot URDF.

        Uses the URDF from the parameter server, falling back to the DynaArm example URDF.

        Returns:
            The Pinocchio model, also stored as the model attribute.
        """
        # Try to get URDF from parameter server first (preferred)
        urdf_xml = self.get_robot_urdf_from_param()

        if urdf_xml:
            # Use URDF from parameter server
            urdf_file_path = self._write_urdf_to_temp_file(urdf_xml)
        else:
            # Fallback to hardcoded path if parameter server fails
            self.node.get_logger().warn("URDF not found on parameter server, using fallback URDF")
            urdf_file_path = self.get_dynaarm_urdf()

        # Load model with Pinocchio and clean it up
        self.model = pin.buildModelFromUrdf(urdf_file_path)

        self.node.get_logger().info(
            f"Pinocchio model loaded with {self.model.nq} DOF, {len(self.model.joints)} joints and {len(self.model.frames)} frames."
        )

        return self.model

    def _write_urdf_to_temp_file(self, urdf_xml: str) -> str:
        """
        Write URDF XML string to a temporary file.

        Args:
            urdf_xml: The URDF as XML string.

        Returns:
            Path of the temporary file, which is not deleted automatically.
        """
        with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as urdf_file:
            urdf_file.write(urdf_xml.encode())
            return urdf_file.name

    def get_dynaarm_urdf(self) -> str | None:
        """
        Fallback method to get default DynaArm URDF if parameter server fails.

        Returns:
            Path of a temporary file with the URDF, or None if it could not be loaded.
        """
        package_share_desc_name = "dynaarm_single_example_description"
        urdf_name = "dynaarm_single_example.urdf.xacro"

        # Load the robot description
        try:
            pkg_share_description = get_package_share_directory(package_share_desc_name)
            xacro_path = os.path.join(pkg_share_description, "urdf", urdf_name)
            urdf_xml = xacro.process_file(xacro_path).toxml()

            # Write URDF to a temporary file
            with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as urdf_file:
                urdf_file.write(urdf_xml.encode())
                urdf_file_path = urdf_file.name
                return urdf_file_path
        except Exception as e:
            self.node.get_logger().error(
                f"Failed to load fallback URDF from {package_share_desc_name}: {e}"
            )
            return None

    def get_robot_urdf_from_param(self) -> str | None:
        """
        Retrieve the robot URDF from the parameter server.

        Returns:
            The URDF XML string, or None if it could not be retrieved.
        """
        return self.param_helper.get_urdf("robot_state_publisher", "robot_description")

    def get_joint_values_by_names(
        self, q: Sequence[float], joint_names: Sequence[str]
    ) -> list[float]:
        """
        Extract specific joint values from q array by joint names.

        Indexes q by the position of the joint in the model's joint list, i.e. assumes
        single-DOF joints.

        Args:
            q: Joint positions in the order of the model's joints.
            joint_names: Names of the joints to extract.

        Returns:
            The values of the requested joints, 0.0 for joints that are not in the model.
        """
        joint_values = []
        all_joint_names = [name for name in self.model.names[1:]]  # Skip universe joint

        for joint_name in joint_names:
            try:
                joint_index = all_joint_names.index(joint_name)
                joint_values.append(q[joint_index])
            except ValueError:
                self.node.get_logger().warn(f"Joint '{joint_name}' not found in model")
                joint_values.append(0.0)  # Default value for missing joints

        return joint_values

    def get_joint_values_by_arm_prefix(
        self, q: Sequence[float], arm_prefix: str
    ) -> tuple[list[float], list[str]]:
        """
        Extract joint values from q array for joints starting with arm_prefix.

        Args:
            q: Joint positions in the order of the model's joints.
            arm_prefix: Prefix of the joint names, without the trailing '/'.

        Returns:
            Tuple of the values and the names of the joints named '<arm_prefix>/...'.
        """
        joint_values = []
        joint_names = []
        all_joint_names = [name for name in self.model.names[1:]]  # Skip universe joint

        for i, joint_name in enumerate(all_joint_names):
            if joint_name.startswith(f"{arm_prefix}/"):
                joint_values.append(q[i])
                joint_names.append(joint_name)

        return joint_values, joint_names

    def get_all_joint_names(self) -> list[str]:
        """
        Get all joint names from the model (excluding universe joint).

        Returns:
            The joint names in the order of the model.
        """
        return [name for name in self.model.names[1:]]

    def _convert_joint_values_to_array(
        self, current_joint_values: JointValues
    ) -> npt.NDArray[np.floating]:
        """
        Convert joint values (dict or array) to a proper numpy array for Pinocchio.

        Args:
            current_joint_values: Joint positions, either by joint name (missing joints are set
                to 0.0) or as a sequence in the order of the model's configuration vector
                (padded with zeros if too short).

        Returns:
            The configuration vector.
        """
        if isinstance(current_joint_values, dict):
            # Create full joint configuration array from dictionary
            q = np.zeros(self.model.nq)

            # Handle multi-DOF joints properly
            joint_index = 0  # Track position in q array

            for model_joint_idx in range(1, len(self.model.joints)):  # Skip universe joint
                joint = self.model.joints[model_joint_idx]
                joint_name = self.model.names[model_joint_idx]

                if joint_name in current_joint_values:
                    if joint.nq == 1:
                        # Single DOF joint (most common)
                        q[joint_index] = current_joint_values[joint_name]
                        self.node.get_logger().debug(
                            f"Set q[{joint_index}] = {current_joint_values[joint_name]} for joint '{joint_name}'"
                        )
                    else:
                        # Multi-DOF joint (rare, but handle it)
                        self.node.get_logger().debug(
                            f"Multi-DOF joint '{joint_name}' has {joint.nq} DOFs - using first value only"
                        )
                        q[joint_index] = current_joint_values[joint_name]
                else:
                    # Missing joint - use 0.0
                    if joint.nq == 1:
                        q[joint_index] = 0.0
                        self.node.get_logger().debug(
                            f"Missing joint '{joint_name}', set q[{joint_index}] = 0.0"
                        )

                joint_index += joint.nq  # Advance by the number of DOFs this joint has

        else:
            # Convert list/array to numpy array
            q = np.array(current_joint_values, dtype=np.float64)

            # If we don't have enough joints, pad with zeros
            if len(q) < self.model.nq:
                q_full = np.zeros(self.model.nq)
                q_full[: len(q)] = q
                q = q_full

            self.node.get_logger().debug(f"Converted array input: {q}")

        self.node.get_logger().debug(f"Final q array shape: {q.shape}, values: {q}")
        return q
