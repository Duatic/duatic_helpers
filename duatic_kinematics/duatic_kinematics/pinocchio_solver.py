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

import logging
import tempfile

import numpy as np
import pinocchio as pin

logger = logging.getLogger(__name__)


class PinocchioSolver:
    """ROS-agnostic FK/IK solver using the Pinocchio library.

    Takes a URDF as input (file path or XML string) and provides
    forward/inverse kinematics computation.

    Args:
        urdf_input: URDF file path or XML string
        joint_margins: safety margin from joint limits in radians (default: 0.2)
    """

    def __init__(self, urdf_input, joint_margins=0.2):
        self.model = self._load_model(urdf_input)
        self.data = self.model.createData()

        self.lower = self.model.lowerPositionLimit + joint_margins
        self.upper = self.model.upperPositionLimit - joint_margins

        self.joint_names = [name for name in self.model.names[1:]]  # Skip universe joint

        logger.info(
            f"Pinocchio model loaded with {self.model.nq} DOF, "
            f"{len(self.model.joints)} joints and {len(self.model.frames)} frames."
        )

    def _load_model(self, urdf_input):
        """Load a Pinocchio model from a file path or XML string."""
        if isinstance(urdf_input, str) and not urdf_input.strip().startswith("<"):
            # File path
            return pin.buildModelFromUrdf(urdf_input)
        else:
            # XML string — write to temp file since Pinocchio needs a file path
            with tempfile.NamedTemporaryFile(delete=False, suffix=".urdf") as urdf_file:
                urdf_file.write(urdf_input.encode())
                return pin.buildModelFromUrdf(urdf_file.name)

    def get_fk(self, joint_values, frame, target_frame=None):
        """Compute forward kinematics for the specified frame.

        Args:
            joint_values: joint configuration as dict {name: value}, list, or numpy array
            frame: name of the frame to compute FK for
            target_frame: optional reference frame to express the result in.
                If None, result is in the model root frame.

        Returns:
            pinocchio.SE3 pose of the frame
        """
        q = self._convert_joint_values_to_array(joint_values)

        pin.forwardKinematics(self.model, self.data, q[: self.model.nq])
        pin.updateFramePlacements(self.model, self.data)

        frame_id = self.model.getFrameId(frame)
        pose = self.data.oMf[frame_id]

        if target_frame is not None:
            try:
                target_frame_id = self.model.getFrameId(target_frame)
                target_pose = self.data.oMf[target_frame_id]
                return target_pose.inverse() * pose
            except Exception as e:
                logger.warning(f"Could not transform to frame '{target_frame}': {e}")

        return pose

    def get_fk_translation_and_quaternion(self, joint_values, frame, target_frame=None):
        """Compute FK and return as (translation, quaternion_wxyz).

        Args:
            joint_values: joint configuration
            frame: name of the frame
            target_frame: optional reference frame

        Returns:
            (translation, quaternion) where translation is (3,) ndarray
            and quaternion is (4,) ndarray in [w, x, y, z] format
        """
        pose = self.get_fk(joint_values, frame, target_frame)

        translation = np.array(pose.translation)
        quat = pin.Quaternion(pose.rotation)
        quaternion_wxyz = np.array([quat.w, quat.x, quat.y, quat.z])

        return translation, quaternion_wxyz

    def solve_ik(
        self, joint_values, target_se3, frame, target_frame=None, iterations=100, threshold=0.01
    ):
        """Solve inverse kinematics using damped pseudo-inverse.

        Args:
            joint_values: initial joint configuration
            target_se3: target pose as pinocchio.SE3
            frame: name of the end-effector frame
            target_frame: optional reference frame for the target
            iterations: maximum number of iterations
            threshold: convergence threshold for the pose error norm

        Returns:
            (q, error) where q is the solved joint configuration as numpy array
            and error is the final pose error vector
        """
        q = self._convert_joint_values_to_array(joint_values)
        error = np.inf

        frame_id = self.model.getFrameId(frame)

        # Get actual DOF count from Jacobian
        pin.forwardKinematics(self.model, self.data, q[: self.model.nq])
        pin.updateFramePlacements(self.model, self.data)
        J_test = pin.computeFrameJacobian(
            self.model, self.data, q[: self.model.nq], frame_id, pin.LOCAL
        )
        actual_dof = J_test.shape[1]

        W = np.diag(np.ones(actual_dof))

        for i in range(iterations):
            pin.forwardKinematics(self.model, self.data, q[: self.model.nq])
            pin.updateFramePlacements(self.model, self.data)

            error = self.get_pose_error(q, target_se3, frame, target_frame)
            if np.linalg.norm(error) < threshold:
                break

            J = pin.computeFrameJacobian(
                self.model, self.data, q[: self.model.nq], frame_id, pin.LOCAL
            )

            J_w = J @ W
            dq = -0.1 * W @ np.linalg.pinv(J_w) @ error

            q[:actual_dof] += dq

            if len(self.lower) >= actual_dof and len(self.upper) >= actual_dof:
                q[:actual_dof] = np.clip(
                    q[:actual_dof], self.lower[:actual_dof], self.upper[:actual_dof]
                )

        return q, error

    def get_pose_error(self, joint_values, target_se3, frame, target_frame=None):
        """Compute pose error between current FK and target pose.

        Args:
            joint_values: current joint configuration
            target_se3: target pose as pinocchio.SE3
            frame: end-effector frame name
            target_frame: optional reference frame

        Returns:
            (6,) error vector (translation + scaled rotation)
        """
        current_se3 = self.get_fk(joint_values, frame, target_frame)
        error = pin.log(target_se3.inverse() * current_se3).vector
        # Scale rotation part to balance translation/rotation error
        rot_scale = 0.2
        error[3:] *= rot_scale
        return error

    @staticmethod
    def pose_to_se3(position, quaternion_wxyz):
        """Convert position + quaternion to pinocchio.SE3.

        Args:
            position: (3,) translation
            quaternion_wxyz: (4,) quaternion in [w, x, y, z] format

        Returns:
            pinocchio.SE3 object
        """
        translation = np.asarray(position, dtype=np.float64)
        quat = np.asarray(quaternion_wxyz, dtype=np.float64)
        return pin.SE3(pin.Quaternion(*quat).matrix(), translation)

    def get_joint_values_by_names(self, q, joint_names):
        """Extract specific joint values from q array by joint names.

        Args:
            q: full joint configuration array
            joint_names: list of joint names to extract

        Returns:
            list of joint values in the same order as joint_names
        """
        joint_values = []
        for joint_name in joint_names:
            try:
                joint_index = self.joint_names.index(joint_name)
                joint_values.append(q[joint_index])
            except ValueError:
                logger.warning(f"Joint '{joint_name}' not found in model")
                joint_values.append(0.0)
        return joint_values

    def get_joint_values_by_prefix(self, q, prefix):
        """Extract joint values from q array for joints starting with prefix.

        Args:
            q: full joint configuration array
            prefix: joint name prefix (e.g. "arm_left")

        Returns:
            (joint_values, joint_names) tuple
        """
        joint_values = []
        joint_names = []
        for i, joint_name in enumerate(self.joint_names):
            if joint_name.startswith(f"{prefix}/"):
                joint_values.append(q[i])
                joint_names.append(joint_name)
        return joint_values, joint_names

    def get_all_joint_names(self):
        """Get all joint names from the model (excluding universe joint)."""
        return list(self.joint_names)

    def _convert_joint_values_to_array(self, joint_values):
        """Convert joint values (dict or array) to a numpy array for Pinocchio."""
        if isinstance(joint_values, dict):
            q = np.zeros(self.model.nq)
            joint_index = 0

            for model_joint_idx in range(1, len(self.model.joints)):
                joint = self.model.joints[model_joint_idx]
                joint_name = self.model.names[model_joint_idx]

                if joint_name in joint_values:
                    q[joint_index] = joint_values[joint_name]
                else:
                    q[joint_index] = 0.0

                joint_index += joint.nq

        else:
            q = np.array(joint_values, dtype=np.float64)
            if len(q) < self.model.nq:
                q_full = np.zeros(self.model.nq)
                q_full[: len(q)] = q
                q = q_full

        return q
