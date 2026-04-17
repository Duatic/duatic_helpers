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

import io
import numpy as np
import pyroki as pk
import yourdfpy
import jax
import jax.numpy as jnp
import jax_dataclasses as jdc
import jaxlie
import jaxls


def _load_urdf(urdf_input):
    """Load URDF from file path, StringIO, or XML string."""
    if isinstance(urdf_input, str) and not urdf_input.startswith("<"):
        return yourdfpy.URDF.load(urdf_input)
    elif isinstance(urdf_input, io.StringIO):
        return yourdfpy.URDF.load(urdf_input)
    else:
        return yourdfpy.URDF.load(io.StringIO(urdf_input))


def _limit_margin_residual(
    vals: jaxls.VarValues,
    robot: pk.Robot,
    joint_var: jaxls.Var[jax.Array],
    margin: float = 0.15,
    weight: float = 5.0,
) -> jax.Array:
    """Penalizes joints approaching their limits within a margin zone.

    Unlike limit_constraint which only prevents exceeding limits,
    this creates a soft cost that pushes joints away from limits,
    preventing singularities (e.g. fully extended elbow).
    """
    joint_cfg = vals[joint_var]
    joint_cfg_eff = robot.joints.get_full_config(joint_cfg)
    upper_proximity = jnp.maximum(0.0, joint_cfg_eff - (robot.joints.upper_limits_all - margin))
    lower_proximity = jnp.maximum(0.0, (robot.joints.lower_limits_all + margin) - joint_cfg_eff)
    return (jnp.concatenate([upper_proximity, lower_proximity]) * weight).flatten()


_limit_margin_cost = jaxls.Cost.factory(_limit_margin_residual)


@jdc.jit
def _solve_ik(
    robot: pk.Robot,
    robot_coll: pk.collision.RobotCollision,
    target_link_index: jax.Array,
    target_wxyz: jax.Array,
    target_position: jax.Array,
    joint_mask: jax.Array,
    prev_cfg: jax.Array,
    initial_cfg: jax.Array,
    self_collision_weight: float = 10.0,
    self_collision_margin: float = 0.01,
    limit_margin: float = 0.15,
    limit_margin_weight: float = 5.0,
) -> jax.Array:
    """Solve IK using jaxls least-squares with analytic Jacobian and joint masking.

    prev_cfg is used as the rest-pose regularization target.
    initial_cfg is used as the optimization starting point (may differ from prev_cfg
    to break singularities).
    """
    joint_var = robot.joint_var_cls(0)

    target_pose = jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3(target_wxyz), target_position)

    costs = [
        pk.costs.pose_cost_analytic_jac(
            robot,
            joint_var,
            target_pose,
            target_link_index,
            pos_weight=50.0,
            ori_weight=10.0,
            joint_mask=joint_mask,
        ),
        pk.costs.rest_cost(
            joint_var,
            rest_pose=prev_cfg,
            weight=1.0,
        ),
        pk.costs.limit_constraint(
            robot,
            joint_var,
        ),
        # pk.costs.self_collision_cost(
        #     robot,
        #     robot_coll,
        #     joint_var,
        #     margin=self_collision_margin,
        #     weight=self_collision_weight,
        # ),
        _limit_margin_cost(
            robot,
            joint_var,
            margin=limit_margin,
            weight=limit_margin_weight,
        ),
    ]

    sol = (
        jaxls.LeastSquaresProblem(costs=costs, variables=[joint_var])
        .analyze()
        .solve(
            verbose=False,
            linear_solver="dense_cholesky",
            trust_region=jaxls.TrustRegionConfig(lambda_initial=1.0),
            initial_vals=jaxls.VarValues.make([joint_var.with_value(initial_cfg)]),
        )
    )
    return sol[joint_var]


@jdc.jit
def _compute_pose_error(
    robot: pk.Robot,
    joint_cfg: jax.Array,
    target_link_index: jax.Array,
    target_wxyz: jax.Array,
    target_position: jax.Array,
) -> tuple[jax.Array, jax.Array]:
    """Compute position and orientation error between FK result and target."""
    Ts_world_link = robot.forward_kinematics(joint_cfg)
    actual_pose = jaxlie.SE3(Ts_world_link[target_link_index])
    target_pose = jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3(target_wxyz), target_position)
    pose_error = (actual_pose.inverse() @ target_pose).log()
    pos_err = jnp.linalg.norm(pose_error[:3])
    ori_err = jnp.linalg.norm(pose_error[3:])
    return pos_err, ori_err


@jdc.jit
def _solve_ik_multi(
    robot: pk.Robot,
    robot_coll: pk.collision.RobotCollision,
    target_link_indices: jax.Array,
    target_wxyzs: jax.Array,
    target_positions: jax.Array,
    joint_mask: jax.Array,
    prev_cfg: jax.Array,
    initial_cfg: jax.Array,
    self_collision_weight: float = 10.0,
    self_collision_margin: float = 0.01,
    limit_margin: float = 0.15,
    limit_margin_weight: float = 5.0,
) -> jax.Array:
    """Solve whole-body IK for multiple targets simultaneously (bimanual pattern).

    prev_cfg is used as the rest-pose regularization target.
    initial_cfg is used as the optimization starting point.
    """
    JointVar = robot.joint_var_cls

    target_pose = jaxlie.SE3.from_rotation_and_translation(
        jaxlie.SO3(target_wxyzs), target_positions
    )
    batch_axes = target_pose.get_batch_axes()

    # Broadcast joint_mask to match batch axes (N, n_joints)
    batched_mask = jnp.broadcast_to(joint_mask[None], batch_axes + joint_mask.shape)

    costs = [
        pk.costs.pose_cost_analytic_jac(
            jax.tree.map(lambda x: x[None], robot),
            JointVar(jnp.full(batch_axes, 0)),
            target_pose,
            target_link_indices,
            pos_weight=50.0,
            ori_weight=10.0,
            joint_mask=batched_mask,
        ),
        pk.costs.rest_cost(
            JointVar(0),
            rest_pose=prev_cfg,
            weight=1.0,
        ),
        pk.costs.limit_constraint(
            robot,
            JointVar(0),
        ),
        pk.costs.self_collision_cost(
            robot,
            robot_coll,
            JointVar(0),
            margin=self_collision_margin,
            weight=self_collision_weight,
        ),
        _limit_margin_cost(
            robot,
            JointVar(0),
            margin=limit_margin,
            weight=limit_margin_weight,
        ),
    ]

    sol = (
        jaxls.LeastSquaresProblem(costs=costs, variables=[JointVar(0)])
        .analyze()
        .solve(
            verbose=False,
            linear_solver="dense_cholesky",
            trust_region=jaxls.TrustRegionConfig(lambda_initial=10.0),
            initial_vals=jaxls.VarValues.make([JointVar(0).with_value(initial_cfg)]),
        )
    )
    return sol[JointVar(0)]


class PyrokiIKSolver:
    """
    IK solver using PyRoKi's native jaxls least-squares optimization.

    Features over the old Adam-based solver:
    - Analytic Jacobian (faster, more accurate)
    - Trust-region optimization (better convergence than Adam)
    - Joint masking for decoupled multi-arm control
    - Joint limits as proper constraints (not just clipping)
    - Rest-pose regularization (smooth, stable solutions)
    - Automatic singularity nudge for near-zero configurations
    """

    def __init__(
        self,
        urdf_input,
        self_collision_weight=10.0,
        self_collision_margin=0.01,
        singularity_nudge_joints=None,
    ):
        """
        Args:
            urdf_input: URDF file path, XML string, StringIO, or yourdfpy.URDF object
            self_collision_weight: weight for self-collision avoidance cost
            self_collision_margin: collision margin in meters
            singularity_nudge_joints: optional dict mapping joint name patterns to nudge
                values (rad). When the robot config is near-zero (singular), matching
                joints are nudged to help the optimizer escape.
                Example: {"elbow_flexion": -0.3, "shoulder_flexion": 0.2}
        """
        if isinstance(urdf_input, yourdfpy.URDF):
            urdf_obj = urdf_input
        else:
            urdf_obj = _load_urdf(urdf_input)
        self.robot = pk.Robot.from_urdf(urdf_obj)
        self.robot_coll = pk.collision.RobotCollision.from_urdf(urdf_obj)
        self.joint_names = list(self.robot.joints.actuated_names)
        self.self_collision_weight = self_collision_weight
        self.self_collision_margin = self_collision_margin

        # Pre-identify joints for singularity nudging (pattern -> (indices, nudge_value))
        self._nudge_config = {}
        if singularity_nudge_joints:
            for pattern, nudge_val in singularity_nudge_joints.items():
                indices = [i for i, n in enumerate(self.joint_names) if pattern in n]
                if indices:
                    self._nudge_config[pattern] = (indices, nudge_val)

    def _nudge_near_zero(self, cfg: jnp.ndarray, thresh: float = 0.08) -> jnp.ndarray:
        """If cfg is near-zero (singular), nudge configured joints to help the optimizer.

        Returns a modified copy suitable as initial_cfg. The original prev_cfg
        should still be used as the rest-pose so the solver doesn't over-commit.
        """
        if not self._nudge_config:
            return cfg  # no nudge joints configured
        if jnp.sum(jnp.abs(cfg) > thresh) > 2:
            return cfg  # not near-zero, no nudge needed

        nudged = cfg.copy()
        for indices, nudge_val in self._nudge_config.values():
            for i in indices:
                nudged[i] = nudge_val
        return nudged

    def forward_kinematics(self, joint_cfg, link_names):
        """Compute forward kinematics for given links.

        Args:
            joint_cfg: (n_actuated,) joint configuration
            link_names: list of link names

        Returns:
            (positions, wxyzs) — (N, 3) positions and (N, 4) quaternions (wxyz)
        """
        Ts = self.robot.forward_kinematics(jnp.array(joint_cfg, dtype=jnp.float32))
        positions = []
        wxyzs = []
        for name in link_names:
            idx = self.robot.links.names.index(name)
            pose = jaxlie.SE3(Ts[idx])
            positions.append(jnp.array(pose.translation()))
            wxyzs.append(jnp.array(pose.rotation().wxyz))
        return np.array(positions, dtype=np.float32), np.array(wxyzs, dtype=np.float32)

    def solve(self, target_link_name, target_pos, target_wxyz, prev_cfg, joint_mask=None):
        """
        Solve IK for a single target link.

        Args:
            target_link_name: Name of the target link (e.g. "flange", "arm_left/flange")
            target_pos: (3,) target position
            target_wxyz: (4,) target quaternion (wxyz format)
            prev_cfg: (n_actuated,) previous joint config — used as initial guess and rest pose
            joint_mask: (n_actuated,) optional, 1.0=optimize, 0.0=lock. Default: all 1.0.

        Returns:
            (cfg, pos_error, ori_error)
        """
        n = len(self.joint_names)
        if joint_mask is None:
            joint_mask = jnp.ones(n, dtype=jnp.float32)

        target_link_index = self.robot.links.names.index(target_link_name)

        prev_cfg_np = jnp.asarray(prev_cfg, dtype=jnp.float32)
        initial_cfg_np = self._nudge_near_zero(prev_cfg_np)

        cfg = _solve_ik(
            self.robot,
            self.robot_coll,
            jnp.array(target_link_index, dtype=jnp.int32),
            jnp.array(target_wxyz, dtype=jnp.float32),
            jnp.array(target_pos, dtype=jnp.float32),
            jnp.array(joint_mask, dtype=jnp.float32),
            jnp.array(prev_cfg_np, dtype=jnp.float32),
            jnp.array(initial_cfg_np, dtype=jnp.float32),
            self_collision_weight=self.self_collision_weight,
            self_collision_margin=self.self_collision_margin,
        )

        # Safety net: enforce locked joints stay exactly at prev_cfg
        cfg_np = jnp.array(cfg)
        mask_arr = jnp.array(joint_mask)
        cfg_np = jnp.where(mask_arr > 0.5, cfg_np, prev_cfg)

        pos_err, ori_err = _compute_pose_error(
            self.robot,
            jnp.array(cfg_np, dtype=jnp.float32),
            jnp.array(target_link_index, dtype=jnp.int32),
            jnp.array(target_wxyz, dtype=jnp.float32),
            jnp.array(target_pos, dtype=jnp.float32),
        )

        return np.array(cfg_np), float(pos_err), float(ori_err)

    def solve_multi(
        self, target_link_names, target_positions, target_wxyzs, prev_cfg, joint_mask=None
    ):
        """
        Solve IK for multiple targets simultaneously (whole-body).

        All joints are optimized together to reach all targets at once.
        Based on PyRoKi's bimanual IK pattern.

        Args:
            target_link_names: list of target link names
            target_positions: (N, 3) array of target positions
            target_wxyzs: (N, 4) array of target quaternions (wxyz)
            prev_cfg: (n_actuated,) previous joint config — initial guess and rest pose
            joint_mask: (n_actuated,) optional, 1.0=optimize, 0.0=lock. Default: all 1.0.

        Returns:
            (cfg, [(pos_err, ori_err), ...]) — solution and per-target errors
        """
        n = len(self.joint_names)
        if joint_mask is None:
            joint_mask = jnp.ones(n, dtype=jnp.float32)

        target_link_indices = [self.robot.links.names.index(name) for name in target_link_names]

        prev_cfg_np = jnp.asarray(prev_cfg, dtype=jnp.float32)
        initial_cfg_np = self._nudge_near_zero(prev_cfg_np)

        cfg = _solve_ik_multi(
            self.robot,
            self.robot_coll,
            jnp.array(target_link_indices, dtype=jnp.int32),
            jnp.array(target_wxyzs, dtype=jnp.float32),
            jnp.array(target_positions, dtype=jnp.float32),
            jnp.array(joint_mask, dtype=jnp.float32),
            jnp.array(prev_cfg_np, dtype=jnp.float32),
            jnp.array(initial_cfg_np, dtype=jnp.float32),
            self_collision_weight=self.self_collision_weight,
            self_collision_margin=self.self_collision_margin,
        )
        cfg_np = jnp.array(cfg)

        # Safety net: enforce locked joints stay exactly at prev_cfg
        mask_arr = jnp.array(joint_mask)
        cfg_np = jnp.where(mask_arr > 0.5, cfg_np, prev_cfg)

        errors = []
        for i, idx in enumerate(target_link_indices):
            pos_err, ori_err = _compute_pose_error(
                self.robot,
                jnp.array(cfg_np, dtype=jnp.float32),
                jnp.array(idx, dtype=jnp.int32),
                jnp.array(target_wxyzs[i], dtype=jnp.float32),
                jnp.array(target_positions[i], dtype=jnp.float32),
            )
            errors.append((float(pos_err), float(ori_err)))

        return np.array(cfg_np), errors
