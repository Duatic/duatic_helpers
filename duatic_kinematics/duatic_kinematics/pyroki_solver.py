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
import xml.etree.ElementTree as ET

import numpy as np
import pyroki as pk
import yourdfpy
import jax
import jax.numpy as jnp
import jax_dataclasses as jdc
import jaxlie
import jaxls

from ament_index_python.packages import get_package_share_directory
import os


def ros_package_filename_handler(fname):
    if fname.startswith("package://"):
        path = fname[len("package://") :]

        pkg_name, relative_path = path.split("/", 1)

        pkg_share = get_package_share_directory(pkg_name)

        return os.path.join(pkg_share, relative_path)

    return fname


def _load_urdf(urdf_input):
    """Load URDF from file path, StringIO, or XML string."""
    if isinstance(urdf_input, str) and not urdf_input.startswith("<"):
        return yourdfpy.URDF.load(urdf_input, filename_handler=ros_package_filename_handler)
    elif isinstance(urdf_input, io.StringIO):
        return yourdfpy.URDF.load(urdf_input, filename_handler=ros_package_filename_handler)
    else:
        return yourdfpy.URDF.load(
            io.StringIO(urdf_input), filename_handler=ros_package_filename_handler
        )


def _freeze_joints(urdf_input, patterns):
    """Return the URDF with every joint whose name matches a pattern made fixed.

    A joint the solver must never move does not belong in the optimization problem
    at all. Masking it out of the pose cost is not enough: it stays a variable, the
    limit constraint still applies to it, and the optimizer spends its effort on a
    result that is thrown away afterwards. On the DXTR that was not merely wasteful
    — the wheels turn freely, pyroki reports a continuous joint with assumed limits
    of +/- pi, and after driving they sit tens of radians outside that, which
    dominates the cost and stops the solve early.

    Freezing has to happen in the XML, before the model is built: setting `type` on
    an already-loaded yourdfpy joint does not change the actuated set. Only leaf
    joints should be named here — a frozen joint keeps whatever value the URDF
    gives it, so freezing one that carries the target link would move the target.
    Verified on the DXTR: 24 actuated joints down to 16 (hip plus both arms), with
    the clamps' forward kinematics identical to 0.0000 mm.

    Which joints those are is the caller's business, hence patterns rather than
    anything robot-specific in here.
    """
    if isinstance(urdf_input, yourdfpy.URDF):
        xml = urdf_input.write_xml_string()
    elif isinstance(urdf_input, io.StringIO):
        xml = urdf_input.getvalue()
    elif isinstance(urdf_input, str) and not urdf_input.startswith("<"):
        with open(urdf_input, encoding="utf-8") as f:
            xml = f.read()
    else:
        xml = urdf_input

    root = ET.fromstring(xml)
    frozen = []
    for joint in root.findall("joint"):
        name = joint.get("name", "")
        if joint.get("type") == "fixed" or not any(p in name for p in patterns):
            continue
        joint.set("type", "fixed")
        # A fixed joint has no axis or limits; leaving them behind only invites a
        # parser to disagree with us about what this joint is.
        for tag in ("axis", "limit", "dynamics", "mimic", "safety_controller"):
            for child in joint.findall(tag):
                joint.remove(child)
        frozen.append(name)
    return ET.tostring(root, encoding="unicode"), frozen


def _limit_margin_residual(
    vals: jaxls.VarValues,
    robot: pk.Robot,
    joint_var: jaxls.Var[jax.Array],
    joint_mask: jax.Array,
    margin: float = 0.15,
    weight: float = 5.0,
) -> jax.Array:
    """Penalizes joints approaching their limits within a margin zone.

    Unlike limit_constraint, which only prevents exceeding a limit, this is a soft
    cost that pushes joints away from their limits, keeping the solution clear of
    singular configurations such as a fully extended elbow.

    Only the joints being optimized are charged for it, which is what joint_mask
    is for. Charging the others is not a harmless extra term, it breaks the solve:
    a continuous joint such as a wheel is reported by pyroki with assumed limits of
    +/- pi, and after a few metres of driving the wheels sit tens of radians past
    that. Measured, one wheel stood 26.5 rad outside its assumed range, which at
    weight 5 is a residual of 132 — a constant, since the wheels are masked out and
    cannot move, but one that dwarfs the pose residual the solve is actually about.
    The optimizer then stops on its relative-improvement criterion while the tool
    is still 3 cm and 0.06 rad away, and does so identically in every situation,
    which is exactly the constant miss that looked like an unreachable pose.
    """
    joint_cfg = vals[joint_var]
    upper_proximity = jnp.maximum(0.0, joint_cfg - (robot.joints.upper_limits - margin))
    lower_proximity = jnp.maximum(0.0, (robot.joints.lower_limits + margin) - joint_cfg)
    masked = jnp.concatenate([upper_proximity * joint_mask, lower_proximity * joint_mask])
    return (masked * weight).flatten()


_limit_margin_cost = jaxls.Cost.factory(_limit_margin_residual)


def _soft_range_residual(
    vals: jaxls.VarValues,
    joint_var: jaxls.Var[jax.Array],
    indices: jax.Array,
    lower: jax.Array,
    upper: jax.Array,
    weights: jax.Array,
) -> jax.Array:
    """Cost for leaving a preferred range, zero inside it.

    Not a limit — the URDF limits still apply and are enforced separately. This
    expresses "possible, but pay for it", which is what a thermal, wear or
    reachability preference needs: forbidding the range outright would make
    poses unreachable that a robot must sometimes assume anyway, while a cost
    makes the solver stay out of it unless there is no alternative.

    Ranges and weights come from the caller; nothing here knows which joints of
    which robot they belong to.
    """
    cfg = vals[joint_var][indices]
    over = jnp.maximum(0.0, cfg - upper)
    under = jnp.maximum(0.0, lower - cfg)
    return ((over + under) * weights).flatten()


_soft_range_cost = jaxls.Cost.factory(_soft_range_residual)


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
    soft_indices: jax.Array,
    soft_lower: jax.Array,
    soft_upper: jax.Array,
    soft_weights: jax.Array,
    self_collision_weight: float = 10.0,
    self_collision_margin: float = 0.01,
    limit_margin: float = 0.15,
    limit_margin_weight: float = 5.0,
) -> jax.Array:
    """Solve IK using jaxls least-squares with analytic Jacobian and joint masking.

    prev_cfg is used as the rest-pose regularization target.
    initial_cfg is used as the optimization starting point (may differ from prev_cfg
    to break singularities).

    soft_* describe preferred joint ranges (see _soft_range_residual), passed as
    index arrays rather than names so this stays jittable; empty arrays disable
    them.

    There is deliberately no continuity COST here. Penalizing a joint's departure
    from its previous value with a plain (a - b) residual blows up near an angle
    wrap, where a physically tiny reorientation is a ~2*pi jump in raw joint
    space, and that destabilizes convergence. Keeping a solution on the same
    branch is handled where it belongs: by solving from several seeds and picking
    among the acceptable results (see solve()).
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
            # 100, not 10. The acceptance thresholds ask for 0.015 m of position
            # and 0.01 rad of orientation — 0.6 degrees — and a weight of 10
            # against a position weight of 50 does not buy that: measured, the
            # solve settled at 4 mm of position but 0.05 rad of orientation, so
            # it was trading away exactly the quantity that was checked most
            # tightly. This is also the value the colleagues' planner service
            # runs with, alongside the same thresholds.
            ori_weight=100.0,
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
            joint_mask,
            margin=limit_margin,
            weight=limit_margin_weight,
        ),
        _soft_range_cost(
            joint_var,
            soft_indices,
            soft_lower,
            soft_upper,
            soft_weights,
        ),
    ]

    sol = (
        jaxls.LeastSquaresProblem(costs=costs, variables=[joint_var])
        .analyze()
        .solve(
            verbose=False,
            linear_solver="dense_cholesky",
            trust_region=jaxls.TrustRegionConfig(lambda_initial=1.0),
            # 60, not 30. The direct continuation from a sane posture was coming
            # back 2.5 cm short of a target it can clearly reach, which is what a
            # solve that ran out of iterations looks like — and being rejected for
            # it is expensive, because the fallback is a different arm shape
            # entirely. Doubling the budget costs single-digit milliseconds.
            termination=jaxls.TerminationConfig(max_iterations=60),
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
        soft_ranges=None,
        continuity_joints=None,
        exclude_joints=None,
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
            soft_ranges: optional dict of joint name pattern -> (lower, upper, weight).
                Preferred ranges, not limits: leaving one costs, it is not forbidden.
                Use it for thermal, wear or reachability preferences that must not
                make a pose unreachable. Which joints and which values is the
                caller's business — a pattern matching none of this robot's joints
                contributes nothing, so no per-robot special case is needed here.
            exclude_joints: optional list of joint name patterns the solver must
                never move — wheels, a head, gripper fingers. They are frozen in
                the URDF before the model is built, so they are not variables at
                all (see _freeze_joints for why masking them is not enough). Name
                only leaf joints.
            continuity_joints: optional dict of joint name pattern -> weight, giving
                those joints extra say when solve() ranks the results of several
                seeds by closeness (see config_distance). Use it for joints that
                could otherwise reach the same pose through a flipped branch —
                typically the rotational ones. These weights never enter the
                solver: as a cost, a plain difference on a wrapping joint
                destabilizes convergence, which is why the selection does the job.
        """
        self.frozen_joints: list = []
        if exclude_joints:
            urdf_input, self.frozen_joints = _freeze_joints(urdf_input, exclude_joints)
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

        # Resolve the preference patterns to index arrays once. They are handed to
        # the jitted solve as arrays rather than names, and an empty array simply
        # contributes nothing — so a robot without these joints needs no special
        # case anywhere.
        idx, lo, hi, w = [], [], [], []
        for pattern, (lower, upper, weight) in (soft_ranges or {}).items():
            for i, name in enumerate(self.joint_names):
                if pattern in name:
                    idx.append(i)
                    lo.append(float(lower))
                    hi.append(float(upper))
                    w.append(float(weight))
        self._soft_idx = jnp.array(idx, dtype=jnp.int32)
        self._soft_lower = jnp.array(lo, dtype=jnp.float32)
        self._soft_upper = jnp.array(hi, dtype=jnp.float32)
        self._soft_weights = jnp.array(w, dtype=jnp.float32)

        # Extra weight when RANKING equally valid solutions (see solve()), not a
        # cost. These are the joints that can reach the same tool pose through a
        # visibly different arm configuration.
        self._rank_weights = np.ones(len(self.joint_names), dtype=np.float64)
        for pattern, weight in (continuity_joints or {}).items():
            for i, name in enumerate(self.joint_names):
                if pattern in name:
                    self._rank_weights[i] = float(weight)

        # A joint whose range spans a full turn wraps, so distances along it must
        # be measured the short way round: without that, a physically tiny
        # reorientation across the wrap point measures as ~2*pi and would make a
        # perfectly continuous solution look like the worst possible one.
        self._wraps = np.zeros(len(self.joint_names), dtype=bool)
        for i, name in enumerate(self.joint_names):
            j = urdf_obj.joint_map.get(name)
            lim = getattr(j, "limit", None) if j is not None else None
            lo = getattr(lim, "lower", None) if lim is not None else None
            hi = getattr(lim, "upper", None) if lim is not None else None
            if lo is None or hi is None:
                # A continuous joint (a wheel, a rotation without end stops)
                # carries no bounds at all.
                self._wraps[i] = True
            elif (hi - lo) >= 2.0 * np.pi - 1e-6:
                self._wraps[i] = True

    def config_distance(self, q, q_ref):
        """Weighted joint-space distance, used only to rank solutions that already
        meet the pose thresholds — never fed into the solver.

        Prefers a result that keeps the ranked joints near where they were over an
        equally valid but reconfigured one. Wrapping joints are compared the short
        way round.
        """
        d = np.asarray(q, dtype=np.float64) - np.asarray(q_ref, dtype=np.float64)
        d = np.where(self._wraps, np.arctan2(np.sin(d), np.cos(d)), d)
        return float(np.sum(self._rank_weights * d * d))

    def joint_step(self, q, q_ref, mask=None):
        """Largest single-joint travel from q_ref to q, the short way round.

        Only the joints the solve was allowed to move are counted: a joint pinned
        to its previous value cannot contribute travel, and one frozen out of the
        model has none to contribute either.
        """
        d = np.asarray(q, dtype=np.float64) - np.asarray(q_ref, dtype=np.float64)
        d = np.where(self._wraps, np.arctan2(np.sin(d), np.cos(d)), d)
        if mask is not None:
            d = d * (np.asarray(mask, dtype=np.float64) > 0.5)
        return float(np.max(np.abs(d))) if d.size else 0.0

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

    def solve(self, target_link_name, target_pos, target_wxyz, prev_cfg,
              joint_mask=None, rest_cfg=None, soft_scale=1.0,
              seeds=None, prefer_near=None,
              pos_err_thresh=None, ori_err_thresh=None, max_joint_step=None):
        """
        Solve IK for a single target link.

        Args:
            target_link_name: Name of the target link (e.g. "flange", "arm_left/flange")
            target_pos: (3,) target position
            target_wxyz: (4,) target quaternion (wxyz format)
            prev_cfg: (n_actuated,) previous joint config — the initial guess, and
                the rest pose unless rest_cfg is given
            joint_mask: (n_actuated,) optional, 1.0=optimize, 0.0=lock. Default: all 1.0.
            soft_scale: multiplier on the configured soft-range weights for this
                one solve. 0.0 switches them off, 1.0 applies them as configured.
                Needed because such a preference is often conditional: a thermal
                limit on holding a load says nothing about how the arm got there,
                and applying it to the whole motion can make the approach itself
                unreachable. Only the weights are scaled, never the index set, so
                switching it per call costs no re-compilation of the solve.
            rest_cfg: (n_actuated,) optional rest-pose target for the
                regularization cost, decoupled from the initial guess. Pass a
                reference posture here to steer which of several valid solutions
                is chosen — an elbow-up reference keeps the arm reaching over an
                object instead of scooping under it. Leaving it at prev_cfg pulls
                the solution towards wherever the arm happens to be, which is
                actively harmful once the arm sits in a bad spot: a config that
                is snagged on the environment then attracts the solve and the
                result is off by 10 cm or more.
            seeds: optional list of (n_actuated,) starting configurations to solve
                from, tried in addition to prev_cfg. A redundant arm reaches the
                same tool pose in several ways, and which one a gradient solve
                converges to is decided entirely by where it starts. One seed
                therefore gives no control at all over the arm's shape: if the
                current configuration sits in the wrong basin, the solve returns
                a valid pose with the elbow folded into the body. Several seeds
                turn that into a choice.
            prefer_near: (n_actuated,) reference for picking among the seeds'
                results. Every result that meets the thresholds is scored by
                config_distance against this, and the closest wins — so the arm
                keeps its shape instead of reconfiguring. Defaults to prev_cfg,
                i.e. to where the arm actually is: that is what continuity means.
                Ranking against the rest pose instead makes a solve that has to
                fall back to another seed reconfigure the whole arm.
            max_joint_step: how far any single joint may travel from prev_cfg for a
                result to count as usable, in radians. Without it, accuracy is the
                only criterion and it picks postures no one would choose: measured,
                the direct continuation missed a grasp by 2.5 cm and was rejected,
                so a solution 1.3 mm from the target won instead — 3.09 rad away,
                with the shoulder and the wrist both sitting on their limits. A near
                miss in a sane posture beats a perfect hit in a contorted one, and
                it beats it on the real robot too, where such a posture is where
                joints overheat and singularities live. Measured the short way
                round, so a wrapping joint is not condemned for crossing +/- pi.
            pos_err_thresh / ori_err_thresh: what counts as reaching the target
                for that selection. A result missing either is only used if no
                seed produced an acceptable one, in which case the best attempt
                is returned with its true errors — reporting the failure is the
                caller's job, not something to paper over here. Default: accept
                any result, i.e. rank purely by closeness.

        Returns:
            (cfg, pos_error, ori_error)
        """
        n = len(self.joint_names)
        if joint_mask is None:
            joint_mask = jnp.ones(n, dtype=jnp.float32)

        target_link_index = self.robot.links.names.index(target_link_name)

        prev_cfg_np = jnp.asarray(prev_cfg, dtype=jnp.float32)
        rest_cfg_np = (prev_cfg_np if rest_cfg is None
                       else jnp.asarray(rest_cfg, dtype=jnp.float32))

        # prev_cfg first: continuing straight from where the arm is, is the best
        # possible outcome, so an acceptable result from that seed is taken and
        # the remaining seeds are never solved. That also keeps a call that
        # already works today bit-for-bit unchanged — the extra seeds only ever
        # come into play once the direct continuation misses.
        seed_list, seen = [], set()
        for cand in [prev_cfg_np] + list(seeds or []):
            arr = np.asarray(cand, dtype=np.float64)
            key = tuple(np.round(arr, 4).tolist())
            if key not in seen:
                seen.add(key)
                seed_list.append(jnp.asarray(cand, dtype=jnp.float32))

        # Ranked against where the arm IS, not against the reference posture.
        # Getting this wrong is expensive and was: with rest_cfg as the reference,
        # a solve whose direct continuation missed fell through to the solution
        # nearest the reference posture, and the arm was commanded to swing its
        # shoulder 2.54 rad — 146 degrees — round to a different but equally valid
        # arm shape. The controller aborted on its trajectory tolerance, and
        # rightly so. Continuity is a statement about the previous configuration;
        # the reference posture's job is to steer the SOLVE (as the rest cost), not
        # to decide which of several valid results to keep.
        reference = np.asarray(
            prefer_near if prefer_near is not None else prev_cfg, dtype=np.float64
        )

        best = None          # (distance, cfg, pos_err, ori_err) among acceptable
        fallback = None      # (pos_err + ori_err, cfg, pos_err, ori_err) otherwise
        self.last_solve_info = {"seeds": len(seed_list), "chosen": None, "attempts": []}

        for seed_i, seed in enumerate(seed_list):
            initial_cfg_np = self._nudge_near_zero(seed)

            cfg = _solve_ik(
                self.robot,
                self.robot_coll,
                jnp.array(target_link_index, dtype=jnp.int32),
                jnp.array(target_wxyz, dtype=jnp.float32),
                jnp.array(target_pos, dtype=jnp.float32),
                jnp.array(joint_mask, dtype=jnp.float32),
                jnp.array(rest_cfg_np, dtype=jnp.float32),
                jnp.array(initial_cfg_np, dtype=jnp.float32),
                self._soft_idx,
                self._soft_lower,
                self._soft_upper,
                self._soft_weights * float(soft_scale),
                self_collision_weight=self.self_collision_weight,
                self_collision_margin=self.self_collision_margin,
            )

            # Safety net: enforce locked joints stay exactly at prev_cfg
            cfg_np = jnp.where(jnp.array(joint_mask) > 0.5, jnp.array(cfg), prev_cfg)

            pos_err, ori_err = _compute_pose_error(
                self.robot,
                jnp.array(cfg_np, dtype=jnp.float32),
                jnp.array(target_link_index, dtype=jnp.int32),
                jnp.array(target_wxyz, dtype=jnp.float32),
                jnp.array(target_pos, dtype=jnp.float32),
            )
            cfg_out = np.array(cfg_np)
            pos_err, ori_err = float(pos_err), float(ori_err)

            step = self.joint_step(cfg_out, np.asarray(prev_cfg, dtype=np.float64),
                                   mask=np.asarray(joint_mask, dtype=np.float64))
            ok = ((pos_err_thresh is None or pos_err <= pos_err_thresh)
                  and (ori_err_thresh is None or ori_err <= ori_err_thresh)
                  and (max_joint_step is None or step <= max_joint_step))
            self.last_solve_info["attempts"].append(
                (seed_i, round(pos_err, 4), round(ori_err, 4), round(step, 3), ok)
            )

            if ok and seed_i == 0:
                self.last_solve_info["chosen"] = 0
                return cfg_out, pos_err, ori_err
            if ok:
                d = self.config_distance(cfg_out, reference)
                if best is None or d < best[0]:
                    best = (d, cfg_out, pos_err, ori_err, seed_i)
            else:
                score = pos_err + ori_err
                if fallback is None or score < fallback[0]:
                    fallback = (score, cfg_out, pos_err, ori_err, seed_i)

        chosen = best if best is not None else fallback
        self.last_solve_info["chosen"] = chosen[4]
        return chosen[1], chosen[2], chosen[3]

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
