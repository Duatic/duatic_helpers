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

class PyrokiIKSolver:
    """
    High-performance Inverse Kinematics helper using PyRoki and JAX.
    This class is pure math/logic and independent of ROS node infrastructure.
    """

    def __init__(self, urdf_input, target_link_name: str):
        self.target_link_name = target_link_name
        
        if isinstance(urdf_input, str) and not urdf_input.startswith('<'):
            urdf_obj = yourdfpy.URDF.load(urdf_input)
        elif isinstance(urdf_input, io.StringIO):
            urdf_obj = yourdfpy.URDF.load(urdf_input)
        else:
            urdf_obj = yourdfpy.URDF.load(io.StringIO(urdf_input))

        self.robot = pk.Robot.from_urdf(urdf_obj)
        self.joint_names = [j.name for j in urdf_obj.robot.joints if j.type != 'fixed']
        
        self._solve_jitted = self._create_jitted_solver()

    def _create_jitted_solver(self):
        link_names = self.robot.links.names
        tcp_idx = link_names.index(self.target_link_name)
        
        lower = jnp.array(self.robot.joints.lower_limits, dtype=jnp.float32)
        upper = jnp.array(self.robot.joints.upper_limits, dtype=jnp.float32)
        
        def quat_loss(q1_wxyz, q2_wxyz):
            q1 = q1_wxyz / (jnp.linalg.norm(q1_wxyz) + 1e-9)
            q2 = q2_wxyz / (jnp.linalg.norm(q2_wxyz) + 1e-9)
            d = jnp.clip(jnp.abs(jnp.dot(q1, q2)), 0.0, 1.0)
            return 1.0 - d * d

        def loss_fn(q, target_pos, target_wxyz, q_init):
            q_clipped = jnp.clip(q, lower, upper)
            transforms = self.robot.forward_kinematics(q_clipped)
            tcp_transform = transforms[tcp_idx]
            
            fk_wxyz = tcp_transform[:4]
            fk_xyz = tcp_transform[4:]
            
            pos_err = fk_xyz - target_pos
            pos_loss = jnp.sum(pos_err * pos_err)
            ori_loss = quat_loss(fk_wxyz, target_wxyz)
            
            jump_loss = jnp.sum((q_clipped - q_init)**2)
            
            return (1.0 * pos_loss) + (0.5 * ori_loss) + (0.05 * jump_loss)

        grad_fn = jax.grad(loss_fn)

        @jax.jit
        def solve_ik(target_pos, target_wxyz, q_init):
            steps = 500
            lr = 0.02
            
            m = jnp.zeros_like(q_init)
            v = jnp.zeros_like(q_init)
            b1, b2 = 0.9, 0.999
            eps = 1e-8

            def body(i, state):
                q, m, v = state
                g = grad_fn(q, target_pos, target_wxyz, q_init)
                m = b1 * m + (1.0 - b1) * g
                v = b2 * v + (1.0 - b2) * (g * g)
                t = i + 1.0
                mhat = m / (1.0 - b1**t)
                vhat = v / (1.0 - b2**t)
                q_new = q - lr * mhat / (jnp.sqrt(vhat) + eps)
                q_new = jnp.clip(q_new, lower, upper)
                return (q_new, m, v)

            q_final, _, _ = jax.lax.fori_loop(0, steps, body, (q_init, m, v))
            
            transforms = self.robot.forward_kinematics(q_final)
            tcp_transform = transforms[tcp_idx]
            pos_err_norm = jnp.linalg.norm(tcp_transform[4:] - target_pos)
            ori_err_norm = quat_loss(tcp_transform[:4], target_wxyz)
            
            return q_final, pos_err_norm, ori_err_norm
            
        return solve_ik

    def solve(self, target_pos, target_wxyz, q_init):
        """Calculates IK. Returns: (joint_array, pos_error, ori_error)"""
        q_sol, p_err, o_err = self._solve_jitted(
            jnp.array(target_pos), jnp.array(target_wxyz), jnp.array(q_init)
        )
        return np.array(q_sol), float(p_err), float(o_err)