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


import numpy as np
import pytest

ps = pytest.importorskip("duatic_kinematics.pyroki_solver")

# One arm whose tip folds down into its base at "fold" = pi/2.
_URDF = """<robot name="test">
  <link name="base"><collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
  <link name="upper"><collision><origin xyz="0 0 0.1"/><geometry><box size="0.04 0.04 0.1"/></geometry></collision></link>
  <link name="tip"><collision><origin xyz="0.3 0 0"/><geometry><box size="0.1 0.04 0.04"/></geometry></collision></link>
  <joint name="turn" type="revolute"><parent link="base"/><child link="upper"/>
    <origin xyz="0.5 0 0.1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="fold" type="revolute"><parent link="upper"/><child link="tip"/>
    <origin xyz="-0.5 0 0.2"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
</robot>"""


def test_solve_multi_without_the_collision_cost_never_builds_it(monkeypatch):
    def no_collision_cost(*args, **kwargs):
        raise AssertionError("self-collision cost built")

    monkeypatch.setattr(ps.pk.costs, "self_collision_cost", no_collision_cost)
    solver = ps.PyrokiIKSolver(_URDF)
    names = solver.joint_names
    goal = np.array([1.5708 if n == "fold" else 0.0 for n in names], dtype=np.float32)
    pos, wxyz = solver.forward_kinematics(goal, ["tip"])
    start = np.zeros(len(names), dtype=np.float32)
    _, errors = solver.solve_multi(["tip"], pos, wxyz, start, avoid_self_collision=False)
    assert errors[0][0] < 1e-3
