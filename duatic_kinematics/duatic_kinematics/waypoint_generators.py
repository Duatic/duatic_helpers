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

"""Cartesian waypoint generators for robot motion planning.

Each generator yields (positions, wxyzs, info) tuples representing
intermediate Cartesian targets along the trajectory path. The caller
is responsible for IK solving, smoothing, and publishing.

Supported generators:
    - linear_waypoints: Straight-line interpolation in Cartesian space
    - arc_waypoints: Circular arc interpolation through a via-point
"""

import numpy as np


def linear_waypoints(
    start_positions,
    target_positions,
    fixed_wxyzs,
    control_rate=25.0,
    linear_velocity=0.1,
    max_step_size=0.002,
):
    """Generate waypoints along a straight line in Cartesian space.

    Orientations remain fixed throughout the motion.
    Both arms move synchronously with the same interpolation parameter.

    The number of waypoints is determined by whichever produces more steps:
    the velocity-based duration at the given control_rate, or the max_step_size
    constraint. This ensures the path is followed precisely even at higher
    velocities or lower control rates.

    Args:
        start_positions: (N, 3) start EE positions
        target_positions: (N, 3) target EE positions
        fixed_wxyzs: (N, 4) orientations to maintain (wxyz format)
        control_rate: waypoint generation rate in Hz
        linear_velocity: Cartesian velocity in m/s
        max_step_size: maximum Cartesian distance between consecutive waypoints
            in meters. Smaller values produce finer resolution. Default 2mm.

    Yields:
        (positions, wxyzs, info) — info contains step, n_steps, t
    """
    max_dist = float(np.max(np.linalg.norm(target_positions - start_positions, axis=1)))
    duration = max(max_dist / linear_velocity, 0.5) if linear_velocity > 0 else 2.0
    rate_sec = 1.0 / control_rate

    # Steps from velocity + control rate
    n_steps_rate = max(int(duration / rate_sec), 1)

    # Steps from max step size constraint
    n_steps_precision = max(int(np.ceil(max_dist / max_step_size)), 1) if max_step_size > 0 else 1

    n_steps = max(n_steps_rate, n_steps_precision)

    for step in range(n_steps):
        t = min(1.0, (step + 1) / n_steps)
        positions = start_positions + t * (target_positions - start_positions)
        yield positions, fixed_wxyzs, {"step": step, "n_steps": n_steps, "t": t}


def _circumscribed_circle_3d(p0, p1, p2):
    """Compute circumscribed circle for 3 points in 3D.

    Returns (center, radius, u_axis, v_axis) where u_axis points from
    center toward p0 and v_axis is perpendicular in the circle plane.
    Returns (None, None, None, None) if points are collinear.
    """
    a = p1 - p0
    b = p2 - p0

    # Check collinearity
    normal = np.cross(a, b)
    normal_len = np.linalg.norm(normal)
    if normal_len < 1e-10:
        return None, None, None, None

    # Circumcenter via barycentric coordinates in the triangle plane
    # C = P0 + alpha * a + beta * b
    aa = np.dot(a, a)
    bb = np.dot(b, b)
    ab = np.dot(a, b)
    denom = 2.0 * (aa * bb - ab * ab)

    alpha = bb * (aa - ab) / denom
    beta = aa * (bb - ab) / denom
    center = p0 + alpha * a + beta * b

    radius = float(np.linalg.norm(center - p0))
    if radius < 1e-10:
        return None, None, None, None

    # Local coordinate frame: u points to p0, v perpendicular in plane
    normal = normal / normal_len
    u_axis = (p0 - center) / radius
    v_axis = np.cross(normal, u_axis)

    return center, radius, u_axis, v_axis


def _compute_arc_sweep(center, p_via, p_target, u_axis, v_axis):
    """Compute the sweep angle from start (angle=0) through via to target.

    The via point determines which direction to traverse the arc.
    """
    # Angles relative to center (start is at angle 0 by construction)
    to_via = p_via - center
    via_angle = np.arctan2(np.dot(to_via, v_axis), np.dot(to_via, u_axis))

    to_target = p_target - center
    target_angle = np.arctan2(np.dot(to_target, v_axis), np.dot(to_target, u_axis))

    # Normalize to [0, 2π)
    via_norm = via_angle % (2 * np.pi)
    target_norm = target_angle % (2 * np.pi)

    # Determine direction: via must be between start (0) and target
    # Try counterclockwise (positive sweep)
    if via_norm <= target_norm or target_norm < 1e-6:
        sweep = target_norm
    else:
        # Go clockwise (negative sweep)
        sweep = target_norm - 2 * np.pi

    # Verify via point is on the chosen arc
    if sweep > 0:
        if not (0 <= via_norm <= sweep + 1e-6):
            sweep = target_norm - 2 * np.pi
    else:
        if not (sweep - 1e-6 <= via_norm - 2 * np.pi <= 0):
            sweep = target_norm

    return float(sweep)


def arc_waypoints(
    start_positions,
    via_positions,
    target_positions,
    fixed_wxyzs,
    control_rate=25.0,
    arc_velocity=0.1,
    max_step_size=0.002,
):
    """Generate waypoints along a circular arc in Cartesian space.

    The arc is defined by three points per arm: start (from FK),
    via (intermediate), and target. Orientations remain fixed.
    Both arms move synchronously with the same interpolation parameter.

    Falls back to linear interpolation if points are collinear.

    Args:
        start_positions: (N, 3) start EE positions (from FK)
        via_positions: (N, 3) intermediate points on the arc
        target_positions: (N, 3) target EE positions
        fixed_wxyzs: (N, 4) orientations to maintain (wxyz format)
        control_rate: waypoint generation rate in Hz
        arc_velocity: Cartesian velocity along the arc in m/s
        max_step_size: maximum Cartesian distance between consecutive waypoints
            in meters. Default 2mm.

    Yields:
        (positions, wxyzs, info) — info contains step, n_steps, t
    """
    n_targets = start_positions.shape[0]
    rate_sec = 1.0 / control_rate

    # Pre-compute arc parameters for each arm
    arcs = []
    max_arc_length = 0.0

    for i in range(n_targets):
        center, radius, u_axis, v_axis = _circumscribed_circle_3d(
            start_positions[i], via_positions[i], target_positions[i]
        )

        if center is None:
            # Collinear — linear fallback
            dist = float(np.linalg.norm(target_positions[i] - start_positions[i]))
            max_arc_length = max(max_arc_length, dist)
            arcs.append(None)
        else:
            sweep = _compute_arc_sweep(
                center, via_positions[i], target_positions[i], u_axis, v_axis
            )
            arc_length = abs(sweep) * radius
            max_arc_length = max(max_arc_length, arc_length)

            arcs.append({
                "center": center,
                "radius": radius,
                "u_axis": u_axis,
                "v_axis": v_axis,
                "sweep": sweep,
            })

    # Compute duration and step count from longest arc
    duration = max(max_arc_length / arc_velocity, 0.5) if arc_velocity > 0 else 2.0
    n_steps_rate = max(int(duration / rate_sec), 1)
    n_steps_precision = max(int(np.ceil(max_arc_length / max_step_size)), 1) if max_step_size > 0 else 1
    n_steps = max(n_steps_rate, n_steps_precision)

    for step in range(n_steps):
        t = min(1.0, (step + 1) / n_steps)
        positions = np.zeros_like(start_positions)

        for i in range(n_targets):
            if arcs[i] is None:
                # Linear fallback
                positions[i] = start_positions[i] + t * (
                    target_positions[i] - start_positions[i]
                )
            else:
                arc = arcs[i]
                angle = t * arc["sweep"]
                positions[i] = (
                    arc["center"]
                    + arc["radius"] * np.cos(angle) * arc["u_axis"]
                    + arc["radius"] * np.sin(angle) * arc["v_axis"]
                )

        yield positions, fixed_wxyzs, {"step": step, "n_steps": n_steps, "t": t}
