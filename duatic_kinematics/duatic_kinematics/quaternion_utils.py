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

"""Quaternion utilities for both wxyz and xyzw conventions."""

import math
import numpy as np


# ---------------------------------------------------------------------------
# Format conversion
# ---------------------------------------------------------------------------


def xyzw_to_wxyz(q):
    """Convert quaternion from [x, y, z, w] to [w, x, y, z]."""
    q = np.asarray(q, dtype=np.float64).reshape(4)
    return np.array([q[3], q[0], q[1], q[2]], dtype=np.float64)


def wxyz_to_xyzw(q):
    """Convert quaternion from [w, x, y, z] to [x, y, z, w]."""
    q = np.asarray(q, dtype=np.float64).reshape(4)
    return np.array([q[1], q[2], q[3], q[0]], dtype=np.float64)


# ---------------------------------------------------------------------------
# Normalization
# ---------------------------------------------------------------------------


def normalize_quat(q):
    """Normalize a quaternion (any convention)."""
    q = np.asarray(q, dtype=np.float64).reshape(4)
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError("Quaternion norm is near zero")
    return q / n


# ---------------------------------------------------------------------------
# wxyz operations
# ---------------------------------------------------------------------------


def quat_multiply_wxyz(q1, q2):
    """Hamilton product of two wxyz quaternions."""
    q1 = normalize_quat(q1)
    q2 = normalize_quat(q2)
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return normalize_quat(
        np.array(
            [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ],
            dtype=np.float64,
        )
    )


def quat_from_axis_angle_wxyz(axis, angle_rad):
    """Create wxyz quaternion from axis-angle representation."""
    axis = np.asarray(axis, dtype=np.float64).reshape(3)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        raise ValueError("Axis norm is near zero")
    axis = axis / n
    half = 0.5 * float(angle_rad)
    s = math.sin(half)
    c = math.cos(half)
    return np.array([c, axis[0] * s, axis[1] * s, axis[2] * s], dtype=np.float64)


def quat_slerp_wxyz(q0, q1, t):
    """Spherical linear interpolation between two wxyz quaternions."""
    q0 = normalize_quat(q0)
    q1 = normalize_quat(q1)

    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    dot = np.clip(dot, -1.0, 1.0)

    if dot > 0.9995:
        return normalize_quat(q0 + t * (q1 - q0))

    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * t
    sin_theta = math.sin(theta)

    s0 = math.sin(theta_0 - theta) / (sin_theta_0 + 1e-12)
    s1 = sin_theta / (sin_theta_0 + 1e-12)
    return s0 * q0 + s1 * q1


def quat_angle_error_wxyz(q1, q2):
    """Compute angular distance (radians) between two wxyz quaternions."""
    q1 = normalize_quat(q1)
    q2 = normalize_quat(q2)
    d = float(np.clip(np.abs(np.dot(q1, q2)), 0.0, 1.0))
    return float(2.0 * math.acos(d))


# ---------------------------------------------------------------------------
# xyzw operations
# ---------------------------------------------------------------------------


def quat_multiply_xyzw(q1, q2):
    """Hamilton product of two xyzw quaternions."""
    q1 = normalize_quat(q1)
    q2 = normalize_quat(q2)
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return normalize_quat(
        np.array(
            [
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            ],
            dtype=np.float64,
        )
    )


def quat_from_axis_angle_xyzw(axis, angle_rad):
    """Create xyzw quaternion from axis-angle representation."""
    axis = np.asarray(axis, dtype=np.float64).reshape(3)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        raise ValueError("Axis norm is near zero")
    axis = axis / n
    half = 0.5 * float(angle_rad)
    s = math.sin(half)
    c = math.cos(half)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, c], dtype=np.float64)


# ---------------------------------------------------------------------------
# Pose helpers (7D: [x, y, z, qx, qy, qz, qw] or [x, y, z, qw, qx, qy, qz])
# ---------------------------------------------------------------------------


def translate_pose(pose, dx=0.0, dy=0.0, dz=0.0):
    """Translate pose position in the base frame."""
    pose = np.asarray(pose, dtype=np.float64).copy()
    pose[0] += float(dx)
    pose[1] += float(dy)
    pose[2] += float(dz)
    return pose


def rotate_pose_local_axis_xyzw(pose_xyzw, axis, angle_rad):
    """Rotate a 7D pose (pos + xyzw quat) about a local TCP axis.

    Args:
        pose_xyzw: (7,) array [x, y, z, qx, qy, qz, qw]
        axis: (3,) local axis to rotate about
        angle_rad: rotation angle in radians

    Returns:
        (7,) rotated pose
    """
    pose = np.asarray(pose_xyzw, dtype=np.float64).copy()
    q_current = pose[3:7]
    q_delta = quat_from_axis_angle_xyzw(axis, angle_rad)
    pose[3:7] = quat_multiply_xyzw(q_current, q_delta)
    return pose
