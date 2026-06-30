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


def smooth_and_limit(target_q, smoothed_q, last_q, alpha, max_velocity, dt):
    """Alpha-filter smoothing with velocity clamping.

    Applies exponential smoothing (alpha filter) to the target joint configuration,
    then clamps the per-joint velocity to max_velocity.

    Args:
        target_q: (n,) desired joint configuration
        smoothed_q: (n,) previous smoothed value, or None on first call
        last_q: (n,) last published smoothed value (for velocity limiting), or None
        alpha: smoothing factor in [0, 1]. Higher = more responsive, lower = smoother
        max_velocity: maximum joint velocity in rad/s
        dt: time step in seconds

    Returns:
        (n,) smoothed and velocity-limited joint configuration
    """
    target_q = np.asarray(target_q, dtype=np.float64)

    if smoothed_q is None:
        result = target_q.copy()
    else:
        smoothed_q = np.asarray(smoothed_q, dtype=np.float64)
        result = alpha * target_q + (1.0 - alpha) * smoothed_q

    if last_q is not None:
        last_q = np.asarray(last_q, dtype=np.float64)
        max_delta = max_velocity * dt
        delta = result - last_q
        result = last_q + np.clip(delta, -max_delta, max_delta)

    return result
