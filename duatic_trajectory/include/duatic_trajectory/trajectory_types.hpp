/*
 * Copyright 2026 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

// assume all relevant definition headers have been included beforehand
#include <rclcpp/time.hpp>
#include <duatic_geometry/kinematic_order.hpp>
#include <duatic_trajectory/kinematic_trajectory_exponential_approach.hpp>

namespace duatic::trajectory
{

template <typename ScalarT, typename TimestampT = rclcpp::Time,
          geometry::KinematicOrder ContinuityOrder = geometry::KinematicOrder::Accel,
          KinematicTrajectorySettingsExponentialApproach KinematicTrajectorySettingsT =
              KinematicTrajectorySettingsExponentialApproachDefault<ScalarT>>
using ExponentialApproachPose3D =
    KinematicTrajectoryExponentialApproach<ScalarT, TimestampT, ContinuityOrder, geometry::KinematicVariable3DT,
                                           KinematicTrajectorySettingsT>;

template <typename ScalarT, typename TimestampT = rclcpp::Time,
          KinematicTrajectorySettingsExponentialApproach KinematicTrajectorySettingsT =
              KinematicTrajectorySettingsExponentialApproachDefault<ScalarT>>
using ExponentialApproachPose3DC1 =
    ExponentialApproachPose3D<ScalarT, TimestampT, geometry::KinematicOrder::Twist, KinematicTrajectorySettingsT>;
using ExponentialApproachPose3DC1d = ExponentialApproachPose3DC1<double>;
template <typename ScalarT, typename TimestampT = rclcpp::Time,
          KinematicTrajectorySettingsExponentialApproach KinematicTrajectorySettingsT =
              KinematicTrajectorySettingsExponentialApproachDefault<ScalarT>>
using ExponentialApproachPose3DC2 =
    ExponentialApproachPose3D<ScalarT, TimestampT, geometry::KinematicOrder::Accel, KinematicTrajectorySettingsT>;
using ExponentialApproachPose3DC2d = ExponentialApproachPose3DC2<double>;

static_assert(is_kinematic_trajectory_v<ExponentialApproachPose3DC1d>);
static_assert(is_kinematic_trajectory_v<ExponentialApproachPose3DC2d>);

}  // namespace duatic::trajectory
