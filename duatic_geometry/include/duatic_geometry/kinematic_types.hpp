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

#include <duatic_data_annotation/annotation.hpp>
#include <duatic_data_annotation/annotation_stamped.hpp>
#include <duatic_data_annotation/annotation_timed.hpp>
#include <duatic_geometry/kinematic_state.hpp>

#include <rclcpp/time.hpp>

using duatic::data_annotation::Stamped;
using duatic::data_annotation::StampedData;
using duatic::data_annotation::Timed;
using duatic::data_annotation::TimedData;

namespace duatic::geometry
{

// assume the one general type template 'KinematicVariable3DT' has been defined beforehand
// derive all other types from this one

// Raw templates and types
template <typename ScalarT>
using Pose3D = KinematicVariable3DT<ScalarT, KinematicOrder::Pose>;
template <typename ScalarT>
using Twist3D = KinematicVariable3DT<ScalarT, KinematicOrder::Twist>;
template <typename ScalarT>
using Accel3D = KinematicVariable3DT<ScalarT, KinematicOrder::Accel>;
template <typename ScalarT>
using Jerk3D = KinematicVariable3DT<ScalarT, KinematicOrder::Jerk>;
template <typename ScalarT>
using Snap3D = KinematicVariable3DT<ScalarT, KinematicOrder::Snap>;

using Pose3Dd = Pose3D<double>;
using Twist3Dd = Twist3D<double>;
using Accel3Dd = Accel3D<double>;
using Jerk3Dd = Jerk3D<double>;
using Snap3Dd = Snap3D<double>;

// Assert concept compliance
static_assert(KinematicVariable<Pose3Dd>);
static_assert(KinematicDiffVariable<Twist3Dd>);
static_assert(KinematicDiffVariable<Accel3Dd>);
static_assert(KinematicDiffVariable<Jerk3Dd>);
static_assert(KinematicDiffVariable<Snap3Dd>);

// Static Assert Correct Variable Diff
static_assert(is_kinematic_diff_of_v<Twist3Dd, Pose3Dd>);
static_assert(is_kinematic_diff_of_v<Accel3Dd, Twist3Dd>);
static_assert(is_kinematic_diff_of_v<Jerk3Dd, Accel3Dd>);
static_assert(is_kinematic_diff_of_v<Snap3Dd, Jerk3Dd>);

// Raw State templates and types
template <typename ScalarT>
using StatePose3D = KinematicState<ScalarT, KinematicOrder::Pose>;
template <typename ScalarT>
using StateTwist3D = KinematicState<ScalarT, KinematicOrder::Twist>;
template <typename ScalarT>
using StateAccel3D = KinematicState<ScalarT, KinematicOrder::Accel>;
template <typename ScalarT>
using StateToJerk3D = KinematicState<ScalarT, KinematicOrder::Jerk>;
template <typename ScalarT>
using StateToSnap3D = KinematicState<ScalarT, KinematicOrder::Snap>;

using StatePose3Dd = StatePose3D<double>;
using StateTwist3Dd = StateTwist3D<double>;
using StateAccel3Dd = StateAccel3D<double>;
using StateToJerk3Dd = StateToJerk3D<double>;
using StateToSnap3Dd = StateToSnap3D<double>;

// Timed templates and types
template <typename ScalarT, KinematicOrder Order, typename TimestampT = rclcpp::Time>
using TimedKinematicVariable3DT = TimedData<KinematicVariable3DT<ScalarT, Order>, TimestampT>;

template <typename ScalarT, typename TimestampT = rclcpp::Time>
using TimedPose3D = TimedKinematicVariable3DT<ScalarT, KinematicOrder::Pose, TimestampT>;
template <typename ScalarT, typename TimestampT = rclcpp::Time>
using TimedTwist3D = TimedKinematicVariable3DT<ScalarT, KinematicOrder::Twist, TimestampT>;
template <typename ScalarT, typename TimestampT = rclcpp::Time>
using TimedAccel3D = TimedKinematicVariable3DT<ScalarT, KinematicOrder::Accel, TimestampT>;
template <typename ScalarT, typename TimestampT = rclcpp::Time>
using TimedJerk3D = TimedKinematicVariable3DT<ScalarT, KinematicOrder::Jerk, TimestampT>;
template <typename ScalarT, typename TimestampT = rclcpp::Time>
using TimedSnap3D = TimedKinematicVariable3DT<ScalarT, KinematicOrder::Snap, TimestampT>;

using TimedPose3Dd = TimedPose3D<double>;
using TimedTwist3Dd = TimedTwist3D<double>;
using TimedAccel3Dd = TimedAccel3D<double>;
using TimedJerk3Dd = TimedJerk3D<double>;
using TimedSnap3Dd = TimedSnap3D<double>;

// Assert concept compliance
static_assert(Timed<TimedPose3Dd>);
static_assert(Timed<TimedTwist3Dd>);
static_assert(Timed<TimedAccel3Dd>);
static_assert(Timed<TimedJerk3Dd>);
static_assert(Timed<TimedSnap3Dd>);

// Stamped templates and types
template <typename ScalarT, KinematicOrder Order, typename TimestampT = rclcpp::Time>
using StampedKinematicVariable3DT = StampedData<KinematicVariable3DT<ScalarT, Order>, TimestampT>;

template <typename ScalarT, typename TimestampT = rclcpp::Time>
using StampedPose3D = StampedKinematicVariable3DT<ScalarT, KinematicOrder::Pose, TimestampT>;
template <typename ScalarT, typename TimestampT = rclcpp::Time>
using StampedTwist3D = StampedKinematicVariable3DT<ScalarT, KinematicOrder::Twist, TimestampT>;
template <typename ScalarT, typename TimestampT = rclcpp::Time>
using StampedAccel3D = StampedKinematicVariable3DT<ScalarT, KinematicOrder::Accel, TimestampT>;
template <typename ScalarT, typename TimestampT = rclcpp::Time>
using StampedJerk3D = StampedKinematicVariable3DT<ScalarT, KinematicOrder::Jerk, TimestampT>;
template <typename ScalarT, typename TimestampT = rclcpp::Time>
using StampedSnap3D = StampedKinematicVariable3DT<ScalarT, KinematicOrder::Snap, TimestampT>;

using StampedPose3Dd = StampedPose3D<double>;
using StampedTwist3Dd = StampedTwist3D<double>;
using StampedAccel3Dd = StampedAccel3D<double>;
using StampedJerk3Dd = StampedJerk3D<double>;
using StampedSnap3Dd = StampedSnap3D<double>;

// Assert concept compliance
static_assert(Stamped<StampedPose3Dd>);
static_assert(Stamped<StampedTwist3Dd>);
static_assert(Stamped<StampedAccel3Dd>);
static_assert(Stamped<StampedJerk3Dd>);
static_assert(Stamped<StampedSnap3Dd>);

}  // namespace duatic::geometry
