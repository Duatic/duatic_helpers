#pragma once

// assume all relevant definition headers have been included beforehand
#include <rclcpp/time.hpp>
#include <duatic_trajectory/kinematic_trajectory_exponential_approach.hpp>

namespace duatic::trajectory
{

template <typename ScalarT, geometry::KinematicOrder EvalOrderDepth, typename TimestampT = rclcpp::Time,
          KinematicLimitsExponentialApproach KinematicLimitsT = KinematicLimitsExponentialApproachDefault<ScalarT>>
using ExponentialApproachPose3D =
    KinematicTrajectoryExponentialApproach<ScalarT, EvalOrderDepth, TimestampT, geometry::KinematicVariable3DT,
                                           KinematicLimitsT>;

template <typename ScalarT, typename TimestampT = rclcpp::Time,
          KinematicLimitsExponentialApproach KinematicLimitsT = KinematicLimitsExponentialApproachDefault<ScalarT>>
using ExponentialApproachPose3DPoseState =
    ExponentialApproachPose3D<ScalarT, geometry::KinematicOrder::Pose, TimestampT, KinematicLimitsT>;
using ExponentialApproachPose3DPoseStated = ExponentialApproachPose3DPoseState<double>;

template <typename ScalarT, typename TimestampT = rclcpp::Time,
          KinematicLimitsExponentialApproach KinematicLimitsT = KinematicLimitsExponentialApproachDefault<ScalarT>>
using ExponentialApproachPose3DTwistState =
    ExponentialApproachPose3D<ScalarT, geometry::KinematicOrder::Twist, TimestampT, KinematicLimitsT>;
using ExponentialApproachPose3DTwistStated = ExponentialApproachPose3DTwistState<double>;

static_assert(is_kinematic_trajectory_v<ExponentialApproachPose3DPoseStated>);
static_assert(is_kinematic_trajectory_v<ExponentialApproachPose3DTwistStated>);

}  // namespace duatic::trajectory
