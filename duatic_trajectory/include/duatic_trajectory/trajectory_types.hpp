#pragma once

// assume all relevant definition headers have been included beforehand
#include <rclcpp/time.hpp>
#include <duatic_trajectory/kinematic_trajectory_exponential_approach.hpp>

namespace duatic::trajectory
{

template <typename ScalarT, geometry::KinematicOrder EvalOrderDepth, typename TimestampT = rclcpp::Time>
using ExponentialApproachPose3D =
    KinematicTrajectoryExponentialApproach<ScalarT, EvalOrderDepth, TimestampT, geometry::KinematicVariable3DT>;

template <typename ScalarT, typename TimestampT = rclcpp::Time>
using ExponentialApproachPose3DPoseState =
    ExponentialApproachPose3D<ScalarT, geometry::KinematicOrder::Pose, TimestampT>;
using ExponentialApproachPose3DPoseStated = ExponentialApproachPose3DPoseState<double>;

template <typename ScalarT, typename TimestampT = rclcpp::Time>
using ExponentialApproachPose3DTwistState =
    ExponentialApproachPose3D<ScalarT, geometry::KinematicOrder::Twist, TimestampT>;
using ExponentialApproachPose3DTwistStated = ExponentialApproachPose3DTwistState<double>;

static_assert(KinematicTrajectory<ExponentialApproachPose3DPoseStated>);
static_assert(KinematicTrajectory<ExponentialApproachPose3DTwistStated>);

}  // namespace duatic::trajectory
