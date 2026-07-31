#pragma once

// assume all relevant definition headers have been included beforehand
#include <rclcpp/time.hpp>
#include <duatic_trajectory/kinematic_trajectory_exponential_approach.hpp>

namespace duatic::trajectory
{

template <typename ScalarT, typename TimestampT = rclcpp::Time,
          KinematicTrajectorySettingsExponentialApproach KinematicTrajectorySettingsT = KinematicTrajectorySettingsExponentialApproachDefault<ScalarT>>
using ExponentialApproachPose3D =
    KinematicTrajectoryExponentialApproach<ScalarT, TimestampT, geometry::KinematicVariable3DT, KinematicTrajectorySettingsT>;
using ExponentialApproachPose3Dd = ExponentialApproachPose3D<double>;

static_assert(is_kinematic_trajectory_v<ExponentialApproachPose3Dd>);

}  // namespace duatic::trajectory
