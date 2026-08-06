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
