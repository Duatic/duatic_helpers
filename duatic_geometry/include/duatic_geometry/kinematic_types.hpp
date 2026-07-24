#pragma once

#include <duatic_geometry/kinematic_state_3d.hpp>
#include <duatic_geometry/timed.hpp>
#include <duatic_geometry/stamped.hpp>

#include <rclcpp/time.hpp>

namespace duatic::geometry
{

// assume the one general type template 'KinematicVariable3DT' has been defined beforehand
// derive all other types from this one

// Raw templates and types
template <typename ScalarT>
using Pose3D = KinematicVariable3DT<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Pose)>;
template <typename ScalarT>
using Twist3D = KinematicVariable3DT<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Twist)>;
template <typename ScalarT>
using Accel3D = KinematicVariable3DT<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Accel)>;
template <typename ScalarT>
using Jerk3D = KinematicVariable3DT<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Jerk)>;
template <typename ScalarT>
using Snap3D = KinematicVariable3DT<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Snap)>;

using Pose3Dd = Pose3D<double>;
using Twist3Dd = Twist3D<double>;
using Accel3Dd = Accel3D<double>;
using Jerk3Dd = Jerk3D<double>;
using Snap3Dd = Snap3D<double>;

// Assert concept compliance
static_assert(KinematicVariable3D<Pose3Dd>);
static_assert(KinematicDiffVariable3D<Twist3Dd>);
static_assert(KinematicDiffVariable3D<Accel3Dd>);
static_assert(KinematicDiffVariable3D<Jerk3Dd>);
static_assert(KinematicDiffVariable3D<Snap3Dd>);

// Static Assert Correct Variable Diff
static_assert(is_kinematic_diff_of_v<Twist3Dd, Pose3Dd>);
static_assert(is_kinematic_diff_of_v<Accel3Dd, Twist3Dd>);
static_assert(is_kinematic_diff_of_v<Jerk3Dd, Accel3Dd>);
static_assert(is_kinematic_diff_of_v<Snap3Dd, Jerk3Dd>);

// Raw State templates and types
template <typename ScalarT>
using StateToPose3D = KinematicState3D<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Pose)>;
template <typename ScalarT>
using StateToTwist3D = KinematicState3D<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Twist)>;
template <typename ScalarT>
using StateToAccel3D = KinematicState3D<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Accel)>;
template <typename ScalarT>
using StateToJerk3D = KinematicState3D<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Jerk)>;
template <typename ScalarT>
using StateToSnap3D = KinematicState3D<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Snap)>;

using StateToPose3Dd = StateToPose3D<double>;
using StateToTwist3Dd = StateToTwist3D<double>;
using StateToAccel3Dd = StateToAccel3D<double>;
using StateToJerk3Dd = StateToJerk3D<double>;
using StateToSnap3Dd = StateToSnap3D<double>;

// Timed templates and types
template <typename ScalarT, KinematicOrderT Order, typename TimestampT = rclcpp::Time>
using TimedKinematicVariable3DT = Timed<KinematicVariable3DT<ScalarT, Order>, TimestampT>;

template <typename ScalarT, typename TimestampT = rclcpp::Time>
using TimedPose3D = TimedKinematicVariable3DT<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Pose), TimestampT>;
template <typename ScalarT, typename TimestampT = rclcpp::Time>
using TimedTwist3D =
    TimedKinematicVariable3DT<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Twist), TimestampT>;
template <typename ScalarT, typename TimestampT = rclcpp::Time>
using TimedAccel3D =
    TimedKinematicVariable3DT<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Accel), TimestampT>;
template <typename ScalarT, typename TimestampT = rclcpp::Time>
using TimedJerk3D = TimedKinematicVariable3DT<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Jerk), TimestampT>;
template <typename ScalarT, typename TimestampT = rclcpp::Time>
using TimedSnap3D = TimedKinematicVariable3DT<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Snap), TimestampT>;

using TimedPose3Dd = TimedPose3D<double>;
using TimedTwist3Dd = TimedTwist3D<double>;
using TimedAccel3Dd = TimedAccel3D<double>;
using TimedJerk3Dd = TimedJerk3D<double>;
using TimedSnap3Dd = TimedSnap3D<double>;

// Stamped templates and types
template <typename ScalarT, KinematicOrderT Order, typename TimestampT = rclcpp::Time>
using StampedKinematicVariable3DT = Stamped<KinematicVariable3DT<ScalarT, Order>, TimestampT>;

template <typename ScalarT, typename TimestampT = rclcpp::Time>
using StampedPose3D =
    StampedKinematicVariable3DT<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Pose), TimestampT>;
template <typename ScalarT, typename TimestampT = rclcpp::Time>
using StampedTwist3D =
    StampedKinematicVariable3DT<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Twist), TimestampT>;
template <typename ScalarT, typename TimestampT = rclcpp::Time>
using StampedAccel3D =
    StampedKinematicVariable3DT<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Accel), TimestampT>;
template <typename ScalarT, typename TimestampT = rclcpp::Time>
using StampedJerk3D =
    StampedKinematicVariable3DT<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Jerk), TimestampT>;
template <typename ScalarT, typename TimestampT = rclcpp::Time>
using StampedSnap3D =
    StampedKinematicVariable3DT<ScalarT, static_cast<KinematicOrderT>(KinematicOrder::Snap), TimestampT>;

using StampedPose3Dd = StampedPose3D<double>;
using StampedTwist3Dd = StampedTwist3D<double>;
using StampedAccel3Dd = StampedAccel3D<double>;
using StampedJerk3Dd = StampedJerk3D<double>;
using StampedSnap3Dd = StampedSnap3D<double>;

}  // namespace duatic::geometry
