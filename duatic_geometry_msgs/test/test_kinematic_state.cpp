#include <gtest/gtest.h>

#include <type_traits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <duatic_geometry/geometry.hpp>
#include <duatic_geometry_msgs/duatic_geometry_msgs.hpp>

namespace duatic_geometry_msgs
{
namespace
{

using duatic::geometry::Accel3Dd;
using duatic::geometry::Pose3Dd;
using duatic::geometry::StateAccel3Dd;
using duatic::geometry::StatePose3Dd;
using duatic::geometry::StateTwist3Dd;
using duatic::geometry::Twist3Dd;

using TimedStatePose = duatic::geometry::TimedData<StatePose3Dd, rclcpp::Time>;
using TimedStateTwist = duatic::geometry::TimedData<StateTwist3Dd, rclcpp::Time>;
using TimedStateAccel = duatic::geometry::TimedData<StateAccel3Dd, rclcpp::Time>;
using StampedStatePose = duatic::geometry::StampedData<StatePose3Dd, rclcpp::Time>;
using StampedStateTwist = duatic::geometry::StampedData<StateTwist3Dd, rclcpp::Time>;
using StampedStateAccel = duatic::geometry::StampedData<StateAccel3Dd, rclcpp::Time>;

// Same shape as ExpectEncodeDecodeRoundTrip in test_kinematic_variable.cpp, but compares
// pose/twist/accel sub-variables (as many as the state's order depth provides) instead
// of a single linear()/angular() pair.
template <typename MsgT, typename DataT>
void ExpectEncodeDecodeRoundTrip(const DataT& original)
{
  MsgT message{};
  encode(original, message);

  DataT decoded{};
  decode(message, decoded);

  EXPECT_TRUE(decoded.pose().linear().isApprox(original.pose().linear()));
  EXPECT_TRUE(decoded.pose().angular().isApprox(original.pose().angular()));

  if constexpr (DataT::kinematic_order_depth >= duatic::geometry::KinematicOrder::Twist) {
    EXPECT_TRUE(decoded.twist().linear().isApprox(original.twist().linear()));
    EXPECT_TRUE(decoded.twist().angular().isApprox(original.twist().angular()));
  }
  if constexpr (DataT::kinematic_order_depth >= duatic::geometry::KinematicOrder::Accel) {
    EXPECT_TRUE(decoded.accel().linear().isApprox(original.accel().linear()));
    EXPECT_TRUE(decoded.accel().angular().isApprox(original.accel().angular()));
  }

  if constexpr (duatic::geometry::is_timed_v<DataT>) {
    EXPECT_EQ(decoded.time(), original.time());
  }
  if constexpr (duatic::geometry::is_stamped_v<DataT>) {
    EXPECT_EQ(decoded.frame_id(), original.frame_id());
  }

  MsgT message_from_decoded{};
  encode(decoded, message_from_decoded);
  EXPECT_EQ(message_from_decoded, message);
}

// ---------------------------------------------------------------------------
// msg_t / msg_stamped_t traits
// ---------------------------------------------------------------------------

TEST(KinematicStateMsgTraits, PoseDepthMapsToStatePose)
{
  static_assert(std::is_same_v<msg_t<StatePose3Dd>, ::duatic_geometry_msgs::msg::StatePose>);
  static_assert(std::is_same_v<msg_stamped_t<StatePose3Dd>, ::duatic_geometry_msgs::msg::StatePoseStamped>);
  SUCCEED();
}

TEST(KinematicStateMsgTraits, TwistDepthMapsToStateTwist)
{
  static_assert(std::is_same_v<msg_t<StateTwist3Dd>, ::duatic_geometry_msgs::msg::StateTwist>);
  static_assert(std::is_same_v<msg_stamped_t<StateTwist3Dd>, ::duatic_geometry_msgs::msg::StateTwistStamped>);
  SUCCEED();
}

TEST(KinematicStateMsgTraits, AccelDepthMapsToStateAccel)
{
  static_assert(std::is_same_v<msg_t<StateAccel3Dd>, ::duatic_geometry_msgs::msg::StateAccel>);
  static_assert(std::is_same_v<msg_stamped_t<StateAccel3Dd>, ::duatic_geometry_msgs::msg::StateAccelStamped>);
  SUCCEED();
}

// ---------------------------------------------------------------------------
// plain: encode / decode round trips
// ---------------------------------------------------------------------------

TEST(KinematicStateEncodeDecode, PoseDepthRoundTripsThroughPlainMessage)
{
  const StatePose3Dd original(Pose3Dd(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity()));
  ExpectEncodeDecodeRoundTrip<msg_t<StatePose3Dd>>(original);
}

TEST(KinematicStateEncodeDecode, PoseDepthRoundTripsThroughStampedMessage)
{
  const StatePose3Dd original(Pose3Dd(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity()));
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<StatePose3Dd>>(original);
}

TEST(KinematicStateEncodeDecode, TwistDepthRoundTripsThroughPlainMessage)
{
  const StateTwist3Dd original(Pose3Dd(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity()),
                                 Twist3Dd(Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0.4, 0.5, 0.6)));
  ExpectEncodeDecodeRoundTrip<msg_t<StateTwist3Dd>>(original);
}

TEST(KinematicStateEncodeDecode, TwistDepthRoundTripsThroughStampedMessage)
{
  const StateTwist3Dd original(Pose3Dd(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity()),
                                 Twist3Dd(Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0.4, 0.5, 0.6)));
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<StateTwist3Dd>>(original);
}

TEST(KinematicStateEncodeDecode, AccelDepthRoundTripsThroughPlainMessage)
{
  const StateAccel3Dd original(Pose3Dd(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity()),
                                 Twist3Dd(Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0.4, 0.5, 0.6)),
                                 Accel3Dd(Eigen::Vector3d(0.7, 0.8, 0.9), Eigen::Vector3d(1.0, 1.1, 1.2)));
  ExpectEncodeDecodeRoundTrip<msg_t<StateAccel3Dd>>(original);
}

TEST(KinematicStateEncodeDecode, AccelDepthRoundTripsThroughStampedMessage)
{
  const StateAccel3Dd original(Pose3Dd(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity()),
                                 Twist3Dd(Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0.4, 0.5, 0.6)),
                                 Accel3Dd(Eigen::Vector3d(0.7, 0.8, 0.9), Eigen::Vector3d(1.0, 1.1, 1.2)));
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<StateAccel3Dd>>(original);
}

// ---------------------------------------------------------------------------
// timed: encode / decode round trips (msg_stamped_t only, see the analogous note
// in test_kinematic_variable.cpp)
// ---------------------------------------------------------------------------

TEST(KinematicStateEncodeDecode, TimedPoseDepthRoundTripsThroughStampedMessage)
{
  const TimedStatePose original(rclcpp::Time(42, 7, RCL_ROS_TIME),
                                Pose3Dd(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity()));
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<TimedStatePose>>(original);
}

TEST(KinematicStateEncodeDecode, TimedTwistDepthRoundTripsThroughStampedMessage)
{
  const TimedStateTwist original(rclcpp::Time(1, 2, RCL_ROS_TIME),
                                 Pose3Dd(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity()),
                                 Twist3Dd(Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0.4, 0.5, 0.6)));
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<TimedStateTwist>>(original);
}

TEST(KinematicStateEncodeDecode, TimedAccelDepthRoundTripsThroughStampedMessage)
{
  const TimedStateAccel original(rclcpp::Time(3, 4, RCL_ROS_TIME),
                                 Pose3Dd(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity()),
                                 Twist3Dd(Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0.4, 0.5, 0.6)),
                                 Accel3Dd(Eigen::Vector3d(0.7, 0.8, 0.9), Eigen::Vector3d(1.0, 1.1, 1.2)));
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<TimedStateAccel>>(original);
}

// ---------------------------------------------------------------------------
// stamped: encode / decode round trips (header stamp and frame_id both transferred)
// ---------------------------------------------------------------------------

TEST(KinematicStateEncodeDecode, StampedPoseDepthRoundTripsThroughStampedMessage)
{
  const StampedStatePose original(rclcpp::Time(5, 6, RCL_ROS_TIME), "base_link",
                                  Pose3Dd(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity()));
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<StampedStatePose>>(original);
}

TEST(KinematicStateEncodeDecode, StampedTwistDepthRoundTripsThroughStampedMessage)
{
  const StampedStateTwist original(rclcpp::Time(7, 8, RCL_ROS_TIME), "odom",
                                   Pose3Dd(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity()),
                                   Twist3Dd(Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0.4, 0.5, 0.6)));
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<StampedStateTwist>>(original);
}

TEST(KinematicStateEncodeDecode, StampedAccelDepthRoundTripsThroughStampedMessage)
{
  const StampedStateAccel original(rclcpp::Time(9, 10, RCL_ROS_TIME), "world",
                                   Pose3Dd(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity()),
                                   Twist3Dd(Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0.4, 0.5, 0.6)),
                                   Accel3Dd(Eigen::Vector3d(0.7, 0.8, 0.9), Eigen::Vector3d(1.0, 1.1, 1.2)));
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<StampedStateAccel>>(original);
}

}  // namespace
}  // namespace duatic_geometry_msgs

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
