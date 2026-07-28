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
using duatic::geometry::StampedAccel3Dd;
using duatic::geometry::StampedPose3Dd;
using duatic::geometry::StampedTwist3Dd;
using duatic::geometry::TimedAccel3Dd;
using duatic::geometry::TimedPose3Dd;
using duatic::geometry::TimedTwist3Dd;
using duatic::geometry::Twist3Dd;

// Exercises both roundtrip directions: encode(original) -> message, decode(message)
// -> decoded should reproduce the original data, and re-encoding decoded should
// reproduce the same message (i.e. decode is a true inverse of encode, not just a
// one-way approximation).
template <typename MsgT, typename DataT>
void ExpectEncodeDecodeRoundTrip(const DataT& original)
{
  MsgT message{};
  encode(original, message);

  DataT decoded{};
  decode(message, decoded);

  EXPECT_TRUE(decoded.linear().isApprox(original.linear()));
  EXPECT_TRUE(decoded.angular().isApprox(original.angular()));

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

TEST(KinematicVariableMsgTraits, PoseMapsToGeometryMsgsPose)
{
  static_assert(std::is_same_v<msg_t<Pose3Dd>, ::geometry_msgs::msg::Pose>);
  static_assert(std::is_same_v<msg_stamped_t<Pose3Dd>, ::geometry_msgs::msg::PoseStamped>);
  SUCCEED();
}

TEST(KinematicVariableMsgTraits, TwistMapsToGeometryMsgsTwist)
{
  static_assert(std::is_same_v<msg_t<Twist3Dd>, ::geometry_msgs::msg::Twist>);
  static_assert(std::is_same_v<msg_stamped_t<Twist3Dd>, ::geometry_msgs::msg::TwistStamped>);
  SUCCEED();
}

TEST(KinematicVariableMsgTraits, AccelMapsToGeometryMsgsAccel)
{
  static_assert(std::is_same_v<msg_t<Accel3Dd>, ::geometry_msgs::msg::Accel>);
  static_assert(std::is_same_v<msg_stamped_t<Accel3Dd>, ::geometry_msgs::msg::AccelStamped>);
  SUCCEED();
}

// ---------------------------------------------------------------------------
// plain: encode / decode round trips
// ---------------------------------------------------------------------------

TEST(KinematicVariableEncodeDecode, PoseRoundTripsThroughPlainMessage)
{
  const Pose3Dd original(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5).normalized());
  ExpectEncodeDecodeRoundTrip<msg_t<Pose3Dd>>(original);
}

TEST(KinematicVariableEncodeDecode, PoseRoundTripsThroughStampedMessage)
{
  const Pose3Dd original(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5).normalized());
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<Pose3Dd>>(original);
}

TEST(KinematicVariableEncodeDecode, TwistRoundTripsThroughPlainMessage)
{
  const Twist3Dd original(Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0.4, 0.5, 0.6));
  ExpectEncodeDecodeRoundTrip<msg_t<Twist3Dd>>(original);
}

TEST(KinematicVariableEncodeDecode, TwistRoundTripsThroughStampedMessage)
{
  const Twist3Dd original(Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0.4, 0.5, 0.6));
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<Twist3Dd>>(original);
}

TEST(KinematicVariableEncodeDecode, AccelRoundTripsThroughPlainMessage)
{
  const Accel3Dd original(Eigen::Vector3d(0.7, 0.8, 0.9), Eigen::Vector3d(1.0, 1.1, 1.2));
  ExpectEncodeDecodeRoundTrip<msg_t<Accel3Dd>>(original);
}

TEST(KinematicVariableEncodeDecode, AccelRoundTripsThroughStampedMessage)
{
  const Accel3Dd original(Eigen::Vector3d(0.7, 0.8, 0.9), Eigen::Vector3d(1.0, 1.1, 1.2));
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<Accel3Dd>>(original);
}

// ---------------------------------------------------------------------------
// timed: encode / decode round trips (only the stamped message carries a header,
// so only msg_stamped_t is meaningful here -- see Factory<T>, where msg == msg_stamped
// for any Timed/Stamped T)
// ---------------------------------------------------------------------------

TEST(KinematicVariableEncodeDecode, TimedPoseRoundTripsThroughStampedMessage)
{
  const TimedPose3Dd original(rclcpp::Time(42, 7, RCL_ROS_TIME), Eigen::Vector3d(1.0, 2.0, 3.0),
                              Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5).normalized());
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<TimedPose3Dd>>(original);
}

TEST(KinematicVariableEncodeDecode, TimedTwistRoundTripsThroughStampedMessage)
{
  const TimedTwist3Dd original(rclcpp::Time(1, 2, RCL_ROS_TIME), Eigen::Vector3d(0.1, 0.2, 0.3),
                               Eigen::Vector3d(0.4, 0.5, 0.6));
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<TimedTwist3Dd>>(original);
}

TEST(KinematicVariableEncodeDecode, TimedAccelRoundTripsThroughStampedMessage)
{
  const TimedAccel3Dd original(rclcpp::Time(3, 4, RCL_ROS_TIME), Eigen::Vector3d(0.7, 0.8, 0.9),
                               Eigen::Vector3d(1.0, 1.1, 1.2));
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<TimedAccel3Dd>>(original);
}

// ---------------------------------------------------------------------------
// stamped: encode / decode round trips (header stamp and frame_id both transferred)
// ---------------------------------------------------------------------------

TEST(KinematicVariableEncodeDecode, StampedPoseRoundTripsThroughStampedMessage)
{
  const StampedPose3Dd original(rclcpp::Time(5, 6, RCL_ROS_TIME), "base_link", Eigen::Vector3d(1.0, 2.0, 3.0),
                                Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5).normalized());
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<StampedPose3Dd>>(original);
}

TEST(KinematicVariableEncodeDecode, StampedTwistRoundTripsThroughStampedMessage)
{
  const StampedTwist3Dd original(rclcpp::Time(7, 8, RCL_ROS_TIME), "odom", Eigen::Vector3d(0.1, 0.2, 0.3),
                                 Eigen::Vector3d(0.4, 0.5, 0.6));
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<StampedTwist3Dd>>(original);
}

TEST(KinematicVariableEncodeDecode, StampedAccelRoundTripsThroughStampedMessage)
{
  const StampedAccel3Dd original(rclcpp::Time(9, 10, RCL_ROS_TIME), "world", Eigen::Vector3d(0.7, 0.8, 0.9),
                                 Eigen::Vector3d(1.0, 1.1, 1.2));
  ExpectEncodeDecodeRoundTrip<msg_stamped_t<StampedAccel3Dd>>(original);
}

}  // namespace
}  // namespace duatic_geometry_msgs

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
