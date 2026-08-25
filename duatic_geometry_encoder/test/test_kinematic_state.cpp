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
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <type_traits>

#include <rclcpp/time.hpp>

#include <duatic_data_annotation/annotation_stamped.hpp>
#include <duatic_data_annotation/annotation_timed.hpp>
#include <duatic_geometry/geometry.hpp>
#include <duatic_geometry_encoder/duatic_geometry_encoder.hpp>

namespace duatic::data_encoding
{
namespace
{

using duatic::geometry::Pose3Dd;
using duatic::geometry::Twist3Dd;
using duatic::geometry::Accel3Dd;
using duatic::geometry::StatePose3Dd;
using duatic::geometry::StateTwist3Dd;
using duatic::geometry::StateAccel3Dd;

using TimedStatePose = duatic::data_annotation::TimedData<StatePose3Dd, rclcpp::Time>;
using TimedStateTwist = duatic::data_annotation::TimedData<StateTwist3Dd, rclcpp::Time>;
using TimedStateAccel = duatic::data_annotation::TimedData<StateAccel3Dd, rclcpp::Time>;
using StampedStatePose = duatic::data_annotation::StampedData<StatePose3Dd, rclcpp::Time>;
using StampedStateTwist = duatic::data_annotation::StampedData<StateTwist3Dd, rclcpp::Time>;
using StampedStateAccel = duatic::data_annotation::StampedData<StateAccel3Dd, rclcpp::Time>;

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

  if constexpr (duatic::data_annotation::is_timed_v<DataT>) {
    EXPECT_EQ(decoded.time(), original.time());
  }
  if constexpr (duatic::data_annotation::is_stamped_v<DataT>) {
    EXPECT_EQ(decoded.frame_id(), original.frame_id());
  }

  MsgT message_from_decoded{};
  encode(decoded, message_from_decoded);
  EXPECT_EQ(message_from_decoded, message);
}

// ---------------------------------------------------------------------------
// msg_t / msg_stamped_t traits
// ---------------------------------------------------------------------------

// cppcheck-suppress syntaxError  // cppcheck doesn't know the TEST() gtest macro
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
}  // namespace duatic::data_encoding

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
