#pragma once

#include <cstddef>
#include <type_traits>

#include <duatic_geometry/kinematic_order.hpp>
#include <duatic_geometry/kinematic_variable.hpp>

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <duatic_geometry_msgs/encoder.hpp>

using namespace duatic::geometry;

namespace duatic_geometry_msgs
{

template <KinematicOrder Order>
struct KinematicVariableMsgTypeHelper;
template <>
struct KinematicVariableMsgTypeHelper<KinematicOrder::Pose>
{
  using msg = geometry_msgs::msg::Pose;
  using msg_stamped = geometry_msgs::msg::PoseStamped;
};
template <>
struct KinematicVariableMsgTypeHelper<KinematicOrder::Twist>
{
  using msg = geometry_msgs::msg::Twist;
  using msg_stamped = geometry_msgs::msg::TwistStamped;
};
template <>
struct KinematicVariableMsgTypeHelper<KinematicOrder::Accel>
{
  using msg = geometry_msgs::msg::Accel;
  using msg_stamped = geometry_msgs::msg::AccelStamped;
};

// template concretization for KinematicVariable: only orders with
// a standard geometry_msgs analogue are supported (Pose, Twist, Accel).
template <KinematicVariable T>
class FactoryEncoder<T>
{
public:
  using DataType = T;
  static constexpr KinematicOrder order = DataType::kinematic_order;
  using msg = KinematicVariableMsgTypeHelper<order>::msg;
  using msg_stamped = KinematicVariableMsgTypeHelper<order>::msg_stamped;

private:
  template <typename MSG>
  inline static auto& unstamp(MSG& message)
  {
    if constexpr (order == KinematicOrder::Pose) {
      return message.pose;
    } else if constexpr (order == KinematicOrder::Twist) {
      return message.twist;
    } else {
      return message.accel;
    }
  }

public:
  template <typename MSG>
  static void encode(const T& data, MSG& message)
  {
    if constexpr (std::is_base_of_v<msg_stamped, MSG>) {
      encode(data, unstamp(message));
    } else {
      if constexpr (order == KinematicOrder::Pose) {
        message.position.x = data.linear()(0);
        message.position.y = data.linear()(1);
        message.position.z = data.linear()(2);
        message.orientation.x = data.angular().x();
        message.orientation.y = data.angular().y();
        message.orientation.z = data.angular().z();
        message.orientation.w = data.angular().w();
      } else {
        message.linear.x = data.linear()(0);
        message.linear.y = data.linear()(1);
        message.linear.z = data.linear()(2);
        message.angular.x = data.angular()(0);
        message.angular.y = data.angular()(1);
        message.angular.z = data.angular()(2);
      }
    }
  }

  template <typename MSG>
  static void decode(const MSG& message, T& data)
  {
    if constexpr (std::is_base_of_v<msg_stamped, MSG>) {
      decode(unstamp(message), data);
    } else {
      if constexpr (order == KinematicOrder::Pose) {
        data.linear()(0) = message.position.x;
        data.linear()(1) = message.position.y;
        data.linear()(2) = message.position.z;
        data.angular().x() = message.orientation.x;
        data.angular().y() = message.orientation.y;
        data.angular().z() = message.orientation.z;
        data.angular().w() = message.orientation.w;
      } else {
        data.linear()(0) = message.linear.x;
        data.linear()(1) = message.linear.y;
        data.linear()(2) = message.linear.z;
        data.angular()(0) = message.angular.x;
        data.angular()(1) = message.angular.y;
        data.angular()(2) = message.angular.z;
      }
    }
  }
};

}  // namespace duatic_geometry_msgs
