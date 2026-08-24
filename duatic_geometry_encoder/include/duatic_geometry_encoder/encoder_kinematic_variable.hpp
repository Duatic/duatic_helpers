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

#include <duatic_data_encoding/encoder.hpp>

using duatic::geometry::KinematicOrder;
using duatic::geometry::KinematicVariable;

namespace duatic::data_encoding
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

// specialization for KinematicVariable: only orders with
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
    } else if constexpr (order == KinematicOrder::Accel) {
      return message.accel;
    } else {
      static_assert(order <= KinematicOrder::Accel, "FactoryEncoder<KinematicVariable>::unstamp() is not yet "
                                                    "implemented for KinematicOrder higher than Accel");
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

}  // namespace duatic::data_encoding
