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

#include <type_traits>

#include <duatic_geometry/kinematic_order.hpp>
#include <duatic_geometry/kinematic_state.hpp>

#include <duatic_data_encoding/encoder.hpp>
#include <duatic_geometry_encoder/encoder_kinematic_variable.hpp>

#include <duatic_geometry_msgs/msg/state_accel.hpp>
#include <duatic_geometry_msgs/msg/state_accel_stamped.hpp>
#include <duatic_geometry_msgs/msg/state_pose.hpp>
#include <duatic_geometry_msgs/msg/state_pose_stamped.hpp>
#include <duatic_geometry_msgs/msg/state_twist.hpp>
#include <duatic_geometry_msgs/msg/state_twist_stamped.hpp>

using duatic::geometry::is_kinematic_state_v;
using duatic::geometry::KinematicOrder;

namespace duatic::data_encoding
{

template <KinematicOrder OrderDepth>
struct KinematicStateMsgTypeHelper;
template <>
struct KinematicStateMsgTypeHelper<KinematicOrder::Pose>
{
  using msg = duatic_geometry_msgs::msg::StatePose;
  using msg_stamped = duatic_geometry_msgs::msg::StatePoseStamped;
};
template <>
struct KinematicStateMsgTypeHelper<KinematicOrder::Twist>
{
  using msg = duatic_geometry_msgs::msg::StateTwist;
  using msg_stamped = duatic_geometry_msgs::msg::StateTwistStamped;
};
template <>
struct KinematicStateMsgTypeHelper<KinematicOrder::Accel>
{
  using msg = duatic_geometry_msgs::msg::StateAccel;
  using msg_stamped = duatic_geometry_msgs::msg::StateAccelStamped;
};

// specialization for KinematicState
template <typename T>
  requires is_kinematic_state_v<T>
class FactoryEncoder<T>
{
public:
  using DataType = T;
  static constexpr KinematicOrder order_depth = DataType::kinematic_order_depth;
  using msg = typename KinematicStateMsgTypeHelper<order_depth>::msg;
  using msg_stamped = typename KinematicStateMsgTypeHelper<order_depth>::msg_stamped;

private:
  template <KinematicOrder Order>
  using VariableType = typename T::template KinematicVariable3DType<Order>;

public:
  template <typename MSG>
  static void encode(const T& data, MSG& message)
  {
    static_assert(order_depth == KinematicOrder::Pose || order_depth == KinematicOrder::Twist ||
                      order_depth == KinematicOrder::Accel,
                  "FactoryEncoder<KinematicState>::encode() is not yet implemented for this KinematicOrder depth.");
    if constexpr (std::is_base_of_v<msg_stamped, MSG>) {
      encode(data, message.state);
    } else {
      FactoryEncoder<VariableType<KinematicOrder::Pose>>::encode(data.pose(), message.pose);
      if constexpr (order_depth >= KinematicOrder::Twist) {
        FactoryEncoder<VariableType<KinematicOrder::Twist>>::encode(data.twist(), message.twist);
      }
      if constexpr (order_depth >= KinematicOrder::Accel) {
        FactoryEncoder<VariableType<KinematicOrder::Accel>>::encode(data.accel(), message.accel);
      }
    }
  }

  template <typename MSG>
  static void decode(const MSG& message, T& data)
  {
    static_assert(order_depth == KinematicOrder::Pose || order_depth == KinematicOrder::Twist ||
                      order_depth == KinematicOrder::Accel,
                  "FactoryEncoder<KinematicState>::decode() is not yet implemented for this KinematicOrder depth.");
    if constexpr (std::is_base_of_v<msg_stamped, MSG>) {
      decode(message.state, data);
    } else {
      FactoryEncoder<VariableType<KinematicOrder::Pose>>::decode(message.pose, data.pose());
      if constexpr (order_depth >= KinematicOrder::Twist) {
        FactoryEncoder<VariableType<KinematicOrder::Twist>>::decode(message.twist, data.twist());
      }
      if constexpr (order_depth >= KinematicOrder::Accel) {
        FactoryEncoder<VariableType<KinematicOrder::Accel>>::decode(message.accel, data.accel());
      }
    }
  }
};

}  // namespace duatic::data_encoding
