#pragma once

#include <type_traits>

#include <duatic_geometry/kinematic_order.hpp>
#include <duatic_geometry/kinematic_state.hpp>

#include <duatic_geometry_msgs/encoder.hpp>
#include <duatic_geometry_msgs/encoder_kinematic_variable.hpp>

#include <duatic_geometry_msgs/msg/state_accel.hpp>
#include <duatic_geometry_msgs/msg/state_accel_stamped.hpp>
#include <duatic_geometry_msgs/msg/state_pose.hpp>
#include <duatic_geometry_msgs/msg/state_pose_stamped.hpp>
#include <duatic_geometry_msgs/msg/state_twist.hpp>
#include <duatic_geometry_msgs/msg/state_twist_stamped.hpp>

using namespace duatic::geometry;

namespace duatic_geometry_msgs
{

template <KinematicOrder OrderDepth>
struct KinematicStateMsgTypeHelper
{
  static_assert(OrderDepth == KinematicOrder::Pose ||  // line break
                    OrderDepth == KinematicOrder::Twist || OrderDepth == KinematicOrder::Accel,
                "No ROS 2 message mapping is defined for this KinematicState order depth: only Pose, Twist "
                "and Accel depths are supported (StatePose/StateTwist/StateAccel).");
};
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

// template concretization for KinematicState: only order depths with a standard
// mapping are supported (Pose, Twist, Accel), matching the KinematicVariable orders
// that themselves have a ROS 2 message counterpart.
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

}  // namespace duatic_geometry_msgs
