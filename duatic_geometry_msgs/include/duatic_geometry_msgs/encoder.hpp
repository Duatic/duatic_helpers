#pragma once

#include <duatic_geometry/annotation.hpp>

namespace duatic_geometry_msgs
{

// An Encoder must expose the `msg`/`msg_stamped` ROS 2
// message types (without/with header) that the type's fields are encoded into / decoded
// from, thus matching encode()/decode() static functions accepting either message
template <typename T>
concept Encoder =
    !duatic::geometry::is_timed_v<typename T::DataType> && !duatic::geometry::is_stamped_v<typename T::DataType> &&
    requires(const T::DataType& const_data, T::DataType& data, typename T::msg& message,
             const typename T::msg& const_message, typename T::msg_stamped& message_stamped,
             const typename T::msg_stamped& const_message_stamped) {
      typename T::DataType;
      typename T::msg;
      typename T::msg_stamped;

      T::encode(const_data, message);
      T::encode(const_data, message_stamped);
      T::decode(const_message, data);
      T::decode(const_message_stamped, data);
    };

// forward declaration for later concretizations
template <typename T>
class FactoryEncoder
{
  static_assert(false, "duatic_geometry_msgs::FactoryEncoder<T> is not defined for this type. T must be a "
                       "duatic::geometry::KinematicVariable or duatic::geometry::KinematicState.");
};

}  // namespace duatic_geometry_msgs

// include template instantiations
#include <duatic_geometry_msgs/encoder_kinematic_variable.hpp>
