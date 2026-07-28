#pragma once

#include <duatic_geometry_msgs/factory.hpp>

namespace duatic_geometry_msgs
{

// The ROS 2 message types Factory<T> converts T to/from, without and with header.
template <typename T>
using msg_t = typename Factory<T>::msg;

template <typename T>
using msg_stamped_t = typename Factory<T>::msg_stamped;

template <typename T, typename MSG>
void encode(const T& data, MSG& message)
{
  Factory<T>::encode(data, message);
}

template <typename T, typename MSG>
void decode(const MSG& message, T& data)
{
  Factory<T>::decode(message, data);
}

}  // namespace duatic_geometry_msgs
