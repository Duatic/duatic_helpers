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
    };  // NOLINT(readability/braces)

// forward declaration for later concretizations
template <typename T>
class FactoryEncoder
{
  static_assert(false, "duatic_geometry_msgs::FactoryEncoder<T> is not defined for this type. T must be a "
                       "duatic::geometry::KinematicVariable or duatic::geometry::KinematicState.");
};

}  // namespace duatic_geometry_msgs

// include template instantiations
#include <duatic_geometry_msgs/encoder_kinematic_state.hpp>
#include <duatic_geometry_msgs/encoder_kinematic_variable.hpp>
