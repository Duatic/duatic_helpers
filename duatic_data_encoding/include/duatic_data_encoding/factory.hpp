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
#include <duatic_data_annotation/annotation.hpp>
#include <duatic_data_encoding/encoder.hpp>

using duatic::data_annotation::is_stamped_v;
using duatic::data_annotation::is_timed_v;

namespace duatic::data_encoding
{

// std::conditional_t substitutes both branches unconditionally, so it can't be used
// directly with typename U::DataType (U may not have one); dispatch via specialization
// instead. Must live at namespace scope, not nested in Factory<T>: specializations of
// FactoryEncoder added by other packages (e.g. KinematicVariableMsgTypeHelper in
// duatic_geometry_encoder) rely on the same namespace-scope pattern.
template <typename U, bool is_annotated>
struct FactoryDataTypeHelper
{
  using type = U;
};
template <typename U>
struct FactoryDataTypeHelper<U, true>
{
  using type = typename U::DataType;
};

// ---------------------------------------------------------------------------
// Factory
//
// Converts between a data type T and its ROS 2 message counterparts: `msg`
// (no header) and `msg_stamped` (with header), via a FactoryEncoder<T>
// specialization. T may be a plain type or that type wrapped in
// TimedData<> / StampedData<> -- whether T satisfies the Timed/Stamped
// concepts is evaluated automatically and, if so, decode()/encode()
// additionally transfer the message header's stamp (and frame_id, if
// stamped).
// ---------------------------------------------------------------------------
template <typename T, template <typename> typename EncoderT = FactoryEncoder>
class Factory
{
public:
  static constexpr bool is_timed = is_timed_v<T>;
  static constexpr bool is_stamped = is_stamped_v<T>;

  static constexpr bool is_annotated = is_timed || is_stamped;

private:
  using DataType = typename FactoryDataTypeHelper<T, is_annotated>::type;

  using EncoderImpl = EncoderT<DataType>;
  static_assert(Encoder<EncoderImpl>);

public:
  using msg_stamped = typename EncoderImpl::msg_stamped;
  using msg = std::conditional_t<is_annotated, msg_stamped, typename EncoderImpl::msg>;

  template <typename MSG>
    requires std::is_base_of_v<msg, MSG> || std::is_base_of_v<msg_stamped, MSG>
  static void encode(const T& data, MSG& msg)
  {
    // header handling
    if constexpr (std::is_base_of_v<msg_stamped, MSG>) {
      if constexpr (is_timed) {
        msg.header.stamp = data.time();
      }
      if constexpr (is_stamped) {
        static_assert(is_timed);
        msg.header.frame_id = data.frame_id();
      }
    }
    // data handling
    if constexpr (is_annotated) {
      EncoderImpl::encode(data.data(), msg);
    } else {
      EncoderImpl::encode(data, msg);
    }
  }

  template <typename MSG>
    requires std::is_base_of_v<msg, MSG> || std::is_base_of_v<msg_stamped, MSG>
  static void decode(const MSG& msg, T& data)
  {
    // header handling
    if constexpr (std::is_base_of_v<msg_stamped, MSG>) {
      if constexpr (is_timed) {
        data.time() = msg.header.stamp;
      }
      if constexpr (is_stamped) {
        static_assert(is_timed);
        data.frame_id() = msg.header.frame_id;
      }
    }
    // data handling
    if constexpr (is_annotated) {
      EncoderImpl::decode(msg, data.data());
    } else {
      EncoderImpl::decode(msg, data);
    }
  }
};

}  // namespace duatic::data_encoding
