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

#include <concepts>
#include <string>
#include <type_traits>

namespace duatic::data_annotation
{

// Satisfied by TimedData<DataT, TimestampT> (see annotation_timed.hpp) and anything else exposing the same
// DataType/TimestampType typenames and time()/data() accessors.
template <typename T>
concept Timed = requires(T& mutable_value, const T& const_value) {
  typename T::DataType;
  typename T::TimestampType;

  { mutable_value.time() } -> std::same_as<typename T::TimestampType&>;
  { const_value.time() } -> std::same_as<const typename T::TimestampType&>;

  { mutable_value.data() } -> std::same_as<typename T::DataType&>;
  { const_value.data() } -> std::same_as<const typename T::DataType&>;
};  // NOLINT(readability/braces)

// Satisfied by StampedData<DataT, TimestampT> (see annotation_stamped.hpp) and anything else additionally
// exposing a frame_id() accessor on top of the Timed interface.
template <typename T>
concept Stamped = Timed<T> && requires(T& mutable_value, const T& const_value) {
  { mutable_value.frame_id() } -> std::same_as<std::string&>;
  { const_value.frame_id() } -> std::same_as<const std::string&>;
};  // NOLINT(readability/braces)

// trait helpers

template <typename T>
struct is_timed : std::bool_constant<Timed<T>>
{
};

template <typename T>
constexpr bool is_timed_v = is_timed<T>::value;

template <typename T>
struct is_stamped : std::bool_constant<Stamped<T>>
{
};

template <typename T>
constexpr bool is_stamped_v = is_stamped<T>::value;

}  // namespace duatic::data_annotation
