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

#include <iostream>
#include <variant>

#include <duatic_data_annotation/annotation.hpp>

namespace duatic::data_annotation
{

template <typename DataT, typename TimestampT>
class TimedData : public DataT
{
public:
  using DataType = DataT;
  using TimestampType = TimestampT;
  using Self = TimedData<DataType, TimestampType>;

  inline constexpr TimedData() = default;
  inline explicit constexpr TimedData(const Self& other) = default;
  inline explicit constexpr TimedData(Self&& other) = default;

  inline Self& operator=(const Self& other) = default;
  inline Self& operator=(Self&& other) = default;

  template <typename TimeCtor, typename... DataCtors>
  inline constexpr TimedData(const TimeCtor& time_init, const DataCtors&... data_init)
    : DataType(data_init...), time_(time_init)
  {
  }

  inline TimestampType& time()
  {
    return time_;
  }
  inline const TimestampType& time() const
  {
    return time_;
  }

  inline DataType& data()
  {
    return *this;
  }
  inline const DataType& data() const
  {
    return *this;
  }

  // Not every TimestampType is itself streamable (e.g. rclcpp::Time has no operator<<); fall back to
  // printing its .seconds() (available on rclcpp::Time/rclcpp::Duration-like types) instead of failing
  // to compile entirely. Shared by TimedData's and StampedData's operator<<.
  static std::ostream& stream_time(std::ostream& os, const TimestampType& time)
  {
    if constexpr (requires { os << time; }) {
      os << time;
    } else if constexpr (requires { time.seconds(); }) {
      os << time.seconds() << " s";
    } else {
      os << "<unprintable timestamp>";
    }
    return os;
  }

  inline std::ostream& stream_time(std::ostream& os) const
  {
    return Self::stream_time(os, time());
  }

private:
  TimestampType time_;
};

// streaming
template <typename DataT, typename TimestampT>
inline std::ostream& operator<<(std::ostream& os, const TimedData<DataT, TimestampT>& stamped)
{
  os << "Timed data:" << std::endl << " - Time: ";
  stamped.stream_time(os);
  os << std::endl << " - Data: " << static_cast<const DataT&>(stamped);
  return os;
}

// Compile-time verification: TimedData<DataT, TimestampT> must satisfy the Timed concept it exists to implement.
static_assert(Timed<TimedData<std::monostate, double>>,
              "TimedData<DataT, TimestampT> does not satisfy the Timed concept");

}  // namespace duatic::data_annotation
