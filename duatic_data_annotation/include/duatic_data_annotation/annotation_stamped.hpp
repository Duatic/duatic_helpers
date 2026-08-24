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

#include <ostream>
#include <string>
#include <duatic_data_annotation/annotation_timed.hpp>

namespace duatic::data_annotation
{

template <typename DataT, typename TimestampT>
class StampedData : public TimedData<DataT, TimestampT>
{
public:
  using DataType = DataT;
  using TimestampType = TimestampT;
  using Self = StampedData<DataType, TimestampType>;

  inline constexpr StampedData() = default;
  inline explicit constexpr StampedData(const Self& other) = default;
  inline explicit constexpr StampedData(Self&& other) = default;

  inline Self& operator=(const Self& other) = default;
  inline Self& operator=(Self&& other) = default;

  template <typename TimeCtor, typename... DataCtors>
  inline constexpr StampedData(const TimeCtor& time_init, const std::string& frame_init,
                               const DataCtors&... data_init)
    : TimedData<DataType, TimestampType>(time_init, data_init...), frame_(frame_init)
  {
  }

  inline std::string& frame_id()
  {
    return frame_;
  }
  inline const std::string& frame_id() const
  {
    return frame_;
  }

  Self& setFrameNeutral()
  {
    frame_.clear();
    return *this;
  }
  Self& setNeutral()
  {
    TimedData<DataType, TimestampType>::setNeutral();
    return setFrameNeutral();
  }

private:
  std::string frame_;
};

// streaming
template <typename DataT, typename TimeStampT>
inline std::ostream& operator<<(std::ostream& os, const StampedData<DataT, TimeStampT>& stamped)
{
  os << "Stamped:" << std::endl << " - Time: ";
  stamped.stream_time(os);
  os << std::endl << " - Frame: " << stamped.frame_id() << std::endl  // line break
     << " - Data: " << static_cast<const DataT&>(stamped);
  return os;
}

}  // namespace duatic::data_annotation
