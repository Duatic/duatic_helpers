#pragma once

#include <ostream>
#include <string>
#include <duatic_geometry/annotation_timed.hpp>

namespace duatic::geometry
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

}  // namespace duatic::geometry

// streaming
template <typename DataT, typename TimeStampT>
inline std::ostream& operator<<(std::ostream& os, const duatic::geometry::StampedData<DataT, TimeStampT>& stamped)
{
  os << "Stamped:" << std::endl
     << " - Time: " << stamped.time() << std::endl
     << " - Frame: " << stamped.frame_id() << std::endl
     << " - Data: " << static_cast<const DataT&>(stamped);
  return os;
}
