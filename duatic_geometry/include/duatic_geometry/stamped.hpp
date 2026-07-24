#pragma once

#include <ostream>
#include <string>
#include <duatic_geometry/timed.hpp>

namespace duatic::geometry
{

template <typename DataT, typename TimestampT>
class Stamped : public Timed<DataT, TimestampT>
{
public:
  using DataType = DataT;
  using TimestampType = TimestampT;
  using Self = Stamped<DataType, TimestampType>;

  inline constexpr Stamped() = default;
  inline explicit constexpr Stamped(const Self& other) = default;
  inline explicit constexpr Stamped(Self&& other) = default;

  inline Self& operator=(const Self& other) = default;
  inline Self& operator=(Self&& other) = default;

  template <typename TimeCtor, typename... DataCtors>
  inline constexpr Stamped(const TimeCtor& time_init, const std::string& frame_init, const DataCtors&... data_init)
    : Timed<DataType, TimestampType>(time_init, data_init...), frame_(frame_init)
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
    Timed<DataType, TimestampType>::setNeutral();
    return setFrameNeutral();
  }

private:
  std::string frame_;
};

}  // namespace duatic::geometry

// streaming
template <typename DataT, typename TimeStampT>
inline std::ostream& operator<<(std::ostream& os, const duatic::geometry::Stamped<DataT, TimeStampT>& stamped)
{
  os << "Stamped:" << std::endl
     << " - Time: " << stamped.timestamp() << std::endl
     << " - Frame: " << stamped.frame_id() << std::endl
     << " - Data: " << static_cast<const DataT&>(stamped);
  return os;
}
