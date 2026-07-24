#pragma once

#include <iostream>
namespace duatic::geometry
{

template <typename DataT, typename TimestampT>
class Timed : public DataT
{
public:
  using DataType = DataT;
  using TimestampType = TimestampT;
  using Self = Timed<DataType, TimestampType>;

  inline constexpr Timed() = default;
  inline explicit constexpr Timed(const Self& other) = default;
  inline explicit constexpr Timed(Self&& other) = default;

  inline Self& operator=(const Self& other) = default;
  inline Self& operator=(Self&& other) = default;

  template <typename TimeCtor, typename... DataCtors>
  inline constexpr Timed(const TimeCtor& time_init, const DataCtors&... data_init)
    : DataType(data_init...), time(time_init)
  {
  }

  inline TimestampType& timestamp()
  {
    return time;
  }
  inline const TimestampType& timestamp() const
  {
    return time;
  }

  Self& setTimeNeutral()
  {
    time = TimestampType();
    return *this;
  }
  Self& setNeutral()
  {
    DataType::setNeutral();
    return setTimeNeutral();
  }

private:
  TimestampType time;
};

}  // namespace duatic::geometry

// streaming
template <typename DataT, typename TimestampT>
inline std::ostream& operator<<(std::ostream& os, const duatic::geometry::Timed<DataT, TimestampT>& stamped)
{
  os << "Timed data:" << std::endl
     << " - Time: " << stamped.timestamp() << std::endl
     << " - Data: " << static_cast<const DataT&>(stamped);
  return os;
}
