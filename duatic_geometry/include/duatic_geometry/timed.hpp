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

  Self& setTimeNeutral()
  {
    time_ = TimestampType();
    return *this;
  }
  Self& setNeutral()
  {
    DataType::setNeutral();
    return setTimeNeutral();
  }

private:
  TimestampType time_;
};

}  // namespace duatic::geometry

// streaming
template <typename DataT, typename TimestampT>
inline std::ostream& operator<<(std::ostream& os, const duatic::geometry::Timed<DataT, TimestampT>& stamped)
{
  os << "Timed data:" << std::endl
     << " - Time: " << stamped.time() << std::endl
     << " - Data: " << static_cast<const DataT&>(stamped);
  return os;
}
