#pragma once

#include <iostream>
namespace duatic::geometry
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

  Self& setTimeNeutral()
  {
    time_ = TimestampType();
    return *this;
  }
  Self& setDataNeutral()
  {
    DataType::setNeutral();
    return *this;
  }
  Self& setNeutral()
  {
    return setTimeNeutral().setDataNeutral();
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

}  // namespace duatic::geometry
