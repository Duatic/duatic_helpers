#pragma once

#include <concepts>
#include <string>
#include <type_traits>

namespace duatic::geometry
{

// Satisfied by TimedData<DataT, TimestampT> (see timed.hpp) and anything else exposing the same
// DataType/TimestampType typenames and time()/data() accessors.
template <typename T>
concept Timed = requires(T& mutable_value, const T& const_value) {
  typename T::DataType;
  typename T::TimestampType;

  { mutable_value.time() } -> std::same_as<typename T::TimestampType&>;
  { const_value.time() } -> std::same_as<const typename T::TimestampType&>;

  { mutable_value.data() } -> std::same_as<typename T::DataType&>;
  { const_value.data() } -> std::same_as<const typename T::DataType&>;
};

// Satisfied by StampedData<DataT, TimestampT> (see stamped.hpp) and anything else additionally
// exposing a frame_id() accessor on top of the Timed interface.
template <typename T>
concept Stamped = Timed<T> && requires(T& mutable_value, const T& const_value) {
  { mutable_value.frame_id() } -> std::same_as<std::string&>;
  { const_value.frame_id() } -> std::same_as<const std::string&>;
};

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

}  // namespace duatic::geometry
