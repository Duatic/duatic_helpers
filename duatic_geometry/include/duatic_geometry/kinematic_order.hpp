#pragma once

#include <cstdint>
#include <ostream>
#include <string>
#include <type_traits>
#include <assert.h>

namespace duatic::geometry
{

enum class KinematicOrder : unsigned int
{
  Pose = 0,
  Twist = 1,
  Accel = 2,
  Jerk = 3,
  Snap = 4,
  FifthDerivative = 5,
  _KINEMATIC_END_OF_ORDER
};
static_assert(std::is_integral_v<std::underlying_type_t<KinematicOrder>>);
static_assert(std::is_unsigned_v<std::underlying_type_t<KinematicOrder>>);

// operators

inline constexpr std::underlying_type_t<KinematicOrder> to_number(const KinematicOrder order)
{
  return static_cast<std::underlying_type_t<KinematicOrder>>(order);
}

static_assert(to_number(KinematicOrder::Pose) == 0);

inline std::string to_string(const KinematicOrder order)
{
  std::string s = std::to_string(to_number(order));
  switch (order) {
    case KinematicOrder::Pose:
      s += " (Pose)";
      break;
    case KinematicOrder::Twist:
      s += " (Twist)";
      break;
    case KinematicOrder::Accel:
      s += " (Accel)";
      break;
    case KinematicOrder::Jerk:
      s += " (Jerk)";
      break;
    case KinematicOrder::Snap:
      s += " (Snap)";
      break;
    default:
      break;
  }
  return s;
}

inline std::ostream& operator<<(std::ostream& os, const KinematicOrder order)
{
  return os << to_string(order);
}

// KinematicOrder is a scoped enum, so the built-in ==, !=, <, <=, >, >= operators already
// compare it directly (no cast to the underlying type required); only arithmetic (which enums
// don't support natively) needs one, funneled through to_number() below.

template <typename OffsetT>
  requires std::is_convertible_v<const OffsetT, const std::underlying_type_t<KinematicOrder>>
inline constexpr KinematicOrder operator+(const KinematicOrder order, const OffsetT offset)
{
  using underlying_t = std::underlying_type_t<KinematicOrder>;
  const underlying_t order_sum = to_number(order) + static_cast<underlying_t>(offset);
  assert((order_sum < to_number(KinematicOrder::_KINEMATIC_END_OF_ORDER))  // line break
         && "KinematicOrder out of defined range. If you really need this, extend this enum definition.");
  return static_cast<KinematicOrder>(order_sum);
}

template <typename OffsetT>
  requires std::is_convertible_v<const OffsetT, const std::underlying_type_t<KinematicOrder>>
inline constexpr KinematicOrder& operator+=(KinematicOrder& order, const OffsetT offset)
{
  order = order + offset;
  return order;
}

inline constexpr KinematicOrder& operator++(KinematicOrder& order)
{
  order += 1;
  return order;
}

inline constexpr KinematicOrder operator++(KinematicOrder& order, int)
{
  const KinematicOrder previous = order;
  ++order;
  return previous;
}

}  // namespace duatic::geometry
