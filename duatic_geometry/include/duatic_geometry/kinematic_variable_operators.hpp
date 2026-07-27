#pragma once

#include <ostream>
#include <duatic_geometry/kinematic_variable.hpp>

namespace duatic::geometry
{

template <KinematicDiffVariable T>
inline T operator+(const T& lhs, const T& rhs)
{
  T result = lhs;
  result += rhs;
  return result;
}

template <KinematicDiffVariable T>
inline T operator*(const T& lhs, const typename T::ScalarType scalar)
{
  T result = lhs;
  result *= scalar;
  return result;
}

template <KinematicVariable T>
inline std::ostream& operator<<(std::ostream& os, const T& variable)
{
  os << "KinematicVariable<order=" << T::kinematic_order << ">(linear: " << variable.linear()
     << ", angular: " << variable.angular() << ")";
  return os;
}

}  // namespace duatic::geometry
