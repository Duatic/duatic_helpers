#pragma once

#include <ostream>
#include <duatic_geometry/kinematic_variable_3d.hpp>

namespace duatic::geometry
{

template <KinematicDiffVariable3D T>
inline T operator+(const T& lhs, const T& rhs)
{
  T result = lhs;
  result += rhs;
  return result;
}

template <KinematicDiffVariable3D T>
inline T operator*(const T& lhs, const typename T::ScalarType scalar)
{
  T result = lhs;
  result *= scalar;
  return result;
}

template <KinematicVariable3D T>
inline std::ostream& operator<<(std::ostream& os, const T& variable)
{
  os << "KinematicVariable3D<order=" << T::kinematic_order << ">(linear: " << variable.linear()
     << ", angular: " << variable.angular() << ")";
  return os;
}

}  // namespace duatic::geometry
