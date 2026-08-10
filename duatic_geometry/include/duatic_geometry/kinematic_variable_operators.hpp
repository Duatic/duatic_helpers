#pragma once

#include <ostream>
#include <duatic_geometry/kinematic_variable.hpp>

namespace duatic::geometry
{

// TLHS only needs to be KinematicVariable (not KinematicDiffVariable): this must also cover
// Pose +/- Twist (e.g. goal_ + linear_factor*decay throughout the trajectory headers), and Pose's
// own kinematic_order is KinematicOrder::Pose itself, not > Pose, so it fails KinematicDiffVariable.
// TRHS (the delta being applied) does need to be a genuine diff type, hence KinematicDiffVariable.
template <KinematicVariable TLHS, KinematicDiffVariable TRHS>
  requires(std::same_as<typename TLHS::ScalarType, typename TRHS::ScalarType>) &&
          ((TLHS::kinematic_order == TRHS::kinematic_order) || (TLHS::kinematic_order + 1 == TRHS::kinematic_order))
inline TLHS operator+(const TLHS& lhs, const TRHS& rhs)
{
  TLHS result = lhs;
  result += rhs;
  return result;
}

// Deliberately NOT constrained by KinematicVariable/KinematicDiffVariable (unlike operator+ above):
// KinematicVariable<T>'s own definition is expressed in terms of `const_variable - const_variable`
// (see kinematic_variable.hpp), so a concept-constrained operator- here would make checking this
// very overload's viability re-enter that same concept check ("satisfaction of atomic constraint
// ... depends on itself"). Structural checks only (ScalarType match, adjacent order) sidestep that.
// Also deliberately adjacent-order only (TLHS one order below TRHS, e.g. Pose - Twist): the
// same-order case (e.g. Twist - Twist) has different semantics -- it computes a *diff*, promoting to
// the next order (Twist - Twist -> Accel) -- and is already handled by the dedicated, similarly
// unconstrained same-order operator- in kinematic_variable_3d_eigen.hpp.
template <typename TLHS, typename TRHS>
  requires(std::same_as<typename TLHS::ScalarType, typename TRHS::ScalarType>) &&
          (TLHS::kinematic_order + 1 == TRHS::kinematic_order)
inline TLHS operator-(const TLHS& lhs, const TRHS& rhs)
{
  TLHS result = lhs;
  result -= rhs;
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
