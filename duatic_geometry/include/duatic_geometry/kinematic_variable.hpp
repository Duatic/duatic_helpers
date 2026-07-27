#pragma once

#include <concepts>
#include <cstdint>
#include <type_traits>

#include <duatic_geometry/kinematic_order.hpp>

namespace duatic::geometry
{

// Unconstrained (not KinematicVariable-constrained) so it can be used from within
// KinematicVariable's own definition without a self-referential constraint.
template <typename DIFF, typename OF>
struct kinematic_diff_of : std::bool_constant<DIFF::kinematic_order == OF::kinematic_order + 1>
{
  static_assert(std::is_same_v<std::remove_cvref_t<decltype(DIFF::kinematic_order)>, KinematicOrder>);
  static_assert(std::is_same_v<std::remove_cvref_t<decltype(OF::kinematic_order)>, KinematicOrder>);
};

template <typename T>
concept KinematicVariable = requires(T& variable, const T& const_variable) {
  typename T::ScalarType;

  requires(!std::is_void_v<decltype(variable.linear())>);
  requires(!std::is_void_v<decltype(const_variable.linear())>);
  requires(!std::is_void_v<decltype(variable.angular())>);
  requires(!std::is_void_v<decltype(const_variable.angular())>);

  { variable.setLinearNeutral() } -> std::same_as<T&>;
  { variable.setAngularNeutral() } -> std::same_as<T&>;
  { variable.setNeutral() } -> std::same_as<T&>;

  { T::kinematic_order } -> std::convertible_to<KinematicOrder>;

  { const_variable - const_variable };
  requires kinematic_diff_of<std::remove_cvref_t<decltype(const_variable - const_variable)>, T>::value;
};

template <typename T>
concept KinematicDiffVariable =
    KinematicVariable<T> && requires(T& variable, const T& const_variable, typename T::ScalarType scalar) {
      requires(!std::is_void_v<decltype(variable.vector())>);
      requires(!std::is_void_v<decltype(const_variable.vector())>);

      { variable += const_variable } -> std::same_as<T&>;
      { variable -= const_variable } -> std::same_as<T&>;
      { variable *= scalar } -> std::same_as<T&>;
    };

}  // namespace duatic::geometry
