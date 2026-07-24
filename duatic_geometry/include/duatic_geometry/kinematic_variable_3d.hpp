#pragma once

#include <concepts>
#include <cstdint>
#include <type_traits>

namespace duatic::geometry
{

using KinematicOrderT = unsigned int;
enum class KinematicOrder : KinematicOrderT  // TODO: move out into template instatiation header
{
  Pose = 0,
  Twist = 1,
  Accel = 2,
  Jerk = 3,
  Snap = 4,
};
static_assert(std::is_same_v<std::underlying_type_t<KinematicOrder>, KinematicOrderT>);

// Unconstrained (not KinematicVariable3D-constrained) so it can be used from within
// KinematicVariable3D's own definition without a self-referential constraint.
template <typename DIFF, typename OF>
struct kinematic_diff_of : std::bool_constant<DIFF::KinematicOrder == OF::KinematicOrder + 1>
{
};

template <typename T>
concept KinematicVariable3D = requires(T& variable, const T& const_variable) {
  typename T::ScalarType;

  requires(!std::is_void_v<decltype(variable.linear())>);
  requires(!std::is_void_v<decltype(const_variable.linear())>);
  requires(!std::is_void_v<decltype(variable.angular())>);
  requires(!std::is_void_v<decltype(const_variable.angular())>);

  { variable.setLinearNeutral() } -> std::same_as<T&>;
  { variable.setAngularNeutral() } -> std::same_as<T&>;
  { variable.setNeutral() } -> std::same_as<T&>;

  { T::KinematicOrder } -> std::convertible_to<KinematicOrderT>;
  typename std::integral_constant<KinematicOrderT, T::KinematicOrder>;

  { const_variable - const_variable };
  requires kinematic_diff_of<std::remove_cvref_t<decltype(const_variable - const_variable)>, T>::value;
};

template <typename T>
concept KinematicDiffVariable3D =
    KinematicVariable3D<T> && requires(T& variable, const T& const_variable, typename T::ScalarType scalar) {
      requires(!std::is_void_v<decltype(variable.vector())>);
      requires(!std::is_void_v<decltype(const_variable.vector())>);

      { variable += const_variable } -> std::same_as<T&>;
      { variable -= const_variable } -> std::same_as<T&>;
      { variable *= scalar } -> std::same_as<T&>;
    };

}  // namespace duatic::geometry
