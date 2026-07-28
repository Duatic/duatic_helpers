#pragma once

#include <type_traits>
#include <utility>
#include <duatic_geometry/kinematic_variable.hpp>

namespace duatic::geometry
{

// value traits

template <KinematicVariable T, KinematicVariable U>
struct is_same_kinematic_order : std::bool_constant<T::kinematic_order == U::kinematic_order>
{
};

template <KinematicVariable T, KinematicVariable U>
constexpr bool is_same_kinematic_order_v = is_same_kinematic_order<T, U>::value;

template <KinematicVariable DIFF, KinematicVariable OF>
struct is_kinematic_diff_of : std::bool_constant<kinematic_diff_of_helper<DIFF, OF>()>
{
};

template <KinematicVariable DIFF, KinematicVariable OF>
constexpr bool is_kinematic_diff_of_v = is_kinematic_diff_of<DIFF, OF>::value;

// type traits

template <KinematicVariable T>
struct kinematic_linear_type
{
  using type = std::remove_cvref_t<decltype(std::declval<T&>().linear())>;
};

template <KinematicVariable T>
using kinematic_linear_type_t = typename kinematic_linear_type<T>::type;

template <KinematicVariable T>
struct kinematic_angular_type
{
  using type = std::remove_cvref_t<decltype(std::declval<T&>().angular())>;
};

template <KinematicVariable T>
using kinematic_angular_type_t = typename kinematic_angular_type<T>::type;

template <KinematicDiffVariable T>
struct kinematic_vector_type
{
  using type = std::remove_cvref_t<decltype(std::declval<T&>().vector())>;
};

template <KinematicDiffVariable T>
using kinematic_vector_type_t = typename kinematic_vector_type<T>::type;

template <KinematicVariable T>
struct kinematic_diff_type
{
  using type = std::remove_cvref_t<decltype(std::declval<T&>() - std::declval<T&>())>;
  static_assert(is_kinematic_diff_of_v<type, T>);
};

template <KinematicVariable T>
using kinematic_diff_type_t = typename kinematic_diff_type<T>::type;

}  // namespace duatic::geometry
