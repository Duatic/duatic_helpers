#pragma once

#include <type_traits>
#include <utility>
#include <duatic_geometry/kinematic_variable_3d.hpp>

namespace duatic::geometry
{

// value traits

template <KinematicVariable3D T, KinematicVariable3D U>
struct is_same_kinematic_order : std::bool_constant<T::KinematicOrder == U::KinematicOrder>
{
};

template <KinematicVariable3D T, KinematicVariable3D U>
constexpr bool is_same_kinematic_order_v = is_same_kinematic_order<T, U>::value;

template <KinematicVariable3D DIFF, KinematicVariable3D OF>
struct is_kinematic_diff_of : kinematic_diff_of<DIFF, OF>
{
};

template <KinematicVariable3D DIFF, KinematicVariable3D OF>
constexpr bool is_kinematic_diff_of_v = is_kinematic_diff_of<DIFF, OF>::value;

template <typename T>
constexpr bool is_kinematic_variable_3d_helper()
{
  if constexpr (!KinematicVariable3D<T>) {
    return false;
  } else if constexpr (T::KinematicOrder > static_cast<KinematicOrderT>(KinematicOrder::Pose)) {
    return KinematicDiffVariable3D<T>;
  } else {
    return true;
  }
}

template <typename T>
struct is_kinematic_variable_3d : std::bool_constant<is_kinematic_variable_3d_helper<T>()>
{
};

template <typename T>
constexpr bool is_kinematic_variable_3d_v = is_kinematic_variable_3d<T>::value;

// type traits

template <KinematicVariable3D T>
struct kinematic_linear_type
{
  using type = std::remove_cvref_t<decltype(std::declval<T&>().linear())>;
};

template <KinematicVariable3D T>
using kinematic_linear_type_t = typename kinematic_linear_type<T>::type;

template <KinematicVariable3D T>
struct kinematic_angular_type
{
  using type = std::remove_cvref_t<decltype(std::declval<T&>().angular())>;
};

template <KinematicVariable3D T>
using kinematic_angular_type_t = typename kinematic_angular_type<T>::type;

template <KinematicDiffVariable3D T>
struct kinematic_vector_type
{
  using type = std::remove_cvref_t<decltype(std::declval<T&>().vector())>;
};

template <KinematicDiffVariable3D T>
using kinematic_vector_type_t = typename kinematic_vector_type<T>::type;

template <KinematicVariable3D T>
struct kinematic_diff_type
{
  using type = std::remove_cvref_t<decltype(std::declval<T&>() - std::declval<T&>())>;
  static_assert(is_kinematic_diff_of_v<type, T>);
};

template <KinematicVariable3D T>
using kinematic_diff_type_t = typename kinematic_diff_type<T>::type;

}  // namespace duatic::geometry
