/*
 * Copyright 2026 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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
