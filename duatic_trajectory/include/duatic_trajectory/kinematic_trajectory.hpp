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

#include <concepts>
#include <numbers>

#include <memory>
#include <utility>

#include <duatic_geometry/annotation.hpp>
#include <duatic_geometry/kinematic_order.hpp>
#include <duatic_geometry/kinematic_state.hpp>

namespace duatic::trajectory
{

template <typename T>
concept KinematicTrajectorySettings = requires(const T& const_variable) {
  typename T::ScalarType;

  { const_variable.velocity_limit_linear() } -> std::same_as<typename T::ScalarType>;
  { const_variable.velocity_limit_angular() } -> std::same_as<typename T::ScalarType>;

  { const_variable.acceleration_limit_linear() } -> std::same_as<typename T::ScalarType>;
  { const_variable.acceleration_limit_angular() } -> std::same_as<typename T::ScalarType>;
};  // NOLINT(readability/braces)

// trait helpers

template <typename T>
struct is_kinematic_trajectory_settings : std::bool_constant<KinematicTrajectorySettings<T>>
{
};

template <typename T>
constexpr bool is_kinematic_trajectory_settings_v = is_kinematic_trajectory_settings<T>::value;

/*
 * Plain-data default implementation of KinematicTrajectorySettings: public members, no validation.
 */
template <typename ScalarT>
struct KinematicTrajectorySettingsDefault
{
  using ScalarType = ScalarT;

  ScalarType v_max_lin_{ 0.1 };                                   // default linear speed limit is 0.1 m/s
  ScalarType v_max_ang_{ 2.0 * std::numbers::pi_v<ScalarType> };  // default angular speed limit is 1 rev/s

  ScalarType a_max_lin_{ v_max_lin_ / 0.1 };  // default linear acceleration limit is max speed / 0.1s
  ScalarType a_max_ang_{ v_max_ang_ / 0.1 };  // default angular acceleration limit is max speed / 0.1s

  inline ScalarType velocity_limit_linear() const
  {
    return v_max_lin_;
  }

  inline ScalarType velocity_limit_angular() const
  {
    return v_max_ang_;
  }

  inline ScalarType acceleration_limit_linear() const
  {
    return a_max_lin_;
  }

  inline ScalarType acceleration_limit_angular() const
  {
    return a_max_ang_;
  }

  /*
   * Sets the maximum linear and angular speed the trajectory should approximately respect
   * (see the "Approximate V-Limit" note above). Returns false (without changing any state)
   * if either limit is negative.
   */
  inline bool set_velocity_limits(const ScalarType v_max_lin, const ScalarType v_max_ang)
  {
    if ((v_max_lin < static_cast<ScalarType>(0)) || (v_max_ang < static_cast<ScalarType>(0))) {
      return false;
    }
    v_max_lin_ = v_max_lin;
    v_max_ang_ = v_max_ang;
    return true;
  }

  /*
   * Sets the maximum linear and angular acceleration the trajectory should approximately respect.
   * Returns false (without changing any state) if either limit is negative.
   */
  inline bool set_acceleration_limits(const ScalarType a_max_lin, const ScalarType a_max_ang)
  {
    if ((a_max_lin < static_cast<ScalarType>(0)) || (a_max_ang < static_cast<ScalarType>(0))) {
      return false;
    }
    a_max_lin_ = a_max_lin;
    a_max_ang_ = a_max_ang;
    return true;
  }
};

static_assert(KinematicTrajectorySettings<KinematicTrajectorySettingsDefault<double>>,  //
              "KinematicTrajectorySettingsDefault must satisfy the KinematicTrajectorySettings concept.");

template <typename T>
concept KinematicTrajectory =
    requires(T& variable, const T& const_variable, const typename T::UpdateStateType& in_update_state,
             const typename T::UpdateStateType::TimestampType in_timestamp,
             typename T::template KinematicState<geometry::KinematicOrder::Pose>& out_state,
             const typename T::TrajectoryDescriptionType& in_description) {
      typename T::ScalarType;
      typename T::KinematicTrajectorySettingsType;
      typename T::TrajectoryDescriptionType;
      typename T::UpdateStateType;
      typename T::template KinematicState<geometry::KinematicOrder::Pose>;

      requires is_kinematic_trajectory_settings_v<typename T::KinematicTrajectorySettingsType>;
      requires std::convertible_to<typename T::KinematicTrajectorySettingsType::ScalarType, typename T::ScalarType>;
      requires std::constructible_from<T, std::shared_ptr<typename T::KinematicTrajectorySettingsType>>;
      requires geometry::is_timed_v<typename T::UpdateStateType>;
      requires geometry::is_kinematic_state_v<typename T::UpdateStateType::DataType>;

      // the continuity order this trajectory guarantees at replan boundaries (e.g. Twist == C1);
      // std::convertible_to (rather than std::same_as) because decltype((T::continuity_order)) is
      // "const geometry::KinematicOrder&" for a static constexpr data member, not the bare enum type
      { T::continuity_order } -> std::convertible_to<geometry::KinematicOrder>;

      // calculate an entire new trajectory starting from the given state
      { variable.calculate(in_update_state, in_description) };  // NOLINT(readability/braces)

      { variable.calculate_neutral(in_update_state) };  // NOLINT(readability/braces)

      // update the existing trajectory starting from the given timestamp
      { variable.update(in_timestamp, in_description) };  // NOLINT(readability/braces)

      { variable.update_neutral(in_timestamp) };  // NOLINT(readability/braces)

      // update from an existing trajectory starting from the given timestamp
      { variable.update_from(const_variable, in_timestamp, in_description) };  // NOLINT(readability/braces)

      {
        const_variable.template evaluate<geometry::KinematicOrder::Pose>(in_timestamp, out_state)
      };  // NOLINT(readability/braces)

      {
        const_variable.template evaluate<geometry::KinematicOrder::Pose>(in_timestamp)
      } -> std::same_as<typename T::template KinematicState<geometry::KinematicOrder::Pose>>;
    };  // NOLINT(readability/braces)

template <typename T>
struct is_kinematic_trajectory : std::bool_constant<KinematicTrajectory<T>>
{
};

template <typename T>
constexpr bool is_kinematic_trajectory_v = is_kinematic_trajectory<T>::value;

}  // namespace duatic::trajectory
