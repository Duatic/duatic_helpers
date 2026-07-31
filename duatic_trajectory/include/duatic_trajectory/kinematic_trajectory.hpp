#pragma once

#include <concepts>
#include <memory>
#include <numbers>
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
};

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

  ScalarType v_max_lin_{ 1.0 };
  ScalarType v_max_ang_{ 2.0 * std::numbers::pi_v<ScalarType> };

  inline ScalarType velocity_limit_linear() const
  {
    return v_max_lin_;
  }

  inline ScalarType velocity_limit_angular() const
  {
    return v_max_ang_;
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
};

static_assert(KinematicTrajectorySettings<KinematicTrajectorySettingsDefault<double>>, "KinematicTrajectorySettingsDefa"
                                                                                       "ult must satisfy the "
                                                                                       "KinematicTrajectorySettings "
                                                                                       "concept.");

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

      // calculate an entire new trajectory starting from the given state
      { variable.calculate(in_update_state, in_description) };

      { variable.calculate_neutral(in_update_state) };

      // update the existing trajectory starting from the given timestamp
      { variable.update(in_timestamp, in_description) };

      { variable.update_neutral(in_timestamp) };

      // update from an existing trajectory starting from the given timestamp
      { variable.update_from(const_variable, in_timestamp, in_description) };

      { const_variable.template evaluate<geometry::KinematicOrder::Pose>(in_timestamp, out_state) };

      {
        const_variable.template evaluate<geometry::KinematicOrder::Pose>(in_timestamp)
      } -> std::same_as<typename T::template KinematicState<geometry::KinematicOrder::Pose>>;
    };

template <typename T>
struct is_kinematic_trajectory : std::bool_constant<KinematicTrajectory<T>>
{
};

template <typename T>
constexpr bool is_kinematic_trajectory_v = is_kinematic_trajectory<T>::value;

}  // namespace duatic::trajectory
