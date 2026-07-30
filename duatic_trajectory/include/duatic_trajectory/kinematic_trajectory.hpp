#pragma once

#include <concepts>
#include <memory>
#include <numbers>
#include <utility>

namespace duatic::trajectory
{

template <typename T>
concept KinematicLimits = requires(const T& const_variable) {
  typename T::ScalarType;

  { const_variable.velocity_limit_linear() } -> std::same_as<typename T::ScalarType>;
  { const_variable.velocity_limit_angular() } -> std::same_as<typename T::ScalarType>;
};

/*
 * Plain-data default implementation of KinematicLimits: public members, no validation.
 */
template <typename ScalarT>
struct KinematicLimitsDefault
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

static_assert(KinematicLimits<KinematicLimitsDefault<double>>, "KinematicLimitsDefault must satisfy the "
                                                               "KinematicLimits concept.");

template <typename T>
concept KinematicTrajectory =
    requires(T& variable, const T& const_variable, const typename T::TimestampType timestamp,
             const typename T::ScalarType scalar_value, const typename T::KinematicUpdateState& in_state,
             typename T::KinematicEvalState& out_state) {
      typename T::ScalarType;
      typename T::TimestampType;
      typename T::KinematicUpdateState;
      typename T::TrajectoryUpdateType;
      typename T::KinematicEvalState;
      typename T::KinematicLimitsType;

      requires KinematicLimits<typename T::KinematicLimitsType>;
      requires std::convertible_to<typename T::KinematicLimitsType::ScalarType, typename T::ScalarType>;
      requires std::constructible_from<T, std::shared_ptr<typename T::KinematicLimitsType>>;

      { variable.update(in_state, std::declval<const typename T::TrajectoryUpdateType&>()) };

      { variable.update_neutral(in_state) };

      { const_variable.evaluate(timestamp, out_state) };

      { const_variable.evaluate(timestamp) } -> std::same_as<typename T::KinematicEvalState>;
    };

// trait helpers

template <typename T>
struct is_kinematic_limits : std::bool_constant<KinematicLimits<T>>
{
};

template <typename T>
constexpr bool is_kinematic_limits_v = is_kinematic_limits<T>::value;

template <typename T>
struct is_kinematic_trajectory : std::bool_constant<KinematicTrajectory<T>>
{
};

template <typename T>
constexpr bool is_kinematic_trajectory_v = is_kinematic_trajectory<T>::value;

}  // namespace duatic::trajectory
