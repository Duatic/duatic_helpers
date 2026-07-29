#pragma once

#include <concepts>
#include <utility>

namespace duatic::trajectory
{

template <typename T>
concept KinematicTrajectory =
    requires(T& variable, const T& const_variable, const typename T::TimestampType timestamp,
             const typename T::KinematicUpdateState& in_state, typename T::KinematicEvalState& out_state) {
      typename T::ScalarType;
      typename T::TimestampType;
      typename T::KinematicUpdateState;
      typename T::TrajectoryUpdateType;
      typename T::KinematicEvalState;

      { variable.update(in_state, std::declval<const typename T::TrajectoryUpdateType&>()) };

      { variable.update_neutral(in_state) };

      { const_variable.evaluate(timestamp, out_state) };

      { const_variable.evaluate(timestamp) } -> std::same_as<typename T::KinematicEvalState>;
    };

// trait helpers

template <typename T>
struct is_kinematic_trajectory : std::bool_constant<KinematicTrajectory<T>>
{
};

template <typename T>
constexpr bool is_kinematic_trajectory_v = is_kinematic_trajectory<T>::value;

}  // namespace duatic::trajectory
