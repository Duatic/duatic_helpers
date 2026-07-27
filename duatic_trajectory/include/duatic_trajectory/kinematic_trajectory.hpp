#pragma once

#include <concepts>
#include <utility>

#include <duatic_geometry/kinematic_order.hpp>
#include <duatic_geometry/timed.hpp>

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

      { const_variable.evaluate(timestamp, out_state) };

      { const_variable.evaluate(timestamp) } -> std::same_as<typename T::KinematicEvalState>;
    };

}  // namespace duatic::trajectory
