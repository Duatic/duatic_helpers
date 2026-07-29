#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Geometry>

#include <duatic_geometry/geometry.hpp>
#include <duatic_trajectory/trajectory.hpp>

namespace duatic::trajectory
{

// Exponential approach towards a goal:
//
//      x(t) = goal + (A + B * t) * e^(-omega * t)
// ->  x'(t) = (B - omega * (A + B * t)) * e^(-omega * t)
// -> x''(t) = omega * (omega * (A + B * t) - 2*B) * e^(-omega * t)
//
//   A = x0 - goal          (initial offset from the goal)
//   B = v0 + omega * A     (initial velocity, corrected for the offset)
//
// omega is the tracking the convergence speed: higher values converge faster.
//
// Note that the entire non-const calculation can be done in the goal's diff-type and be added to the goal.
// The diff-evlaluation converges toward numerically stable zero
template <typename ScalarT, geometry::KinematicOrder EvalOrderDepth, typename TimestampT,
          template <typename, geometry::KinematicOrder> typename KinematicVariableT>
class KinematicTrajectoryExponentialApproach
{
public:
  using ScalarType = ScalarT;
  static constexpr geometry::KinematicOrder eval_order_depth = EvalOrderDepth;
  using TimestampType = TimestampT;

  using Self = KinematicTrajectoryExponentialApproach<ScalarType, eval_order_depth, TimestampType, KinematicVariableT>;

  template <geometry::KinematicOrder Order>
  using KinematicVariableType = KinematicVariableT<ScalarType, Order>;
  template <geometry::KinematicOrder OrderDepth>
  using KinematicStateType = geometry::KinematicState<ScalarType, OrderDepth, KinematicVariableT>;

  using PoseType = KinematicVariableType<geometry::KinematicOrder::Pose>;
  using TwistType = KinematicVariableType<geometry::KinematicOrder::Twist>;
  using TwistStateType = KinematicStateType<geometry::KinematicOrder::Twist>;

  using KinematicUpdateState = geometry::TimedData<TwistStateType, TimestampType>;
  using KinematicEvalState = KinematicStateType<Self::eval_order_depth>;

  using TrajectoryUpdateType = PoseType;

  inline KinematicTrajectoryExponentialApproach() = default;

  inline void update(const KinematicUpdateState& in_state, const PoseType& update_pose)
  {
    omega_ = 1.0;  // TODO: calculate from limits

    start_time_ = in_state.time();
    goal_ = update_pose;

    A_ = in_state.pose() - goal_;
    B_ = in_state.twist() + (A_ * omega_);
  }

  inline void update_neutral(const KinematicUpdateState& in_state)
  {
    update(in_state, in_state.data().pose());
  }

  inline void evaluate(const TimestampType& time, KinematicEvalState& out_state) const
  {
    const ScalarType t = (time - start_time_).seconds();
    const ScalarType decay = std::exp(-omega_ * t);
    const TwistType linear_factor = A_ + (B_ * t);

    out_state.pose() = goal_ + (linear_factor * decay);

    if constexpr (Self::eval_order_depth >= geometry::KinematicOrder::Twist) {
      const TwistType scaled_linear_factor = linear_factor * omega_;
      // B_ and scaled_linear_factor are the same order, so "-" would resolve to the special
      // diff operator (which returns the next higher order) instead of a same-order subtraction.
      out_state.twist() = (B_ + (-scaled_linear_factor)) * decay;

      if constexpr (Self::eval_order_depth >= geometry::KinematicOrder::Accel) {
        out_state.accel() = (scaled_linear_factor - (B_ * 2)) * omega_ * decay;
      }
    }

    static_assert(Self::eval_order_depth < geometry::KinematicOrder::Jerk,  // line break
                  "This kinematic depth has not yet been implemented, just do it.");
  }

  inline KinematicEvalState evaluate(const TimestampType& time) const
  {
    KinematicEvalState out_state;
    evaluate(time, out_state);
    return out_state;
  }

private:
  TimestampType start_time_;
  ScalarType omega_;

  PoseType goal_;
  TwistType A_, B_;
};

}  // namespace duatic::trajectory
