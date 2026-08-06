#pragma once

#include <cassert>
#include <cmath>
#include <concepts>
#include <limits>
#include <memory>
#include <numbers>
#include <type_traits>
#include <utility>

#include <Eigen/Geometry>

#include <duatic_geometry/geometry.hpp>
#include <duatic_trajectory/kinematic_trajectory.hpp>
#include <duatic_trajectory/trajectory.hpp>

namespace duatic::trajectory
{

/*
 * Exponential approach towards a goal:
 *
 *      x(t) = goal + (A + B * t) * e^(-omega * t)
 * ->  x'(t) = (B - omega * (A + B * t)) * e^(-omega * t) = (v_0 - (B * omega * t)) * e^(-omega * t)
 * -> x''(t) = omega * (omega * (A + B * t) - 2*B) * e^(-omega * t)
 *
 *   A = x0 - goal          (initial offset from the goal)
 *   B = v0 + omega * A     (initial velocity, corrected for the offset)
 *
 * omega is tracking the convergence speed: higher values converge faster.
 *
 * x''(t_a0) = 0  ->  t_a0 = 2/omega - A/B
 * x'(t_a0) = -B * e^(omega*A/B -2)           <- NOTE: This equation is not explicitly solvable for omega !
 *
 * ALTERNATIVE: Approximate V-Limit
 *  1. Ignore the initial velocity, even if it exceeds the limit
 *  2. Satisfy the following approximation: v(t) <= (|v| + omega * |A|) / e = v_max
 *     -> omega = ((v_max * e) - |v|) / |A|   IF |A| = 0 : use max omega (given by parameter)
 * -> See attached mathematical proof including omega-clamp introduction
 *
 * Separate linear and angular max velocities, but synchonize both mothins by using the smalles omega of both
 *
 * Note that the entire non-const calculation can be done in the goal's diff-type and be added to the goal.
 * The diff-evlaluation converges toward numerically stable zero
 */
template <typename ScalarT, typename TimestampT,
          template <typename, geometry::KinematicOrder> typename KinematicVariableT,
          KinematicTrajectorySettingsExponentialApproach KinematicTrajectorySettingsT>
  requires std::convertible_to<typename KinematicTrajectorySettingsT::ScalarType, ScalarT>
class KinematicTrajectoryExponentialApproach<ScalarT, TimestampT, geometry::KinematicOrder::Twist, KinematicVariableT,
                                             KinematicTrajectorySettingsT>
{
public:
  using ScalarType = ScalarT;
  using TimestampType = TimestampT;
  static constexpr geometry::KinematicOrder continuity_order = geometry::KinematicOrder::Twist;
  using KinematicTrajectorySettingsType = KinematicTrajectorySettingsT;

  using Self = KinematicTrajectoryExponentialApproach<ScalarType, TimestampType, continuity_order, KinematicVariableT,
                                                      KinematicTrajectorySettingsType>;

  template <geometry::KinematicOrder Order>
  using KinematicVariable = KinematicVariableT<ScalarType, Order>;
  template <geometry::KinematicOrder OrderDepth>
  using KinematicState = geometry::KinematicState<ScalarType, OrderDepth, KinematicVariableT>;

  using PoseType = KinematicVariable<geometry::KinematicOrder::Pose>;
  using TwistType = KinematicVariable<geometry::KinematicOrder::Twist>;
  using TwistStateType = KinematicState<geometry::KinematicOrder::Twist>;

  using UpdateStateType = geometry::TimedData<TwistStateType, TimestampType>;

  using TrajectoryDescriptionType = PoseType;

  /*
   * limits is the sole source of truth for the velocity and omega bounds (see determine_omega()
   * below); it is kept as a shared_ptr so it may be shared with (and updated by) other owners,
   * but this trajectory only ever reads from it. omega_ is seeded from limits->omega_max()
   * because, with a freshly-constructed zero offset A_, determine_omega() would compute exactly
   * that value anyway (see the attached mathematical proof).
   */
  inline explicit KinematicTrajectoryExponentialApproach(std::shared_ptr<KinematicTrajectorySettingsType> shared_limits)
    : limits(std::move(shared_limits))
  {
    assert(limits != nullptr && "Given shared limits don't exist");
    omega_ = limits->omega_min();
  }

  /*
   * Available only if KinematicTrajectorySettingsType is default-constructible: creates a privately-owned
   * limits object (not shared with any other owner) using its defaults.
   */
  inline KinematicTrajectoryExponentialApproach()
    requires std::default_initializable<KinematicTrajectorySettingsType>
    : KinematicTrajectoryExponentialApproach(std::make_shared<KinematicTrajectorySettingsType>())
  {
  }

  inline void calculate(const UpdateStateType& in_update_state, const TrajectoryDescriptionType& in_description)
  {
    start_time_ = in_update_state.time();
    goal_ = in_description;
    A_ = in_update_state.pose() - goal_;
    omega_ = determine_omega(in_update_state.twist());
    B_ = in_update_state.twist() + (A_ * omega_);
  }

  inline void calculate_neutral(const UpdateStateType& in_update_state)
  {
    calculate(in_update_state, in_update_state.data().pose());
  }

  /*
   * Replans starting from the 'other' trajectory's predicted state at in_timestamp (rather than an
   * externally supplied UpdateStateType), so the caller only needs to provide the new target.
   */
  inline void update_from(const Self& other, const TimestampType& in_timestamp,
                          const TrajectoryDescriptionType& in_description)
  {
    // there are no future trajectory data existing to be copied
    calculate(
        UpdateStateType(in_timestamp, other.evaluate<UpdateStateType::DataType::kinematic_order_depth>(in_timestamp)),
        in_description);
  }

  /*
   * Replans starting from this trajectory's own predicted state at in_timestamp (rather than an
   * externally supplied UpdateStateType), so the caller only needs to provide the new target.
   */
  inline void update(const TimestampType& in_timestamp, const TrajectoryDescriptionType& in_description)
  {
    update_from(*this, in_timestamp, in_description);
  }

  inline void update_neutral(const TimestampType& in_timestamp)
  {
    calculate_neutral(
        UpdateStateType(in_timestamp, evaluate<UpdateStateType::DataType::kinematic_order_depth>(in_timestamp)));
  }

  template <geometry::KinematicOrder Order>
  inline void evaluate(const TimestampType& time, KinematicState<Order>& out_state) const
  {
    const ScalarType t = (time - start_time_).seconds();
    const ScalarType decay = std::exp(-omega_ * t);
    const TwistType linear_factor = A_ + (B_ * t);

    out_state.pose() = goal_ + (linear_factor * decay);

    if constexpr (Order >= geometry::KinematicOrder::Twist) {
      const TwistType scaled_linear_factor = linear_factor * omega_;
      // B_ and scaled_linear_factor are the same order, so "-" would resolve to the special
      // diff operator (which returns the next higher order) instead of a same-order subtraction.
      out_state.twist() = (B_ + (-scaled_linear_factor)) * decay;

      if constexpr (Order >= geometry::KinematicOrder::Accel) {
        out_state.accel() = (scaled_linear_factor - (B_ * 2)) * omega_ * decay;
      }
    }

    static_assert(Order < geometry::KinematicOrder::Jerk,  // line break
                  "This kinematic depth has not yet been implemented, just do it.");
  }

  template <geometry::KinematicOrder Order>
  inline KinematicState<Order> evaluate(const TimestampType& time) const
  {
    KinematicState<Order> out_state;
    evaluate<Order>(time, out_state);
    return out_state;
  }

public:
  std::shared_ptr<KinematicTrajectorySettingsType> limits;

private:
  /*
   * assumed the internal variable A_ has already benn determined !
   */
  inline ScalarType determine_omega(const TwistType& v_zero) const
  {
    assert(limits->velocity_limit_linear() >= 0.0);
    assert(limits->velocity_limit_angular() >= 0.0);
    return std::min(determine_omega(limits->velocity_limit_linear(), v_zero.linear().norm(), A_.linear().norm()),
                    determine_omega(limits->velocity_limit_angular(), v_zero.angular().norm(), A_.angular().norm()));
  }

  inline ScalarType determine_omega(const ScalarType v_max, const ScalarType v_zero, const ScalarType a) const
  {
    assert(v_max >= 0.0);
    assert(v_zero >= 0.0);
    assert(a >= 0.0);
    assert(limits->omega_min() > 0.0);
    assert(limits->omega_max() >= limits->omega_min());
    const ScalarType v_descision = (std::numbers::e_v<ScalarType> * v_max) - v_zero;
    if (limits->omega_min() * a >= v_descision) {
      return limits->omega_min();
    } else if (limits->omega_max() * a < v_descision) {
      return limits->omega_max();
    } else {
      return v_descision / a;
    }
  }

  ScalarType omega_;
  TimestampType start_time_;
  PoseType goal_;
  TwistType A_, B_;
};

}  // namespace duatic::trajectory
