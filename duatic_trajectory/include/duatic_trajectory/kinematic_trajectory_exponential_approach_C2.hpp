#pragma once

#include <algorithm>
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
 * Exponential approach towards a goal, now matching the initial acceleration too (triple pole at
 * -omega instead of the double pole used by the C1 variant):
 *
 *      x(t) = goal + (A + B*t + C*t^2) * e^(-omega*t)
 *
 * with A = x0 - goal,
 *      B = v0 + omega*A,
 *      C = (a0 + 2*omega*B - omega^2*A) / 2
 *
 * Derivatives:
 *
 *   x'(t)  = [(B - omega*A) + (2C - omega*B)t - omega*C*t^2] * e^(-omega*t)
 *   x''(t) = [(2C - 2*omega*B + omega^2*A) + (omega^2*B - 4*omega*C)t + omega^2*C*t^2] * e^(-omega*t)
 *
 * ALTERNATIVE: Approximate V-Limit (see the attached mathematical proof: "Derived velocity bounds
 * for third-order system using superposition decomposition"), refined into three per-piece
 * envelope bounds (tighter than a single triangle-inequality sum over all three basis responses):
 *
 *   |x'(t)| <= max( |v0|,
 *                   (|v0| + |a0|/(2*omega) + omega*|A|/2) / e,
 *                   (|v0| + 2*omega*|A|) / e^2 )
 *
 *  Requiring |x'(t)| <= v_max is thus equivalent to requiring all three pieces individually <=
 *  v_max:
 *   - term 1 (|v0|) doesn't depend on omega at all -- either the initial velocity itself already
 *     respects the limit, or nothing here can fix that.
 *   - term 2 is U-shaped in omega (minimum at omega* = sqrt(|a0|/|A|)), giving a feasible interval
 *     [omega2_minus, omega2_plus] found via the quadratic formula from
 *     |A|*omega^2 - 2*(e*v_max - |v0|)*omega + |a0| <= 0.
 *   - term 3 is monotonically increasing in omega, giving a feasible upper bound
 *     omega3 = (e^2*v_max - |v0|) / (2*|A|).
 *
 *  The fastest omega respecting all three is thus min(omega2_plus, omega3), clamped into the
 *  configured [omega_min, omega_max] range; if that combined upper bound falls below omega2_minus
 *  (the two pieces' feasible ranges don't overlap) or term 2 has no real solution at all (its
 *  minimum alone already exceeds v_max), the least-bad achievable omega is used instead.
 *
 * Separate linear and angular max velocities, but synchronize both motions by using the smallest
 * omega of both.
 *
 * Note that the entire non-const calculation can be done in the goal's diff-type and be added to
 * the goal. The diff-evaluation converges toward numerically stable zero.
 */
template <typename ScalarT, typename TimestampT,
          template <typename, geometry::KinematicOrder> typename KinematicVariableT,
          KinematicTrajectorySettingsExponentialApproach KinematicTrajectorySettingsT>
  requires std::convertible_to<typename KinematicTrajectorySettingsT::ScalarType, ScalarT>
class KinematicTrajectoryExponentialApproach<ScalarT, TimestampT, geometry::KinematicOrder::Accel, KinematicVariableT,
                                             KinematicTrajectorySettingsT>
{
public:
  using ScalarType = ScalarT;
  using TimestampType = TimestampT;
  static constexpr geometry::KinematicOrder continuity_order = geometry::KinematicOrder::Accel;
  using KinematicTrajectorySettingsType = KinematicTrajectorySettingsT;

  using Self = KinematicTrajectoryExponentialApproach<ScalarType, TimestampType, continuity_order, KinematicVariableT,
                                                      KinematicTrajectorySettingsType>;

  template <geometry::KinematicOrder Order>
  using KinematicVariable = KinematicVariableT<ScalarType, Order>;
  template <geometry::KinematicOrder OrderDepth>
  using KinematicState = geometry::KinematicState<ScalarType, OrderDepth, KinematicVariableT>;

  using PoseType = KinematicVariable<geometry::KinematicOrder::Pose>;
  using TwistType = KinematicVariable<geometry::KinematicOrder::Twist>;
  using AccelType = KinematicVariable<geometry::KinematicOrder::Accel>;
  using JerkType = KinematicVariable<geometry::KinematicOrder::Jerk>;
  using AccelStateType = KinematicState<geometry::KinematicOrder::Accel>;

  using UpdateStateType = geometry::TimedData<AccelStateType, TimestampType>;

  using TrajectoryDescriptionType = PoseType;

  /*
   * limits is the sole source of truth for the velocity and omega bounds (see determine_omega()
   * below); it is kept as a shared_ptr so it may be shared with (and updated by) other owners,
   * but this trajectory only ever reads from it. omega_ is seeded from limits->omega_min()
   * because, with a freshly-constructed zero offset A_ and zero acceleration, determine_omega()
   * would compute exactly that value anyway.
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
    const TwistType v0 = in_update_state.twist();
    const AccelType a0 = in_update_state.accel();
    omega_ = determine_omega(v0, a0);
    B_ = v0 + (A_ * omega_);
    // C = a0/2 + omega*B - (omega^2/2)*A; the a0 term is added last so the Twist+Accel -> Twist
    // composition operator can fold it in (the reverse order, Accel+Twist, isn't defined).
    C_ = (B_ * omega_) + (A_ * (static_cast<ScalarType>(-0.5) * omega_ * omega_)) + (a0 * static_cast<ScalarType>(0.5));
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
    // P(t) = A + B*t + C*t^2, and its first derivative P'(t) = B + 2*C*t (P''(t) is the constant
    // 2*C, folded in directly below since it needs no further computation).
    const TwistType quadratic_factor = (A_ + (B_ * t)) + (C_ * (t * t));

    out_state.pose() = goal_ + (quadratic_factor * decay);

    if constexpr (Order >= geometry::KinematicOrder::Twist) {
      const TwistType quadratic_derivative = B_ + (C_ * (t * 2));
      // Same-order Twist arithmetic, so "-" would resolve to the special diff operator (which
      // returns the next higher order) instead of a same-order subtraction; negate-and-add instead.
      out_state.twist() = (quadratic_derivative + (quadratic_factor * (-omega_))) * decay;

      if constexpr (Order >= geometry::KinematicOrder::Accel) {
        // accel(t)/decay = Q''(t) - 2*omega*Q'(t) + omega^2*Q(t), computed in Twist arithmetic
        // (all three operands are TwistType) and then reinterpreted as AccelType via its raw
        // vector -- unlike C1, this can't rely on the same-order "-" diff-operator promotion
        // trick, since accel here isn't a subtraction of exactly two Twist operands.
        const TwistType accel_factor = ((C_ * static_cast<ScalarType>(2.0)) + (quadratic_factor * (omega_ * omega_))) +
                                       (quadratic_derivative * (static_cast<ScalarType>(-2.0) * omega_));
        out_state.accel() = AccelType(accel_factor.vector()) * decay;

        if constexpr (Order >= geometry::KinematicOrder::Jerk) {
          // jerk(t)/decay = -3*omega*Q''(t) + 3*omega^2*Q'(t) - omega^3*Q(t)  (Q'''(t) == 0)
          const TwistType jerk_factor = ((quadratic_derivative * (static_cast<ScalarType>(3.0) * omega_ * omega_)) +
                                         (C_ * (static_cast<ScalarType>(-6) * omega_))) +
                                        (quadratic_factor * (-omega_ * omega_ * omega_));
          out_state.jerk() = JerkType(jerk_factor.vector()) * decay;
        }
      }
    }

    static_assert(Order < geometry::KinematicOrder::Snap,  // line break
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
   * assumed the internal variable A_ has already been determined !
   */
  inline ScalarType determine_omega(const TwistType& v_zero, const AccelType& a_zero) const
  {
    assert(limits->velocity_limit_linear() >= 0.0);
    assert(limits->velocity_limit_angular() >= 0.0);
    return std::min(determine_omega(limits->velocity_limit_linear(), v_zero.linear().norm(), a_zero.linear().norm(),
                                    A_.linear().norm()),
                    determine_omega(limits->velocity_limit_angular(), v_zero.angular().norm(), a_zero.angular().norm(),
                                    A_.angular().norm()));
  }

  /*
   * The fastest omega respecting both term 2 and term 3 (term 1 being omega-independent, it plays
   * no part in this choice) is simply the smaller of the two -- each sub-function already clamps
   * its own candidate into [omega_min, omega_max], and since std::clamp is monotonic, the min of
   * two clamped candidates equals the clamp of their min, so no further clamping is needed here.
   */
  inline ScalarType determine_omega(const ScalarType v_max, const ScalarType v_zero, const ScalarType a_zero,
                                    const ScalarType a) const
  {
    return std::min(  // minimum of all estimates, clamped into the configured [omega_min, omega_max] range
        determine_omega_term_lin(v_max, v_zero, a),        //
        determine_omega_term_sq(v_max, v_zero, a_zero, a)  //
    );                                                     //
  }

  /*
   * Linear Term's constraint, (v_zero + 2*omega*a)/e^2 <= v_max, is monotonically increasing in
   * omega (same shape as the C1 double-pole bound, just rescaled: e^2 instead of e, 2*a instead
   * of a), so it simply clamps omega from above. Division only happens in the else-branch,
   * which is reachable only when two_a > 0.
   */
  inline ScalarType determine_omega_term_lin(const ScalarType v_max, const ScalarType v_zero, const ScalarType a) const
  {
    assert(v_max >= 0.0);
    assert(v_zero >= 0.0);
    assert(a >= 0.0);
    assert(limits->omega_min() > 0.0);
    assert(limits->omega_max() >= limits->omega_min());

    constexpr ScalarType e_sq = std::numbers::e_v<ScalarType> * std::numbers::e_v<ScalarType>;
    const ScalarType v_decision = (e_sq * v_max) - v_zero;
    const ScalarType two_a = static_cast<ScalarType>(2.0) * a;

    if (limits->omega_min() * two_a >= v_decision) {
      return limits->omega_min();
    } else if (limits->omega_max() * two_a < v_decision) {
      return limits->omega_max();
    } else {
      return v_decision / two_a;
    }
  }

  /*
   * Quadratic Term's constraint, (v_zero + a_accel/(2*omega) + omega*a_pos/2)/e <= v_max, is equivalent
   * (multiplying through by 2*omega > 0) to a_pos*omega^2 - 2*(e*v_max-v_zero)*omega + a_accel <= 0
   * -- U-shaped in omega, feasible on the interval [omega_minus, omega_plus] between its two roots
   * (if real).
   *
   * The answer is always the point in [omega_min, omega_max] closest to omega_plus -- if omega_plus
   * itself falls outside that range, the least-bad choice is simply the nearest endpoint -- which is
   * exactly clamp(omega_plus, omega_min, omega_max). omega_minus never changes that answer (so it's
   * not needed at all, not even for a feasibility check): omega_minus <= omega_plus always holds, so
   * moving from omega_plus *toward* [omega_min, omega_max] can only ever move into the feasible
   * interval or stop short of it, never overshoot past omega_minus out the other side.
   */
  inline ScalarType determine_omega_term_sq(const ScalarType v_max, const ScalarType v_zero, const ScalarType a_zero,
                                            const ScalarType a) const
  {
    assert(v_max >= 0.0);
    assert(v_zero >= 0.0);
    assert(a_zero >= 0.0);
    assert(a >= 0.0);
    assert(limits->omega_min() > 0.0);
    assert(limits->omega_max() >= limits->omega_min());

    if (a <= static_cast<ScalarType>(0.0)) {
      // no upper bound from this term: satisfied for all sufficiently large omega, so the fastest
      // allowed omega is always at least as good.
      return limits->omega_max();
    }

    constexpr ScalarType e = std::numbers::e_v<ScalarType>;
    const ScalarType v_decision = (e * v_max) - v_zero;
    const ScalarType discriminant = (v_decision * v_decision) - (a * a_zero);
    if (discriminant < static_cast<ScalarType>(0.0)) {
      // this term's minimum alone already exceeds v_max: no omega satisfies it. Fall back to the
      // omega minimizing it, i.e. its vertex omega* = sqrt(a_zero / a).
      return std::clamp(std::sqrt(a_zero / a), limits->omega_min(), limits->omega_max());
    }

    const ScalarType omega_plus = (v_decision + std::sqrt(discriminant)) / a;
    return std::clamp(omega_plus, limits->omega_min(), limits->omega_max());
  }

  ScalarType omega_;
  TimestampType start_time_;
  PoseType goal_;
  TwistType A_, B_, C_;
};

}  // namespace duatic::trajectory
