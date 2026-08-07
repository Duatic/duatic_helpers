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
   * settings is the sole source of truth for the velocity and omega bounds (see determine_omega()
   * below); it is kept as a shared_ptr so it may be shared with (and updated by) other owners,
   * but this trajectory only ever reads from it. omega_ is seeded from settings->omega_min()
   * because, with a freshly-constructed zero offset A_ and zero acceleration, determine_omega()
   * would compute exactly that value anyway.
   */
  inline explicit KinematicTrajectoryExponentialApproach(
      std::shared_ptr<KinematicTrajectorySettingsType> shared_settings)
    : settings(std::move(shared_settings))
  {
    assert(settings != nullptr && "Given shared settings don't exist");
    omega_ = settings->omega_min();
  }

  /*
   * Available only if KinematicTrajectorySettingsType is default-constructible: creates a privately-owned
   * settings object (not shared with any other owner) using its defaults.
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

private:
  /*
   * assumed the internal variable A_ has already been determined !
   */
  inline ScalarType determine_omega(const TwistType& v_zero, const AccelType& a_zero) const
  {
    assert(settings->velocity_limit_linear() >= 0.0);
    assert(settings->velocity_limit_angular() >= 0.0);
    return std::min(determine_omega(settings->velocity_limit_linear(), v_zero.linear().norm(), a_zero.linear().norm(),
                                    A_.linear().norm()),
                    determine_omega(settings->velocity_limit_angular(), v_zero.angular().norm(),
                                    a_zero.angular().norm(), A_.angular().norm()));
  }

  /*
   * Finds the fastest omega respecting both remaining envelope pieces *simultaneously* (term 1,
   * |v0|, is omega-independent and plays no part in this choice): term 3's constraint,
   * (v_zero + 2*omega*a)/e^2 <= v_max, is monotonically increasing in omega, giving a feasible
   * upper bound omega_upper_lin; term 2's constraint, (v_zero + a_zero/(2*omega) + omega*a/2)/e <=
   * v_max, is U-shaped, giving a feasible interval [omega_lower_sq, omega_upper_sq] between its
   * two roots (if real).
   *
   * An earlier version of this function computed each term's own "best omega" independently and
   * combined them with std::min() -- but that's unsound: term 3's formula has no idea a_zero
   * (acceleration) exists, so whenever it recommends an omega smaller than term 2's
   * acceleration-safety lower bound, std::min() picks the unsafe one regardless of what term 2
   * says (confirmed by a randomized stress test finding a genuine, if less severe, recurrence of
   * the omega-too-small divergence this was built to fix). The two constraints must instead be
   * intersected as actual *ranges*: the combined feasible interval is
   * [omega_lower_sq, min(omega_upper_lin, omega_upper_sq)], further intersected with the
   * configured [omega_min, omega_max]; the fastest available omega is then simply its upper end.
   *
   * If the combined range is empty (the two pieces can't be simultaneously satisfied within
   * [omega_min, omega_max]), going slower than omega_lower_sq is the more dangerous direction (a
   * tiny omega barely decays, so nonzero acceleration can drive an essentially unbounded velocity
   * excursion), so the fallback prefers staying at or above omega_lower_sq even at the cost of
   * violating term 3's cap, which only risks a bounded (if possibly large) position-offset-driven
   * overshoot instead -- confirmed against a brute-force search over the allowed omega range to be
   * close to the best any omega choice can achieve in that regime, not merely "less bad".
   */
  inline ScalarType determine_omega(const ScalarType v_max, const ScalarType v_zero, const ScalarType a_zero,
                                    const ScalarType a) const
  {
    assert(v_max >= 0.0);
    assert(v_zero >= 0.0);
    assert(a_zero >= 0.0);
    assert(a >= 0.0);
    assert(settings->omega_min() > 0.0);
    assert(settings->omega_max() >= settings->omega_min());

    constexpr ScalarType e = std::numbers::e_v<ScalarType>;
    constexpr ScalarType e_sq = e * e;
    const ScalarType infinity = std::numeric_limits<ScalarType>::infinity();

    // term 3: omega <= omega_upper (unconstrained, or unconditionally infeasible, if a == 0, since
    // the term doesn't depend on omega at all in that case).
    ScalarType omega_upper = infinity;
    if (a > static_cast<ScalarType>(0.0)) {
      omega_upper = ((e_sq * v_max) - v_zero) / (static_cast<ScalarType>(2.0) * a);
    } else if (v_zero > e_sq * v_max) {
      omega_upper = -infinity;  // unconditionally infeasible: no omega helps.
    }

    // term 2: feasible on [omega_lower, omega_upper_sq] between its two roots (if real); folded
    // into the same omega_upper above via std::min(). If a == 0 (and hence this loop's caller
    // already returned via the branch above), this term imposes no further constraint either.
    ScalarType omega_lower = static_cast<ScalarType>(0.0);
    if (a > static_cast<ScalarType>(0.0)) {
      const ScalarType v_decision = (e * v_max) - v_zero;
      const ScalarType discriminant = (v_decision * v_decision) - (a * a_zero);
      const ScalarType vertex = std::sqrt(a_zero / a);
      const ScalarType sqrt_discriminant =
          (discriminant >= static_cast<ScalarType>(0.0)) ? std::sqrt(discriminant) : static_cast<ScalarType>(0.0);
      const ScalarType root_hi = (v_decision + sqrt_discriminant) / a;
      if (discriminant < static_cast<ScalarType>(0.0) || root_hi <= static_cast<ScalarType>(0.0)) {
        // Either no real root (this term's minimum alone already exceeds v_max for every omega),
        // or both roots are non-positive (the entire feasible interval sits at/below omega=0, so
        // it's equally unreachable for any actual, positive omega) -- v_zero dominating v_max is
        // what drives v_decision very negative and causes this. Either way, this term's own vertex
        // omega* = sqrt(a_zero / a) (balancing the acceleration-vs-position-offset tradeoff) is
        // still the least-bad single point, so treat it as a degenerate [vertex, vertex] "feasible
        // interval" instead of naively clamping a meaningless negative root up to 0.
        omega_lower = vertex;
        omega_upper = std::min(omega_upper, vertex);
      } else {
        const ScalarType root_lo = (v_decision - sqrt_discriminant) / a;
        omega_lower = std::max(static_cast<ScalarType>(0.0), root_lo);
        omega_upper = std::min(omega_upper, root_hi);
      }
    }

    const ScalarType lower_bound = std::max(settings->omega_min(), omega_lower);
    const ScalarType upper_bound = std::min(settings->omega_max(), omega_upper);
    if (lower_bound <= upper_bound) {
      return upper_bound;
    }
    // infeasible: prefer staying at or above the acceleration-safety lower bound (clamped into the
    // configured range) over respecting term 3's cap -- see the header comment above.
    return std::clamp(omega_lower, settings->omega_min(), settings->omega_max());
  }

public:
  std::shared_ptr<KinematicTrajectorySettingsType> settings;

private:
  ScalarType omega_;
  TimestampType start_time_;
  PoseType goal_;
  TwistType A_, B_, C_;
};

}  // namespace duatic::trajectory
