#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <concepts>
#include <memory>
#include <numbers>
#include <type_traits>
#include <utility>

#include <Eigen/Geometry>

#include <duatic_geometry/geometry.hpp>
#include <duatic_trajectory/kinematic_trajectory.hpp>
#include <duatic_trajectory/kinematic_trajectory_base.hpp>
#include <duatic_trajectory/kinematic_trajectory_exponential_approach_C1.hpp>
#include <duatic_trajectory/trajectory.hpp>

namespace duatic::trajectory
{

/*
 * Exponential approach towards a goal, now additionally matching the initial acceleration too --
 * built as an *additive patch* on top of the C1 (Twist-continuity) variant, which this class holds
 * as a private member (c1_) rather than inheriting from, rather than as a separate triple-pole
 * derivation:
 *
 *      x_C2(t) = x_C1(t) + h(t)
 *
 *      x_C1(t) = goal + (A + B*t) * e^(-omega_pv*t)      (term1: c1_'s own trajectory shape)
 *      h(t)    = D * t^2 * e^(-omega_a*t)                (zero position AND velocity at t=0)
 *
 *      A  = x0 - goal
 *      B  = v0 + omega_pv*A
 *      P1 = -omega_pv^2*A - 2*omega_pv*v0                (term1''(0), i.e. x_C1''(0))
 *      Q1 = omega_pv^2*v0 + omega_pv^3*A                 (term1's own d/dt of the accel envelope)
 *      a1 = a0 - P1                                       (residual: whatever term1 doesn't already give)
 *      D  = a1 / 2                                        (patch amplitude)
 *
 * Derivatives -- term1's own derivative formulas untouched, h's derivatives added on top:
 *
 *      x_C2'(t)  = x_C1'(t)  + D*t*(2 - omega_a*t) * e^(-omega_a*t)
 *      x_C2''(t) = x_C1''(t) + D*(2 - 4*omega_a*t + omega_a^2*t^2) * e^(-omega_a*t)
 *
 * Boundary conditions (all exact by construction -- h and h' both vanish at t=0 because of h's
 * double zero there, so term1 alone is responsible for matching pose/twist; h'' does not depend on
 * omega_a at all, so the acceleration match below holds for *any* omega_a > 0):
 *
 *      x_C2(0)   = x_C1(0) + 0    = x0         (term1 already guarantees this)
 *      x_C2'(0)  = x_C1'(0) + 0   = v0         (term1 already guarantees this)
 *      x_C2''(0) = x_C1''(0) + a1 = a0         (exact, given a1 := a0 - x_C1''(0))
 *
 * Two independent convergence rates, solved in a strict one-directional order -- omega_pv first
 * (it never needs to know omega_a exists), omega_a second (it needs omega_pv's result):
 *
 *  - omega_pv (position+velocity+acceleration): unlike the very first cut of this class, this is
 *    NOT simply c1_'s own determine_omega()'s velocity+t=0-jump bound reused unmodified -- that
 *    bound never sees a0, but the combined x_C2''(0) = a0 exactly regardless of omega_pv, so a
 *    large a0 can *only* be reined in by shaping term1 to counteract it (there is no other free
 *    parameter at t=0). determine_omega_pv() below therefore combines c1_'s own velocity ceiling
 *    (identical formula/derivation to c1_'s determine_vel_omega(), just reimplemented here since
 *    that stays private even to friends) with a *new* acceleration ceiling that bounds the combined
 *    trajectory's peak accel via a0 directly, then overwrites c1_'s omega_/B_ with the result (see
 *    calculate() below) -- c1_.calculate() is still called first, purely to obtain A_ from x0 and
 *    the (otherwise-private) goal_, and its own provisional omega_/B_ choice is simply discarded.
 *  - omega_a (the residual correction's own decay rate): solved from how much velocity margin
 *    omega_pv's own (now a0-aware) choice leaves over -- v_max minus the *actual*, not
 *    maximum-allowed, term1 velocity peak -- once the residual a1 (now finalized) is known. Unlike
 *    an earlier version of this function, this is a pure closed form: no retry/backoff search over
 *    omega_pv. If the margin runs out entirely (e.g. an axis' own v0 already exceeds its v_max
 *    outright, so no choice of omega_pv could ever have reopened it), omega_a simply falls back to
 *    omega_max as a documented best-effort approximation.
 *
 * Both determine_omega_pv() and determine_omega_a() compute a per-axis (linear/angular) candidate
 * and then combine the two axes -- omega_pv via MIN (mirroring c1_'s own determine_omega()'s own
 * lin/ang combination: the more restrictive axis governs), omega_a via MAX (the *faster* axis must
 * win here, since a slower shared omega_a would leave the other axis' own residual decaying too
 * slowly -- this is unchanged from, and was already established by, an earlier version of this
 * class; the axis-combination direction isn't spelled out by the scalar derivation these two
 * functions otherwise follow).
 *
 * Deliberate deviations from a strict reading of the underlying derivation:
 *  - No hard "INFEASIBLE" rejection when v_max <= |v0| or a_max <= |a0|: both omega ceilings above
 *    degrade gracefully to their best-effort endpoint (omega_min or omega_max, whichever is safer)
 *    instead, consistent with every other limit in this codebase (see e.g.
 *    kinematic_trajectory_exponential_approach_C1.hpp's own "Approximate V-/A-Limit" philosophy).
 */
template <typename ScalarT, typename TimestampT,
          template <typename, geometry::KinematicOrder> typename KinematicVariableT,
          KinematicTrajectorySettingsExponentialApproach KinematicTrajectorySettingsT>
  requires std::convertible_to<typename KinematicTrajectorySettingsT::ScalarType, ScalarT>
class KinematicTrajectoryExponentialApproach<ScalarT, TimestampT, geometry::KinematicOrder::Accel, KinematicVariableT,
                                             KinematicTrajectorySettingsT>
  : public KinematicTrajectoryBase<KinematicTrajectorySettingsT, geometry::KinematicOrder::Accel>
{
  /*
   * This class holds a private instance of the C1 (Twist-continuity) specialization (c1_ below) and
   * reaches directly into its internal state (start_time_, A_, B_, omega_) rather than through
   * public API, since it patches c1_'s own trajectory shape with an additive correction term (see
   * the derivation above). C++ does not allow a friend declaration to name one specific
   * specialization of a template that is itself defined only via per-ContinuityOrder specializations
   * (there is no single specialization here to name until C1's own header has been included) -- so
   * the whole ContinuityOrder-indexed family is friended instead, matching the primary template's
   * own declaration in kinematic_trajectory_exponential_approach.hpp.
   */
  template <typename FriendScalarT, typename FriendTimestampT, geometry::KinematicOrder FriendContinuityOrder,
            template <typename, geometry::KinematicOrder> typename FriendKinematicVariableT,
            KinematicTrajectorySettingsExponentialApproach FriendKinematicTrajectorySettingsT>
    requires std::convertible_to<typename FriendKinematicTrajectorySettingsT::ScalarType, FriendScalarT>
  friend class KinematicTrajectoryExponentialApproach;

public:
  using ScalarType = ScalarT;
  using TimestampType = TimestampT;
  using KinematicTrajectorySettingsType = KinematicTrajectorySettingsT;

  using Base = KinematicTrajectoryBase<KinematicTrajectorySettingsType, geometry::KinematicOrder::Accel>;
  using Base::continuity_order;
  using Base::settings;

  // c1_'s own type (the Twist-continuity specialization of this same template family); see c1_ below.
  using C1 = KinematicTrajectoryExponentialApproach<ScalarType, TimestampType, geometry::KinematicOrder::Twist,
                                                    KinematicVariableT, KinematicTrajectorySettingsType>;

  using Self = KinematicTrajectoryExponentialApproach<ScalarType, TimestampType, continuity_order, KinematicVariableT,
                                                      KinematicTrajectorySettingsType>;

  template <geometry::KinematicOrder Order>
  using KinematicVariable = KinematicVariableT<ScalarType, Order>;
  template <geometry::KinematicOrder OrderDepth>
  using KinematicState = geometry::KinematicState<ScalarType, OrderDepth, KinematicVariableT>;

  using PoseType = KinematicVariable<geometry::KinematicOrder::Pose>;
  using TwistType = KinematicVariable<geometry::KinematicOrder::Twist>;
  using AccelType = KinematicVariable<geometry::KinematicOrder::Accel>;
  using AccelStateType = KinematicState<geometry::KinematicOrder::Accel>;

  using UpdateStateType = geometry::TimedData<AccelStateType, TimestampType>;

  using TrajectoryDescriptionType = PoseType;

  /*
   * settings is owned independently by Base (this class' own KinematicTrajectoryBase) and by c1_'s
   * own Base -- both point at the *same* settings object, though, since shared_settings itself
   * (rather than a copy of its pointee) is passed to both constructors below. omega_a_ is seeded
   * from settings().omega_min() as a well-defined default before the first calculate().
   */
  inline explicit KinematicTrajectoryExponentialApproach(
      std::shared_ptr<KinematicTrajectorySettingsType> shared_settings)
    : Base(shared_settings), c1_(shared_settings)
  {
    omega_a_ = this->settings().omega_min();
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
    const TwistType v0 = in_update_state.twist();
    const AccelType a0 = in_update_state.accel();

    // Step 1: delegate to c1_ purely to obtain start_time_ and A_ = x0 - goal_ (goal_ itself stays
    // private to c1_, so this is the only way to get at A_). c1_'s own omega_/B_ choice here is
    // provisional and gets overwritten by Step 2 below -- it never saw a0.
    c1_.calculate(typename C1::UpdateStateType(in_update_state.time(), in_update_state.pose(), v0), in_description);

    // Step 2: omega_pv -- solved directly from v0, A_, a0, v_max, a_max (see determine_omega_pv()),
    // superseding c1_'s own provisional choice from Step 1. B_ depends on omega_, so it must be
    // recomputed here too (identical formula to c1_'s own calculate()'s B_ = v0 + omega*A_).
    c1_.omega_ = determine_omega_pv(v0, a0);
    c1_.B_ = v0 + (c1_.A_ * c1_.omega_);

    // Step 3: the residual acceleration term1 (at its now-final omega_pv) doesn't cover, and the
    // resulting patch amplitude. a0 and P1 (through its AccelType reinterpretation) are the same
    // order, so "-" would resolve to the diff-promoting operator (giving a Jerk) rather than a
    // same-order subtraction -- negate-and-add instead.
    const TwistType p1 = (c1_.A_ * (-c1_.omega_ * c1_.omega_)) + (v0 * (static_cast<ScalarType>(-2) * c1_.omega_));
    const AccelType a1 = a0 + (-AccelType(p1.vector()));
    D_ = TwistType(a1.vector()) * static_cast<ScalarType>(0.5);

    // Step 4: omega_a -- solved downstream of the now-final omega_pv and a1, closed-form (see
    // determine_omega_a()).
    omega_a_ = determine_omega_a(v0, a1);
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
    calculate(UpdateStateType(in_timestamp,
                              other.template evaluate<UpdateStateType::DataType::kinematic_order_depth>(in_timestamp)),
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
    // Evaluate c1_ directly into out_state (reusing c1_'s own internally-shared decay computation
    // rather than calling c1_.evaluate<>() three separate times), then add h(t)'s contribution on
    // top in place.
    c1_.template evaluate<Order>(time, out_state);

    const ScalarType t = (time - c1_.start_time_).seconds();
    const ScalarType decay_a = std::exp(-omega_a_ * t);

    // h(t) = D * t^2 * e^(-omega_a*t): the additive correction bringing x_C1''(0) up to a0 exactly,
    // while leaving x_C1's own pose/twist at t=0 undisturbed (h and h' both vanish there). Pose has
    // no += (only Twist/Accel/... do), so this one stays a plain assignment of Pose + Twist -> Pose.
    out_state.pose() = out_state.pose() + (D_ * (t * t * decay_a));

    if constexpr (Order >= geometry::KinematicOrder::Twist) {
      // h'(t) = D*t*(2 - omega_a*t)*e^(-omega_a*t)
      out_state.twist() += D_ * (t * (static_cast<ScalarType>(2) - (omega_a_ * t)) * decay_a);

      if constexpr (Order >= geometry::KinematicOrder::Accel) {
        // h''(t) = D*(2 - 4*omega_a*t + omega_a^2*t^2)*e^(-omega_a*t), computed in TwistType
        // arithmetic (D_ is a TwistType) and then reinterpreted as AccelType via its raw vector --
        // this can't rely on the same-order "-" diff-operator promotion trick since accel here isn't
        // a subtraction of exactly two Twist operands.
        const TwistType h_double_prime =
            D_ * ((static_cast<ScalarType>(2) - (static_cast<ScalarType>(4) * omega_a_ * t) +
                   (omega_a_ * omega_a_ * t * t)) *
                  decay_a);
        out_state.accel() += AccelType(h_double_prime.vector());
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

private:
  /*
   * One axis' ceiling on omega_pv, combining two independent bounds:
   *
   *  - the velocity ceiling: identical formula/derivation to c1_'s own determine_vel_omega() (kept
   *    private there even to friends, so it's reimplemented here rather than reused).
   *
   *  - the acceleration ceiling: bounds the *combined* C2 trajectory's peak accel via a0 directly
   *    (unlike c1_'s own determine_acc_omega(), which only ever sees term1's own t=0 jump), by
   *    solving
   *        a_offset*(2+1/e)*omega^2 + v_zero*(4+1/e)*omega + (a_zero - a_max) = 0
   *    for the smallest nonnegative omega satisfying it. The left-hand side is non-decreasing in
   *    omega (all three coefficients are >= 0 given a_offset, v_zero >= 0), so -- exactly like every
   *    other omega-bound in this codebase -- the range's endpoints are checked first (omega_min
   *    checked first for the same "safer on an exact tie" reason as elsewhere), and only the
   *    in-between case needs the quadratic solved at all. The root itself uses the same
   *    "rationalized" form as c1_'s own determine_acc_omega() (multiplying by the conjugate instead
   *    of subtracting two close values); as a bonus, unlike the naive -b+sqrt(...)/(2a) form, this
   *    one stays well-defined as a_offset -> 0 (collapsing smoothly to the then-linear equation's own
   *    solution) without needing a separate branch for it.
   */
  inline ScalarType determine_omega_pv_component(const ScalarType v_max, const ScalarType a_max,
                                                 const ScalarType v_zero, const ScalarType a_offset,
                                                 const ScalarType a_zero) const
  {
    assert(v_max >= 0.0);
    assert(a_max >= 0.0);
    assert(v_zero >= 0.0);
    assert(a_offset >= 0.0);
    assert(a_zero >= 0.0);
    assert(this->settings().omega_min() > 0.0);
    assert(this->settings().omega_max() >= this->settings().omega_min());

    ScalarType omega_vel;
    const ScalarType v_decision = (std::numbers::e_v<ScalarType> * v_max) - v_zero;
    if (this->settings().omega_min() * a_offset >= v_decision) {
      omega_vel = this->settings().omega_min();
    } else if (this->settings().omega_max() * a_offset < v_decision) {
      omega_vel = this->settings().omega_max();
    } else {
      omega_vel = v_decision / a_offset;
    }

    const ScalarType inv_e = static_cast<ScalarType>(1) / std::numbers::e_v<ScalarType>;
    const ScalarType c2 = a_offset * (static_cast<ScalarType>(2) + inv_e);
    const ScalarType c1 = v_zero * (static_cast<ScalarType>(4) + inv_e);
    const ScalarType c0 = a_zero - a_max;

    ScalarType omega_acc;
    const ScalarType omega_min = this->settings().omega_min();
    const ScalarType omega_max = this->settings().omega_max();
    if (((c2 * omega_min * omega_min) + (c1 * omega_min) + c0) >= static_cast<ScalarType>(0)) {
      omega_acc = omega_min;  // already violated at the bottom of the range -- best effort
    } else if (((c2 * omega_max * omega_max) + (c1 * omega_max) + c0) < static_cast<ScalarType>(0)) {
      omega_acc = omega_max;  // unconstrained even at the top of the range
    } else {
      omega_acc =
          (static_cast<ScalarType>(-2) * c0) / (c1 + std::sqrt((c1 * c1) - (static_cast<ScalarType>(4) * c2 * c0)));
    }

    return std::clamp(std::min(omega_vel, omega_acc), this->settings().omega_min(), this->settings().omega_max());
  }

  /*
   * omega_pv: combines the linear and angular axes' own ceilings via MIN (the more restrictive axis
   * governs the single shared rate), then clamps -- mirroring c1_'s own determine_omega()'s own
   * lin/ang combination.
   */
  inline ScalarType determine_omega_pv(const TwistType& v_zero, const AccelType& a_zero) const
  {
    const ScalarType omega_lin = determine_omega_pv_component(
        this->settings().velocity_limit_linear(), this->settings().acceleration_limit_linear(), v_zero.linear().norm(),
        c1_.A_.linear().norm(), a_zero.linear().norm());
    const ScalarType omega_ang = determine_omega_pv_component(
        this->settings().velocity_limit_angular(), this->settings().acceleration_limit_angular(),
        v_zero.angular().norm(), c1_.A_.angular().norm(), a_zero.angular().norm());
    return std::clamp(std::min(omega_lin, omega_ang), this->settings().omega_min(), this->settings().omega_max());
  }

  /*
   * One axis' candidate omega_a: sized so the residual h(t) decays away within roughly the velocity
   * margin omega_pv's (now a0-aware) choice left over -- v_max minus the *actual* peak speed term1
   * reaches (not the maximum it's merely allowed to reach). No residual on this axis (a1_component
   * <= 0) places no constraint on omega_a at all, so that's trivially omega_min -- the non-binding
   * choice for the MAX combination in determine_omega_a() below (omega_min can never win a max()
   * against a genuine, larger need from the other axis). Otherwise, if the margin has run out
   * entirely (e.g. this axis' own v0 already exceeds its v_max outright, pinning the margin at 0
   * regardless of omega_pv), there is no search left to do -- omega_a simply falls back to omega_max
   * as a documented best-effort approximation, rather than iteratively re-shrinking omega_pv the way
   * an earlier version of this function did.
   */
  inline ScalarType determine_omega_a_component(const ScalarType v_max, const ScalarType v_zero,
                                                const ScalarType a_offset, const ScalarType omega_pv,
                                                const ScalarType a1_component) const
  {
    assert(v_max >= 0.0);
    assert(v_zero >= 0.0);
    assert(a_offset >= 0.0);
    assert(omega_pv > 0.0);
    assert(a1_component >= 0.0);

    if (a1_component <= static_cast<ScalarType>(0)) {
      return this->settings().omega_min();
    }

    const ScalarType v1_actual = std::max(v_zero, (v_zero + (omega_pv * a_offset)) / std::numbers::e_v<ScalarType>);
    const ScalarType margin = std::max(v_max - v1_actual, static_cast<ScalarType>(0));
    if (margin <= static_cast<ScalarType>(0)) {
      return this->settings().omega_max();
    }

    const ScalarType kappa = (std::sqrt(static_cast<ScalarType>(2)) - static_cast<ScalarType>(1)) *
                             std::exp(std::sqrt(static_cast<ScalarType>(2)) - static_cast<ScalarType>(2));
    return std::clamp(kappa * a1_component / margin, this->settings().omega_min(), this->settings().omega_max());
  }

  /*
   * omega_a: combines the linear and angular axes' own candidates via MAX -- the *faster* axis must
   * win, since a shared omega_a slower than what either axis' own residual needs would leave that
   * axis decaying too slowly (unlike omega_pv, where the more restrictive/slower axis governs).
   */
  inline ScalarType determine_omega_a(const TwistType& v_zero, const AccelType& a1) const
  {
    const ScalarType omega_a_lin =
        determine_omega_a_component(this->settings().velocity_limit_linear(), v_zero.linear().norm(),
                                    c1_.A_.linear().norm(), c1_.omega_, a1.linear().norm());
    const ScalarType omega_a_ang =
        determine_omega_a_component(this->settings().velocity_limit_angular(), v_zero.angular().norm(),
                                    c1_.A_.angular().norm(), c1_.omega_, a1.angular().norm());
    return std::clamp(std::max(omega_a_lin, omega_a_ang), this->settings().omega_min(), this->settings().omega_max());
  }

  // The Twist-continuity trajectory this class patches an additive acceleration-matching correction
  // on top of; see the derivation above. Constructed sharing the same settings object as this
  // class' own Base (see the constructor above).
  C1 c1_;
  TwistType D_;
  ScalarType omega_a_;
};

}  // namespace duatic::trajectory
