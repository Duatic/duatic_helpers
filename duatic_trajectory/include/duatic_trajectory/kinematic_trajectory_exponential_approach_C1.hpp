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

#include <Eigen/Geometry>
#include <concepts>
#include <numbers>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>
#include <type_traits>
#include <utility>

#include <duatic_geometry/geometry.hpp>
#include <duatic_trajectory/kinematic_trajectory.hpp>
#include <duatic_trajectory/kinematic_trajectory_base.hpp>
#include <duatic_trajectory/kinematic_trajectory_pose_target_base.hpp>
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
 * ALTERNATIVE: Approximate A-Limit
 *  Because only pose and twist (not accel) are matched at t=0, x''(t) can jump discontinuously at the
 *  start of a (re-)plan; unlike the velocity case, this jump is NOT automatically bounded by omega_min/
 *  omega_max/v_max. There are exactly two candidate locations for the peak of |x''(t)| over t >= 0:
 *   1. The t=0 jump itself:       x''(0)    = -(omega^2 * A + 2 * omega * v0)
 *   2. An interior extremum at:   t*        = 3/omega - A/B
 *                                 x''(t*)   = omega * B * e^(omega*A/B - 3) = -(omega/e) * x'(t_a0)
 *  Which one is larger depends only on k = omega*A/B: the t=0 jump dominates for k outside
 *  [k_threshold, 3] (k_threshold = 2 - W(1/e) ~= 1.7215, from solving (2-k)*e^(3-k) = 1); the interior
 *  extremum can exceed it only inside that window. determine_acc_omega() below bounds both candidates
 *  unconditionally rather than gating the interior one on k, for simplicity.
 *
 * Separate linear and angular max velocities, but synchronize both mothins by using the smallest omega of both
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
  : public KinematicTrajectoryPoseTargetBase<
        KinematicTrajectoryExponentialApproach<ScalarT, TimestampT, geometry::KinematicOrder::Twist,
                                               KinematicVariableT, KinematicTrajectorySettingsT>,
        ScalarT, TimestampT, geometry::KinematicOrder::Twist, KinematicVariableT, KinematicTrajectorySettingsT>
{
  /*
   * Allow other instantiations, especially once of higher continuity order, to access this one's private members (e.g.
   * start_time_, A_, B_) for their own calculations if they base their own calculations on this one.
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

  using Self = KinematicTrajectoryExponentialApproach<ScalarType, TimestampType, geometry::KinematicOrder::Twist,
                                                      KinematicVariableT, KinematicTrajectorySettingsType>;

  using Base = KinematicTrajectoryPoseTargetBase<Self, ScalarType, TimestampType, geometry::KinematicOrder::Twist,
                                                 KinematicVariableT, KinematicTrajectorySettingsType>;
  using Base::continuity_order;
  using Base::settings;
  // Un-hide Base's return-by-value evaluate<Order>(time): this class declares its own
  // evaluate<Order>(time, out_state) below, which would otherwise hide the whole inherited
  // "evaluate" overload set (member name hiding is per-name, not per-signature).
  using Base::evaluate;

  template <geometry::KinematicOrder Order>
  using KinematicVariable = KinematicVariableT<ScalarType, Order>;
  template <geometry::KinematicOrder OrderDepth>
  using KinematicState = geometry::KinematicState<ScalarType, OrderDepth, KinematicVariableT>;

  using PoseType = KinematicVariable<geometry::KinematicOrder::Pose>;
  using TwistType = KinematicVariable<geometry::KinematicOrder::Twist>;
  using TwistStateType = KinematicState<geometry::KinematicOrder::Twist>;

  using UpdateStateType = data_annotation::TimedData<TwistStateType, TimestampType>;

  using TrajectoryDescriptionType = PoseType;

  /*
   * settings (the velocity and omega bounds read by determine_omega() below) is owned by Base; only
   * omega_ itself is initialized here. It is seeded from this->settings().omega_max() because, with a
   * freshly-constructed zero offset A_, determine_omega() would compute exactly that value anyway
   * (see the attached mathematical proof).
   */
  inline explicit KinematicTrajectoryExponentialApproach(
      std::shared_ptr<KinematicTrajectorySettingsType> shared_settings)
    : Base(std::move(shared_settings))
  {
    omega_ = this->settings().omega_min();
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
    omega_ = determine_omega(in_update_state.twist());
    B_ = in_update_state.twist() + (A_ * omega_);
  }

  // calculate_neutral(), update_from(), update(), and update_neutral() are inherited from Base --
  // see kinematic_trajectory_pose_target_base.hpp.

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

  // The return-by-value evaluate<Order>(time) overload is inherited from Base (brought into scope
  // above via "using Base::evaluate;") -- see kinematic_trajectory_pose_target_base.hpp.

private:
  /*
   * assumed the internal variable A_ has already benn determined !
   */
  inline ScalarType determine_omega(const TwistType& v_zero) const
  {
    assert(this->settings().velocity_limit_linear() >= 0.0);
    assert(this->settings().velocity_limit_angular() >= 0.0);
    assert(this->settings().acceleration_limit_linear() >= 0.0);
    assert(this->settings().acceleration_limit_angular() >= 0.0);

    const ScalarType omega_lin = std::min(
        determine_vel_omega(this->settings().velocity_limit_linear(), v_zero.linear().norm(), A_.linear().norm()),
        determine_acc_omega(this->settings().acceleration_limit_linear(), this->settings().velocity_limit_linear(),
                            v_zero.linear().norm(), A_.linear().norm()));
    const ScalarType omega_ang = std::min(
        determine_vel_omega(this->settings().velocity_limit_angular(), v_zero.angular().norm(), A_.angular().norm()),
        determine_acc_omega(this->settings().acceleration_limit_angular(), this->settings().velocity_limit_angular(),
                            v_zero.angular().norm(), A_.angular().norm()));

    // Both determine_vel_omega() and determine_acc_omega() already return values within
    // [omega_min, omega_max]; clamp again here regardless, so this stays true even if either helper's
    // assumptions are violated (e.g. a structurally infeasible acceleration limit).
    return std::clamp(std::min(omega_lin, omega_ang), this->settings().omega_min(), this->settings().omega_max());
  }

  inline ScalarType determine_vel_omega(const ScalarType v_max, const ScalarType v_zero, const ScalarType a) const
  {
    assert(v_max >= 0.0);
    assert(v_zero >= 0.0);
    assert(a >= 0.0);
    assert(this->settings().omega_min() > 0.0);
    assert(this->settings().omega_max() >= this->settings().omega_min());

    const ScalarType v_descision = (std::numbers::e_v<ScalarType> * v_max) - v_zero;
    if (this->settings().omega_min() * a >= v_descision) {
      return this->settings().omega_min();
    } else if (this->settings().omega_max() * a < v_descision) {
      return this->settings().omega_max();
    } else {
      return v_descision / a;
    }
  }

  /*
   * Bounds omega so that both candidate peaks of |x''(t)| stay within a_max: the t=0 jump, and the
   * later interior extremum x''(t*) (see the "Approximate A-Limit" note above). The interior bound is
   * applied unconditionally rather than only within the narrow k-window where it can strictly exceed
   * the t=0 term -- simpler, and only ever more conservative than necessary outside that window.
   *
   * v_zero and a are magnitudes (this mirrors determine_vel_omega()'s v_max/v_zero/a signature), so
   * the true sign relationship between the initial twist and the a A_ is unknown here. The t=0
   * bound therefore assumes the worst-case *diverging* alignment (B = v_zero + omega*a, which
   * maximizes |x''(0)|).
   */
  inline ScalarType determine_acc_omega(const ScalarType a_max, const ScalarType v_max, const ScalarType v_zero,
                                        const ScalarType a) const
  {
    assert(a_max >= 0.0);
    assert(v_max >= 0.0);
    assert(v_zero >= 0.0);
    assert(a >= 0.0);
    assert(this->settings().omega_min() > 0.0);
    assert(this->settings().omega_max() >= this->settings().omega_min());

    // |x''(0)| <= a_max  <=>  omega^2 * a + 2 * omega * v_zero <= a_max (worst-case diverging B). The
    // left-hand side is non-decreasing in omega (a, v_zero >= 0), so -- exactly like
    // determine_vel_omega() -- compare it at the range's endpoints first (no division, so a == 0
    // needs no special case) and only solve exactly for the in-between case. omega_min is checked
    // first so that an exact tie (both endpoint conditions true at once, e.g. a == v_zero ==
    // a_max == 0) resolves to the safer, more restrictive omega_min rather than omega_max.
    ScalarType omega_zero;
    if (((this->settings().omega_min() * this->settings().omega_min()) * a) +
            (static_cast<ScalarType>(2) * this->settings().omega_min() * v_zero) >=
        a_max) {
      omega_zero = this->settings().omega_min();  // already violated at the bottom of the range -- best effort
    } else if (((this->settings().omega_max() * this->settings().omega_max()) * a) +
                   (static_cast<ScalarType>(2) * this->settings().omega_max() * v_zero) <
               a_max) {
      omega_zero = this->settings().omega_max();  // unconstrained even at the top of the range
    } else {
      // Rationalized form of the usual (-v_zero + sqrt(v_zero^2 + a*a_max)) / a: multiplying by its
      // conjugate avoids subtracting two close values, and stays well-defined as a -> 0 (-> a_max /
      // (2*v_zero), matching the then-linear constraint) instead of needing a separate branch for it.
      omega_zero = a_max / (std::sqrt((v_zero * v_zero) + (a * a_max)) + v_zero);
    }

    // x''(t*) = -(omega/e) * x'(t_a0), and the velocity limiter already keeps |x'(t_a0)| ~= v_max, so
    // bound the interior extremum directly against a_max: omega <= e * a_max / v_max. Rearranged as
    // omega * v_max <= e * a_max (multiplying both sides by v_max, valid since v_max >= 0) to
    // sidestep v_max == 0 the same way, rather than special-casing it. omega_min is checked first
    // for the same tie-break reason as omega_zero above.
    const ScalarType a_decision = std::numbers::e_v<ScalarType> * a_max;
    ScalarType omega_interior;
    if (this->settings().omega_min() * v_max >= a_decision) {
      omega_interior = this->settings().omega_min();
    } else if (this->settings().omega_max() * v_max < a_decision) {
      omega_interior = this->settings().omega_max();
    } else {
      omega_interior = a_decision / v_max;
    }

    // Both branches above are constructed to already land in [omega_min, omega_max]
    return std::min(omega_zero, omega_interior);
  }

private:
  TimestampType start_time_;
  PoseType goal_;
  TwistType A_, B_;
  ScalarType omega_;
};

}  // namespace duatic::trajectory
