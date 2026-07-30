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
 * A KinematicLimits provider suitable for KinematicTrajectoryExponentialApproach: in addition to
 * the linear/angular velocity limits, this trajectory needs a convergence-rate (omega) range.
 */
template <typename T>
concept KinematicLimitsExponentialApproach = KinematicLimits<T> && requires(const T& const_variable) {
  { const_variable.omega_min() } -> std::same_as<typename T::ScalarType>;
  { const_variable.omega_max() } -> std::same_as<typename T::ScalarType>;
};

/*
 * Plain-data default implementation of KinematicLimitsExponentialApproach: public members, no
 * validation. Reuses KinematicLimitsDefault for the linear/angular velocity limits and adds the
 * omega range on top.
 */
template <typename ScalarT, KinematicLimits BaseT = KinematicLimitsDefault<ScalarT>>
struct KinematicLimitsExponentialApproachDefault : public BaseT
{
  using ScalarType = typename BaseT::ScalarType;

  ScalarType omega_min_{ 1.0E-6 };
  ScalarType omega_max_{ 1.0E3 };

  /*
   * Move-assigns the BaseT part from anything BaseT itself can be move-assigned from (BaseT
   * directly, or -- via BaseT's own operator=, e.g. a custom one like
   * CartesianPoseController::KinematicLimitsParams::operator=(Params&&) -- one of BaseT's own
   * ancestors). Constrained to rvalues only (BaseAssignableT deduces to a non-reference type only
   * when called with an rvalue) so this never competes with the implicitly-declared copy-assignment
   * operator for lvalue arguments. Excluded for Self itself, since std::assignable_from<BaseT&,
   * Self&&> is false unless BaseT happens to know about Self -- leaving the implicitly-declared
   * move-assignment operator as the sole candidate for that case.
   */
  template <typename BaseAssignableT>
    requires(!std::is_reference_v<BaseAssignableT>) && std::assignable_from<BaseT&, BaseAssignableT&&>
  inline KinematicLimitsExponentialApproachDefault& operator=(BaseAssignableT&& rhs)
  {
    static_cast<BaseT&>(*this) = std::forward<BaseAssignableT>(rhs);
    return *this;
  }

  inline ScalarType omega_min() const
  {
    return omega_min_;
  }

  inline ScalarType omega_max() const
  {
    return omega_max_;
  }

  /*
   * Sets the minimum and maximum convergence rate (omega) used to satisfy the velocity limits.
   * omega_min must be strictly positive: omega == 0 degenerates the trajectory into a straight
   * line that never reaches the goal (see the attached mathematical proof). Returns false
   * (without changing any state) if omega_min <= 0 or omega_max < omega_min.
   */
  inline bool set_omega_limits(const ScalarType omega_min, const ScalarType omega_max)
  {
    if ((omega_min <= static_cast<ScalarType>(0)) || (omega_max < omega_min)) {
      return false;
    }
    omega_min_ = omega_min;
    omega_max_ = omega_max;
    return true;
  }
};

static_assert(KinematicLimitsExponentialApproach<KinematicLimitsExponentialApproachDefault<double>>,  // line break
              "KinematicLimitsExponentialApproachDefault must satisfy the KinematicLimitsExponentialApproach concept.");

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
 * omega is the tracking the convergence speed: higher values converge faster.
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
template <typename ScalarT, geometry::KinematicOrder EvalOrderDepth, typename TimestampT,
          template <typename, geometry::KinematicOrder> typename KinematicVariableT,
          KinematicLimitsExponentialApproach KinematicLimitsT = KinematicLimitsExponentialApproachDefault<ScalarT>>
  requires std::convertible_to<typename KinematicLimitsT::ScalarType, ScalarT>
class KinematicTrajectoryExponentialApproach
{
public:
  using ScalarType = ScalarT;
  static constexpr geometry::KinematicOrder eval_order_depth = EvalOrderDepth;
  using TimestampType = TimestampT;
  using KinematicLimitsType = KinematicLimitsT;

  using Self = KinematicTrajectoryExponentialApproach<ScalarType, eval_order_depth, TimestampType, KinematicVariableT,
                                                      KinematicLimitsType>;

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

  /*
   * limits is the sole source of truth for the velocity and omega bounds (see determine_omega()
   * below); it is kept as a shared_ptr so it may be shared with (and updated by) other owners,
   * but this trajectory only ever reads from it. omega_ is seeded from limits->omega_max()
   * because, with a freshly-constructed zero offset A_, determine_omega() would compute exactly
   * that value anyway (see the attached mathematical proof).
   */
  inline explicit KinematicTrajectoryExponentialApproach(std::shared_ptr<KinematicLimitsType> shared_limits)
    : limits(std::move(shared_limits))
  {
    assert(limits != nullptr && "Given shared limits don't exist");
    omega_ = limits->omega_min();
  }

  /*
   * Available only if KinematicLimitsType is default-constructible: creates a privately-owned
   * limits object (not shared with any other owner) using its defaults.
   */
  inline KinematicTrajectoryExponentialApproach()
    requires std::default_initializable<KinematicLimitsType>
    : KinematicTrajectoryExponentialApproach(std::make_shared<KinematicLimitsType>())
  {
  }

  inline void update(const KinematicUpdateState& in_state, const PoseType& update_pose)
  {
    std::cout << "\n Replan trajectory with initial state:\n" << in_state << "\n";
    start_time_ = in_state.time();
    goal_ = update_pose;
    A_ = in_state.pose() - goal_;
    omega_ = determine_omega(in_state.twist());
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

public:
  std::shared_ptr<KinematicLimitsType> limits;

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
