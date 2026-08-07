#pragma once

#include <cassert>
#include <cmath>
#include <concepts>
#include <limits>
#include <memory>
#include <numbers>
#include <type_traits>
#include <utility>

#include <duatic_geometry/geometry.hpp>
#include <duatic_trajectory/kinematic_trajectory.hpp>

namespace duatic::trajectory
{

/*
 * A KinematicTrajectorySettings provider suitable for KinematicTrajectoryExponentialApproach: in addition to
 * the linear/angular velocity limits, this trajectory needs a convergence-rate (omega) range.
 */
template <typename T>
concept KinematicTrajectorySettingsExponentialApproach =
    KinematicTrajectorySettings<T> && requires(const T& const_variable) {
      { const_variable.omega_min() } -> std::same_as<typename T::ScalarType>;
      { const_variable.omega_max() } -> std::same_as<typename T::ScalarType>;
    };

/*
 * Plain-data default implementation of KinematicTrajectorySettingsExponentialApproach: public members, no
 * validation. Reuses KinematicTrajectorySettingsDefault for the linear/angular velocity limits and adds the
 * omega range on top.
 */
template <typename ScalarT, KinematicTrajectorySettings BaseT = KinematicTrajectorySettingsDefault<ScalarT>>
struct KinematicTrajectorySettingsExponentialApproachDefault : public BaseT
{
  using ScalarType = typename BaseT::ScalarType;

  /*
   * clamping omega (especially by omega_min) to a small positive value is necessary to avoid degenerating the
   * trajectory into a straight line that never reaches the goal (see the attached mathematical proof). However, by
   * clamping omega upward, the given limits may be violated, so this is the largest allowed value for omega_min, it
   * does not make sense to set omega_min any higher than this.
   */
  static constexpr ScalarType omega_min_upper_bound{ 1.0E-6 };
  static constexpr ScalarType omega_max_lower_bound{ 1.0E3 };
  static_assert(omega_min_upper_bound > 0.0, "omega_min_upper_bound must be strictly positive");
  static_assert(omega_max_lower_bound >= omega_min_upper_bound,  //
                "omega_max_lower_bound must be >= omega_min_upper_bound");
  ScalarType omega_min_{ omega_min_upper_bound };
  ScalarType omega_max_{ omega_max_lower_bound };

  /*
   * Move-assigns the BaseT part from anything BaseT itself can be move-assigned from (BaseT
   * directly, or -- via BaseT's own operator=, e.g. a custom one like
   * CartesianPoseController::KinematicTrajectorySettingsParams::operator=(Params&&) -- one of BaseT's own
   * ancestors). Constrained to rvalues only (BaseAssignableT deduces to a non-reference type only
   * when called with an rvalue) so this never competes with the implicitly-declared copy-assignment
   * operator for lvalue arguments. Excluded for Self itself, since std::assignable_from<BaseT&,
   * Self&&> is false unless BaseT happens to know about Self -- leaving the implicitly-declared
   * move-assignment operator as the sole candidate for that case.
   */
  template <typename BaseAssignableT>
    requires(!std::is_reference_v<BaseAssignableT>) && std::assignable_from<BaseT&, BaseAssignableT&&>
  inline KinematicTrajectorySettingsExponentialApproachDefault& operator=(BaseAssignableT&& rhs)
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
    if ((omega_min <= static_cast<ScalarType>(0.0)) || (omega_min > omega_min_upper_bound) || (omega_max < omega_min) ||
        (omega_max < omega_max_lower_bound)) {
      return false;
    }
    omega_min_ = omega_min;
    omega_max_ = omega_max;
    return true;
  }
};

static_assert(KinematicTrajectorySettingsExponentialApproach<
                  KinematicTrajectorySettingsExponentialApproachDefault<double>>,  // line break
              "KinematicTrajectorySettingsExponentialApproachDefault must satisfy the "
              "KinematicTrajectorySettingsExponentialApproach concept.");

// generic trajectory template forward declaration
template <typename ScalarT, typename TimestampT, geometry::KinematicOrder ContinuityOrder,
          template <typename, geometry::KinematicOrder> typename KinematicVariableT,
          KinematicTrajectorySettingsExponentialApproach KinematicTrajectorySettingsT =
              KinematicTrajectorySettingsExponentialApproachDefault<ScalarT>>
  requires std::convertible_to<typename KinematicTrajectorySettingsT::ScalarType, ScalarT>
class KinematicTrajectoryExponentialApproach;

}  // namespace duatic::trajectory

// include template concretizations for the individual continuity levels
#include <duatic_trajectory/kinematic_trajectory_exponential_approach_C1.hpp>
#include <duatic_trajectory/kinematic_trajectory_exponential_approach_C2.hpp>