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

#include <concepts>
#include <memory>
#include <utility>

#include <duatic_geometry/geometry.hpp>
#include <duatic_trajectory/kinematic_trajectory.hpp>
#include <duatic_trajectory/kinematic_trajectory_base.hpp>

namespace duatic::trajectory
{

/*
 * CRTP base factoring out the functionality shared by every "pose target" kinematic trajectory --
 * i.e. every trajectory whose TrajectoryDescriptionType is a Pose, and which (re-)plans itself
 * towards that pose from some initial kinematic state handed to calculate() (currently: the C1 and
 * C2 exponential-approach trajectories in kinematic_trajectory_exponential_approach_C1.hpp /
 * kinematic_trajectory_exponential_approach_C2.hpp).
 *
 * What actually differs between continuity orders is only:
 *   - UpdateStateType             (the kinematic depth of the initial state calculate() needs)
 *   - calculate(const UpdateStateType&, const TrajectoryDescriptionType&)
 *                                 (how the coefficients of the concrete trajectory shape are derived)
 *   - evaluate<Order>(const TimestampType&, KinematicState<Order>&) const
 *                                 (how the concrete trajectory shape is evaluated at a given time)
 *
 * Everything expressible purely in terms of those three -- calculate_neutral(), update_from(),
 * update(), update_neutral(), and the return-by-value evaluate<Order>() -- lives here instead, via
 * CRTP: DerivedT is expected to publicly derive from
 * KinematicTrajectoryPoseTargetBase<DerivedT, ScalarT, TimestampT, ContinuityOrder, KinematicVariableT,
 * KinematicTrajectorySettingsT> and provide DerivedT::UpdateStateType plus the two customization
 * points above as public members.
 *
 * Since DerivedT necessarily declares its own evaluate<Order>(time, out_state) overload, it will
 * hide this base's evaluate<Order>(time) overload (member name hiding applies across the whole
 * overload set, not just matching signatures) -- DerivedT must re-expose it with a
 * "using KinematicTrajectoryPoseTargetBase::evaluate;" declaration.
 *
 * Construction is deliberately NOT factored out here, even though C1 and C2 currently share an
 * identical "explicit ctor taking shared_settings, plus a default ctor requiring
 * default_initializable settings that delegates to it" shape: a base class constructor cannot
 * itself delegate to a *derived* class' constructor (that direction of delegation does not exist in
 * C++), and DerivedT's own explicit ctor also needs to initialize DerivedT-specific state (e.g. the
 * initial omega_) that this base has no business knowing about.
 */
template <typename DerivedT, typename ScalarT, typename TimestampT, geometry::KinematicOrder ContinuityOrder,
          template <typename, geometry::KinematicOrder> typename KinematicVariableT,
          typename KinematicTrajectorySettingsT>
  requires is_kinematic_trajectory_settings_v<KinematicTrajectorySettingsT>
class KinematicTrajectoryPoseTargetBase
  : public KinematicTrajectoryBase<KinematicTrajectorySettingsT, ContinuityOrder>
{
public:
  using ScalarType = ScalarT;
  using TimestampType = TimestampT;
  using KinematicTrajectorySettingsType = KinematicTrajectorySettingsT;

  using KinematicTrajectoryBaseType = KinematicTrajectoryBase<KinematicTrajectorySettingsType, ContinuityOrder>;
  using KinematicTrajectoryBaseType::KinematicTrajectoryBaseType;  // inherit both of its constructors
  using KinematicTrajectoryBaseType::continuity_order;
  using KinematicTrajectoryBaseType::settings;

  // DerivedT itself -- what every "Self" alias throughout the C1/C2 headers refers to.
  using Self = DerivedT;

  template <geometry::KinematicOrder Order>
  using KinematicVariable = KinematicVariableT<ScalarType, Order>;
  template <geometry::KinematicOrder OrderDepth>
  using KinematicState = geometry::KinematicState<ScalarType, OrderDepth, KinematicVariableT>;

  using PoseType = KinematicVariable<geometry::KinematicOrder::Pose>;
  using TwistType = KinematicVariable<geometry::KinematicOrder::Twist>;

  using TrajectoryDescriptionType = PoseType;

  /*
   * calculate() starting from a *neutral* (motionless) state, i.e. resting exactly at
   * in_update_state's own kinematic data forever until this call -- the target is the only new
   * information this needs on top of in_update_state.
   *
   * Templated on UpdateStateT (rather than spelled out as "const typename DerivedT::UpdateStateType&")
   * purely so its declaration doesn't need to look up UpdateStateType inside DerivedT: DerivedT is
   * still incomplete at the point this class is instantiated as DerivedT's own base, so any nested
   * DerivedT:: name can only appear where its resolution is deferred to (function-template/body)
   * instantiation time -- by which point DerivedT is complete -- never in a non-template member's own
   * signature.
   */
  template <typename UpdateStateT>
  inline void calculate_neutral(const UpdateStateT& in_update_state)
  {
    derived().calculate(in_update_state, in_update_state.data().pose());
  }

  /*
   * Replans starting from the 'other' trajectory's predicted state at in_timestamp (rather than an
   * externally supplied UpdateStateType), so the caller only needs to provide the new target.
   */
  inline void update_from(const Self& other, const TimestampType& in_timestamp,
                          const TrajectoryDescriptionType& in_description)
  {
    using UpdateStateType = typename DerivedT::UpdateStateType;
    // there are no future trajectory data existing to be copied
    derived().calculate(
        UpdateStateType(in_timestamp,
                        other.template evaluate<UpdateStateType::DataType::kinematic_order_depth>(in_timestamp)),
        in_description);
  }

  /*
   * Replans starting from this trajectory's own predicted state at in_timestamp (rather than an
   * externally supplied UpdateStateType), so the caller only needs to provide the new target.
   */
  inline void update(const TimestampType& in_timestamp, const TrajectoryDescriptionType& in_description)
  {
    update_from(derived(), in_timestamp, in_description);
  }

  inline void update_neutral(const TimestampType& in_timestamp)
  {
    using UpdateStateType = typename DerivedT::UpdateStateType;
    calculate_neutral(
        UpdateStateType(in_timestamp, evaluate<UpdateStateType::DataType::kinematic_order_depth>(in_timestamp)));
  }

  template <geometry::KinematicOrder Order>
  inline KinematicState<Order> evaluate(const TimestampType& time) const
  {
    KinematicState<Order> out_state;
    derived().template evaluate<Order>(time, out_state);
    return out_state;
  }

private:
  inline DerivedT& derived()
  {
    return static_cast<DerivedT&>(*this);
  }

  inline const DerivedT& derived() const
  {
    return static_cast<const DerivedT&>(*this);
  }
};

}  // namespace duatic::trajectory
