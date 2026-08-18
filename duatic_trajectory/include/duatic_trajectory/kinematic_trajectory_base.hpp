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

#include <cassert>
#include <memory>
#include <utility>

#include <duatic_geometry/kinematic_order.hpp>
#include <duatic_trajectory/kinematic_trajectory.hpp>

namespace duatic::trajectory
{

/*
 * Base factoring out what every KinematicTrajectory implementation needs regardless of its own
 * continuity order: ownership of the (possibly shared) settings object, and the continuity_order
 * constant the KinematicTrajectory concept requires.
 */
template <typename KinematicTrajectorySettingsT, geometry::KinematicOrder ContinuityOrder>
  requires is_kinematic_trajectory_settings_v<KinematicTrajectorySettingsT>
class KinematicTrajectoryBase
{
public:
  using KinematicTrajectorySettingsType = KinematicTrajectorySettingsT;

  // required by the KinematicTrajectory concept
  static constexpr geometry::KinematicOrder continuity_order = ContinuityOrder;

  /*
   * settings is the sole source of truth for whatever limits the derived trajectory reads out of
   * it; kept as a shared_ptr so it may be shared with (and updated by) other owners, but this base
   * -- like every derived trajectory -- only ever reads from it.
   */
  inline explicit KinematicTrajectoryBase(std::shared_ptr<KinematicTrajectorySettingsType> shared_settings)
    : settings_(std::move(shared_settings))
  {
    assert(settings_ != nullptr && "Given shared settings don't exist");
  }

  /*
   * Available only if KinematicTrajectorySettingsType is default-constructible: creates a privately-owned
   * settings object (not shared with any other owner) using its defaults.
   */
  inline KinematicTrajectoryBase()
    requires std::default_initializable<KinematicTrajectorySettingsType>
    : KinematicTrajectoryBase(std::make_shared<KinematicTrajectorySettingsType>())
  {
  }

  inline KinematicTrajectorySettingsType& settings()
  {
    return *settings_;
  }

  inline const KinematicTrajectorySettingsType& settings() const
  {
    return *settings_;
  }

private:
  std::shared_ptr<KinematicTrajectorySettingsType> settings_;
};

}  // namespace duatic::trajectory
