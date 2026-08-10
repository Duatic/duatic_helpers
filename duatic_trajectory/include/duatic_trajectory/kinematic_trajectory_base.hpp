#pragma once

#include <cassert>
#include <concepts>
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
