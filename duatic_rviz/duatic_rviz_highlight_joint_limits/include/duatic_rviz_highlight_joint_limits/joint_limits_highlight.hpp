/*
 * Copyright 2026 Duatic AG
 * Duatic Commercial License 1.0 (DCL-1)
 */

#pragma once

#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "duatic_rviz_highlight/plugin/highlight_plugin.hpp"
#include "duatic_rviz_highlight_joint_limits/moving_parts.hpp"
#include "duatic_rviz_highlight_joint_limits/joint_limits_highlight_parameters.hpp"

namespace duatic::rviz_highlight
{

/**
 * @brief Highlights the links behind every joint that is close to a limit.
 *
 * The links that the joint moves are shown orange when the joint is close to one of its limits, and red when it is
 * closer. These are its child link and everything rigidly attached to it through fixed joints, up to the next movable
 * joint. Only revolute joints are currently monitored (but this could be easily extended).
 *
 * @note No mutex guards `positions_`: RViz spins the executor of its node (which runs the joint state callback) from the
 * same thread that calls update() (see `VisualizationManager::onUpdate()`), so they never run concurrently.
 */
class JointLimitsHighlight : public HighlightPlugin<joint_limits_highlight::ParamListener>
{
public:
  void onRobotDescription(const std::string& urdf_xml) override;
  void update(LinkColors& colors) override;

protected:
  void onInitialize() override;
  void onParametersChanged() override
  {
    validateMargins();
  }

private:
  Margins margins() const;

  /** @brief Warn about margins that do not make sense for the parameters and monitored joints. */
  void validateMargins() const;

  std::map<std::string, MovingPart> parts_;  // by joint name
  std::map<std::string, double> positions_;  // by joint name

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
};

}  // namespace duatic::rviz_highlight
