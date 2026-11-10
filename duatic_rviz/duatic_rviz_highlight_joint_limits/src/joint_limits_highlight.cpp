/*
 * Copyright 2026 Duatic AG
 * Duatic Commercial License 1.0 (DCL-1)
 */

#include "duatic_rviz_highlight_joint_limits/joint_limits_highlight.hpp"

#include <utility>

#include <pluginlib/class_list_macros.hpp>

namespace duatic::rviz_highlight
{

void JointLimitsHighlight::onInitialize()
{
  joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          positions_[msg->name[i]] = msg->position[i];
        }
      });
}

void JointLimitsHighlight::onRobotDescription(const std::string& urdf_xml)
{
  auto parts = parseMovingParts(urdf_xml);
  RCLCPP_INFO(node_->get_logger(), "Monitoring %zu joints for their limits", parts.size());
  parts_ = std::move(parts);
  validateMargins();
}

Margins JointLimitsHighlight::margins() const
{
  const auto& p = params_;
  if (p.mode == "relative") {
    return { MarginMode::kRelative, p.margin.relative.warning, p.margin.relative.critical };
  }
  return { MarginMode::kAbsoluteDeg, p.margin.absolute_deg.warning, p.margin.absolute_deg.critical };
}

void JointLimitsHighlight::validateMargins() const
{
  const auto [mode, warning, critical] = margins();

  // Check that the warning/critical margins are consistent
  if (warning < critical) {
    RCLCPP_WARN(node_->get_logger(),
                "The warning margin (%g) is smaller than the critical margin (%g): no warning is shown", warning,
                critical);
  }

  // Warn about parts that are always highlighted
  if (mode != MarginMode::kAbsoluteDeg) {
    return;
  }
  std::string always_highlighted;
  for (const auto& [joint, part] : parts_) {
    const double range_deg = angles::to_degrees(part.limits.upper - part.limits.lower);
    if (2.0 * warning >= range_deg) {
      always_highlighted += (always_highlighted.empty() ? "" : ", ") + joint;
    }
  }
  if (!always_highlighted.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "The warning margin (%g deg) is at least half of the range of these joints, so they are always "
                "highlighted: %s",
                warning, always_highlighted.c_str());
  }
}

void JointLimitsHighlight::update(LinkColors& colors)
{
  const auto [mode, warning, critical] = margins();

  for (const auto& [joint, part] : parts_) {
    // Check whether we get the joint's state
    const auto position = positions_.find(joint);
    if (position == positions_.end()) continue;

    // Set desired color based on how close the joint is to a limit
    const auto severity = part.limits.severityAt(position->second, warning, critical, mode);
    if (const auto color = severityColor(severity)) {
      for (const auto& link : part.links) {
        colors[link] = *color;
      }
    }
  }
}

}  // namespace duatic::rviz_highlight

DUATIC_RVIZ_HIGHLIGHT_EXPORT_CLASS(duatic::rviz_highlight::JointLimitsHighlight)
