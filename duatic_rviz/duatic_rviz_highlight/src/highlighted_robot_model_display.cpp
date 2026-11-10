/*
 * Copyright 2026 Duatic AG
 * Duatic Commercial License 1.0 (DCL-1)
 */

#include "duatic_rviz_highlight/highlighted_robot_model_display.hpp"

#include <algorithm>
#include <exception>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_default_plugins/robot/robot.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>

namespace duatic::rviz_highlight
{

void HighlightedRobotModelDisplay::onInitialize()
{
  RobotModelDisplay::onInitialize();

  // Common Rviz plugin pattern: get the RViz node from the context
  const auto abstraction = context_->getRosNodeAbstraction().lock();
  if (!abstraction) {
    return;
  }
  node_ = abstraction->get_raw_node();

  // Create the central parameter listener and the plugin loader
  param_listener_ = std::make_unique<highlighted_robot_model_display::ParamListener>(node_);
  loader_ = std::make_unique<pluginlib::ClassLoader<HighlightPluginBase>>("duatic_rviz_highlight",
                                                                          "duatic::rviz_highlight::HighlightPlugin");
}

void HighlightedRobotModelDisplay::load_urdf_from_string(const std::string& robot_description)
{
  RobotModelDisplay::load_urdf_from_string(robot_description);
  syncDescription(true);
}

void HighlightedRobotModelDisplay::load_urdf_from_file(const std::string& filepath)
{
  // The base returns early without touching the robot if the file did not change
  const std::string previous = robot_description_;
  RobotModelDisplay::load_urdf_from_file(filepath);
  syncDescription(robot_description_ != previous);
}

void HighlightedRobotModelDisplay::syncDescription(bool reloaded)
{
  // The base clears robot_description_ together with the robot if the description is empty or fails to parse
  if (robot_description_.empty()) {
    urdf_xml_.clear();
    current_link_colors_.clear();
    return;
  }
  if (!reloaded) {
    return;
  }

  // reset (safely), as the links were recreated
  urdf_xml_ = robot_description_;
  current_link_colors_.clear();
  std::vector<std::string> failed;
  for (const auto& [name, highlight] : highlights_) {
    try {
      highlight->onRobotDescription(urdf_xml_);
    } catch (const std::exception& error) {
      RCLCPP_ERROR(node_->get_logger(), "%s->onRobotDescription() failed: %s", name.c_str(), error.what());
      failed.push_back(name);
    }
  }
  unloadHighlights(failed);
}

void HighlightedRobotModelDisplay::update(float wall_dt, float ros_dt)
{
  RobotModelDisplay::update(wall_dt, ros_dt);

  if (!loader_ || !param_listener_) {
    return;
  }

  // Cheaply update the parameters of the display and the highlights
  bool highlights_changed = false;
  if (param_listener_->try_update_params(params_)) {
    highlights_changed |= highlightsChanged();
  }
  for (const auto& [name, highlight] : highlights_) {
    highlights_changed |= highlight->parametersChanged();  // the priority can change
  }

  // Ascending priority, or the order in `highlight.enabled` on equal priority
  if (highlights_changed) {
    const auto selection_index = [this](const std::string& name) {
      return std::find(created_selection_.begin(), created_selection_.end(), name) - created_selection_.begin();
    };
    std::sort(highlights_.begin(), highlights_.end(), [&](const auto& a, const auto& b) {
      return std::make_pair(a.second->priority(), selection_index(a.first)) <
             std::make_pair(b.second->priority(), selection_index(b.first));
    });
  }

  // Update desired link colors safely
  LinkColors colors;
  std::vector<std::string> failed;
  for (const auto& [name, highlight] : highlights_) {
    try {
      highlight->update(colors);
    } catch (const std::exception& error) {
      RCLCPP_ERROR(node_->get_logger(), "%s->update() failed: %s", name.c_str(), error.what());
      failed.push_back(name);
    }
  }
  unloadHighlights(failed);

  // Update link colors
  updateLinkColors(colors);
}

void HighlightedRobotModelDisplay::unloadHighlights(const std::vector<std::string>& names)
{
  for (const auto& name : names) {
    RCLCPP_WARN(node_->get_logger(), "Unloading the '%s' RViz highlight", name.c_str());
    std::erase_if(highlights_, [&name](const auto& highlight) { return highlight.first == name; });
    std::erase(created_selection_, name);
  }
}

bool HighlightedRobotModelDisplay::highlightsChanged()
{
  // Get sanitized list of desired highlights (non-empty, unique)
  std::vector<std::string> selection;
  for (const auto& name : params_.highlight.enabled) {
    if (!name.empty() && std::find(selection.begin(), selection.end(), name) == selection.end()) {
      selection.push_back(name);
    }
  }
  if (selection == created_selection_) return false;

  // Update highlights
  std::vector<std::pair<std::string, std::shared_ptr<HighlightPluginBase>>> next;
  for (const auto& name : selection) {
    const auto has_name = [&name](const auto& highlight) { return highlight.first == name; };
    const auto existing = std::find_if(highlights_.begin(), highlights_.end(), has_name);
    if (existing != highlights_.end()) {
      // Keep the existing highlight, it is still wanted
      next.push_back(std::move(*existing));
      continue;
    }
    try {
      // Create a new highlight safely
      auto highlight = loader_->createSharedInstance(name);
      highlight->initialize(node_, "highlight." + name + ".");
      if (!urdf_xml_.empty()) {
        highlight->onRobotDescription(urdf_xml_);
      }
      next.emplace_back(name, std::move(highlight));
    } catch (const std::exception& error) {
      RCLCPP_ERROR(node_->get_logger(), "Could not load the RViz highlight '%s': %s", name.c_str(), error.what());
    }
  }

  // The selection that is active, which excludes the highlights that failed to load
  created_selection_.clear();
  for (const auto& [name, highlight] : next) {
    created_selection_.push_back(name);
  }

  // Replace the highlights, simultaneously destroying active but unwanted ones
  highlights_ = std::move(next);
  return true;
}

void HighlightedRobotModelDisplay::updateLinkColors(const LinkColors& target_link_colors)
{
  if (!robot_) {
    return;
  }

  for (const auto& [name, link] : robot_->getLinks()) {
    const auto target_color = target_link_colors.find(name);
    const bool has_target_color = target_color != target_link_colors.end();
    const auto current_color = current_link_colors_.find(name);
    const bool has_current_color = current_color != current_link_colors_.end();
    if (!has_target_color && has_current_color) {
      // Not wanted anymore: restore the link
      link->unsetColor();
      current_link_colors_.erase(current_color);
    } else if (has_target_color && (!has_current_color || current_color->second != target_color->second)) {
      // New or changed: Re-color the link.
      const Color& color = target_color->second;
      link->setColor(color.red / 255.0F, color.green / 255.0F, color.blue / 255.0F);
      current_link_colors_[name] = color;
    }
  }
}

}  // namespace duatic::rviz_highlight

PLUGINLIB_EXPORT_CLASS(duatic::rviz_highlight::HighlightedRobotModelDisplay, rviz_common::Display)
