/*
 * Copyright 2026 Duatic AG
 * Duatic Commercial License 1.0 (DCL-1)
 */

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_default_plugins/displays/robot_model/robot_model_display.hpp>

#include "duatic_rviz_highlight/plugin/highlight_plugin_base.hpp"
#include "duatic_rviz_highlight/highlighted_robot_model_display_parameters.hpp"

namespace duatic::rviz_highlight
{

/**
 * @brief The RobotModel display, plus highlights that recolor its links.
 *
 * Use as `duatic_rviz_highlight/HighlightedRobotModel` instead of `rviz_default_plugins/RobotModel`, and select the
 * highlights with the parameter `highlight.enabled` of the RViz node.
 */
class HighlightedRobotModelDisplay : public rviz_default_plugins::displays::RobotModelDisplay
{
public:
  HighlightedRobotModelDisplay() = default;
  ~HighlightedRobotModelDisplay() override = default;

  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;

protected:
  // robot description source is topic
  void load_urdf_from_string(const std::string& robot_description) override;
  // robot description source is file
  void load_urdf_from_file(const std::string& filepath) override;

private:
  /**
   * @brief Pass the robot description of the base display to the highlights.
   * @param reloaded False if the base display kept its robot as it was.
   */
  void syncDescription(bool reloaded);
  /**
   * @brief Unload highlights, e.g. because they threw. They stay unloaded until `highlight.enabled` changes.
   * @param names The names of the highlights to unload.
   */
  void unloadHighlights(const std::vector<std::string>& names);
  /**
   * @brief Recreate the highlights if `highlight.enabled` changed.
   * @return True if the highlights were recreated, false otherwise.
   */
  bool highlightsChanged();
  /**
   * @brief Recolor the links, and restore the ones that are not recolored anymore.
   * @param wanted The colors the links should have.
   */
  void updateLinkColors(const LinkColors& wanted);

  // The ROS node of RViz, shared with the highlights.
  rclcpp::Node::SharedPtr node_;

  // The parameters of the display and their latest values.
  std::unique_ptr<highlighted_robot_model_display::ParamListener> param_listener_;
  highlighted_robot_model_display::Params params_;

  // Loads the highlight plugins. Declared before `highlights_`, so that those are destroyed first
  std::unique_ptr<pluginlib::ClassLoader<HighlightPluginBase>> loader_;
  // The active highlights as (name, instance), in the order they run this frame (sorted by priority).
  std::vector<std::pair<std::string, std::shared_ptr<HighlightPluginBase>>> highlights_;
  // The names of the active highlights, in the order of `highlight.enabled` (without empty names and duplicates).
  std::vector<std::string> created_selection_;

  // The robot description last given to the highlights, empty if none.
  std::string urdf_xml_;
  // The colors set on the links right now.
  LinkColors current_link_colors_;
};

}  // namespace duatic::rviz_highlight
