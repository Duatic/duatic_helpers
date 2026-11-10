/*
 * Copyright 2026 Duatic AG
 * Duatic Commercial License 1.0 (DCL-1)
 */

#pragma once

#include <concepts>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "duatic_rviz_highlight/common/color.hpp"

namespace duatic::rviz_highlight
{

/** @brief A generated `ParamListener` whose parameters contain an int `priority`. */
template <typename T>
concept HighlightParamListener = requires(const T& listener) {
  { listener.get_params().priority } -> std::convertible_to<int>;
};  // NOLINT(readability/braces)

template <HighlightParamListener ParamListenerT>
class HighlightPlugin;

/**
 * @brief Non-template base class for use in pluginlib.
 *
 * Cannot be derived from directly - derive from HighlightPlugin instead.
 */
class HighlightPluginBase
{
public:
  virtual ~HighlightPluginBase() = default;

  /**
   * @brief Called once after the highlight is created.
   * @param node The ROS node of RViz.
   * @param parameter_prefix Prefix of the highlight's parameters on the node, with the trailing dot.
   */
  virtual void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_prefix) = 0;

  /**
   * @brief Bring the parameters of the highlight up to date. Called every frame by the display, before update().
   * @return True if a parameter changed.
   */
  virtual bool parametersChanged() = 0;

  /**
   * @brief Priority decides the order of update() calls. Higher priority (or later one with equal priority) is called later, thus overrules.
   * @return The priority of the highlight.
   */
  virtual int priority() = 0;

  /**
   * @brief (Optional) Handle a new robot description. Called once after initialize()
   * @param urdf_xml The URDF of the robot.
   * @warning If this throws, the display logs the error and unloads the highlight.
   */
  virtual void onRobotDescription(const std::string& /*urdf_xml*/)
  {
  }

  /**
   * @brief Update the desired link colors. Called every frame.
   * @param colors Add the links to recolor to it.
   * @warning If this throws, the display logs the error and unloads the highlight.
   */
  virtual void update(LinkColors& colors) = 0;

private:
  // Private so that this class cannot be derived from directly: use HighlightPlugin instead
  HighlightPluginBase() = default;

  template <HighlightParamListener ParamListenerT>
  friend class HighlightPlugin;
};

/**
 * @brief Export a highlight plugin, so the display can load it.
 * @param Highlight The class of the highlight.
 */
#define DUATIC_RVIZ_HIGHLIGHT_EXPORT_CLASS(Highlight) \
  PLUGINLIB_EXPORT_CLASS(Highlight, duatic::rviz_highlight::HighlightPluginBase)

}  // namespace duatic::rviz_highlight
