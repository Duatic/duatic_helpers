/*
 * Copyright 2026 Duatic AG
 * Duatic Commercial License 1.0 (DCL-1)
 */

#pragma once

#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "duatic_rviz_highlight/plugin/highlight_plugin_base.hpp"

namespace duatic::rviz_highlight
{

/**
 * @brief A highlight that recolors links of the robot model. Derive from this class.
 *
 * Highlights are plugins, provided by the package that implements them. The `name` attribute of its class in the
 * `plugins_description.xml` file is what goes into `highlight.enabled`. Export it with
 * `DUATIC_RVIZ_HIGHLIGHT_EXPORT_CLASS(MyHighlight)`.
 *
 * Its parameters, on the RViz node under the prefix `highlight.<name>.`, are declared by a generate_parameter_library
 * `ParamListener` and must contain an int `priority`. They are refreshed once per frame by the display, before
 * update(): read them from `params_`.
 *
 * All functions run on the RViz thread, except for those on threads or executors that a highlight creates itself.
 *
 * @tparam ParamListenerT The generated `ParamListener` of the highlight. Its `Params` is deduced from it.
 */
template <HighlightParamListener ParamListenerT>
class HighlightPlugin : public HighlightPluginBase
{
  using ParamsT = decltype(std::declval<const ParamListenerT&>().get_params());

public:
  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_prefix) final
  {
    node_ = node;
    param_listener_ = std::make_unique<ParamListenerT>(node_, parameter_prefix);
    params_ = param_listener_->get_params();
    onInitialize();
    onParametersChanged();
  }

  bool parametersChanged() final
  {
    const bool changed = param_listener_->try_update_params(params_);
    if (changed) {
      onParametersChanged();
    }
    return changed;
  }

  int priority() final
  {
    return static_cast<int>(params_.priority);
  }

protected:
  /** @brief (Optional) Create subscriptions etc. here. Called in initialize(), with `node_` and `params_` set. */
  virtual void onInitialize()
  {
  }

  /**
   * @brief (Optional) React to new parameter values in `params_`, e.g. to validate them.
   *
   * Called at the end of initialize() for the initial values, and by parametersChanged() whenever they changed. Thus
   * always after onInitialize().
   */
  virtual void onParametersChanged()
  {
  }

  rclcpp::Node::SharedPtr node_;

  /** @brief The current parameter values. The display refreshes them once per frame, before update(). */
  ParamsT params_;

private:
  std::unique_ptr<ParamListenerT> param_listener_;
};

}  // namespace duatic::rviz_highlight
