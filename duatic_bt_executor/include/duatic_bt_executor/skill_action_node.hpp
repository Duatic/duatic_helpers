#pragma once

#include <chrono>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "duatic_skills_msgs/action/skill_execution.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace duatic_bt
{

/**
 * Generic BT leaf node that calls a SkillExecution action server.
 *
 * XML usage:
 *   <SkillAction skill_id="animation_playback"
 *                action_server="/skills/animation_playback"
 *                parameters_json='{"file_name":"wave.csv","speed":1.0}' />
 *
 * All three attributes are read from BT ports (can be blackboard variables).
 */
class SkillActionNode : public BT::StatefulActionNode
{
public:
  using SkillExecution = duatic_skills_msgs::action::SkillExecution;
  using GoalHandle = rclcpp_action::ClientGoalHandle<SkillExecution>;

  SkillActionNode(const std::string & name, const BT::NodeConfig & config,
                  rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("skill_id", "", "Skill identifier"),
      BT::InputPort<std::string>("action_server", "", "Action server name"),
      BT::InputPort<std::string>("parameters_json", "{}", "JSON parameters"),
    };
  }

  BT::NodeStatus onStart() override
  {
    std::string skill_id, server_name, params_json;
    if (!getInput("skill_id", skill_id) || skill_id.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "SkillAction: missing skill_id");
      return BT::NodeStatus::FAILURE;
    }
    if (!getInput("action_server", server_name) || server_name.empty()) {
      server_name = "/skills/" + skill_id;
    }
    getInput("parameters_json", params_json);

    // The UI strips outer {} from JSON to prevent BT.CPP treating it as a
    // blackboard variable reference.  Re-wrap so the skill gets valid JSON.
    if (!params_json.empty() && params_json.front() != '{') {
      params_json = "{" + params_json + "}";
    }

    // Create action client on demand
    client_ = rclcpp_action::create_client<SkillExecution>(node_, server_name);

    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(), "Action server '%s' not available", server_name.c_str());
      return BT::NodeStatus::FAILURE;
    }

    auto goal = SkillExecution::Goal();
    goal.skill_id = skill_id;
    goal.parameters_json = params_json;

    auto send_options = rclcpp_action::Client<SkillExecution>::SendGoalOptions();
    send_options.feedback_callback =
      [this](GoalHandle::SharedPtr, const std::shared_ptr<const SkillExecution::Feedback> fb) {
        RCLCPP_DEBUG(node_->get_logger(), "Skill feedback: %s (%.0f%%)",
                     fb->phase.c_str(), fb->progress * 100.0);
      };

    auto future = client_->async_send_goal(goal, send_options);

    // Wait briefly for goal acceptance
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to send goal to '%s'", server_name.c_str());
      return BT::NodeStatus::FAILURE;
    }

    goal_handle_ = future.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(node_->get_logger(), "Goal rejected by '%s'", server_name.c_str());
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!goal_handle_) {
      return BT::NodeStatus::FAILURE;
    }

    auto result_future = client_->async_get_result(goal_handle_);

    if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::milliseconds(100)) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto wrapped = result_future.get();
      if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result->success) {
        RCLCPP_INFO(node_->get_logger(), "Skill succeeded: %s", wrapped.result->message.c_str());
        return BT::NodeStatus::SUCCESS;
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Skill failed: %s", wrapped.result->message.c_str());
        return BT::NodeStatus::FAILURE;
      }
    }

    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    if (goal_handle_ && client_) {
      client_->async_cancel_goal(goal_handle_);
      RCLCPP_INFO(node_->get_logger(), "Skill action cancelled.");
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<SkillExecution>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
};

}  // namespace duatic_bt
