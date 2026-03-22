// Copyright 2026 Duatic AG — BSD-3-Clause (see LICENSE)
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "duatic_bt_executor/skill_action_node.hpp"
#include "duatic_skills_msgs/msg/mission_status.hpp"
#include "duatic_skills_msgs/srv/cancel_mission.hpp"
#include "duatic_skills_msgs/srv/execute_mission.hpp"
#include "rclcpp/rclcpp.hpp"

using ExecuteMission = duatic_skills_msgs::srv::ExecuteMission;
using CancelMission = duatic_skills_msgs::srv::CancelMission;
using MissionStatus = duatic_skills_msgs::msg::MissionStatus;

class BtExecutorNode : public rclcpp::Node
{
public:
  BtExecutorNode() : Node("bt_executor")
  {
    // Separate node for action clients — the main node is already in rclcpp::spin()
    // so spin_until_future_complete() would crash if we reused it.
    action_client_node_ = rclcpp::Node::make_shared("bt_action_clients");

    factory_.registerBuilder<duatic_bt::SkillActionNode>(
      "SkillAction",
      [this](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<duatic_bt::SkillActionNode>(name, config, action_client_node_);
      });

    // Services
    execute_srv_ = create_service<ExecuteMission>(
      "/mission/execute",
      [this](const ExecuteMission::Request::SharedPtr req,
             ExecuteMission::Response::SharedPtr res) { handle_execute(req, res); });

    cancel_srv_ = create_service<CancelMission>(
      "/mission/cancel",
      [this](const CancelMission::Request::SharedPtr req,
             CancelMission::Response::SharedPtr res) { handle_cancel(req, res); });

    // Status publisher (2 Hz)
    status_pub_ = create_publisher<MissionStatus>("/mission/status", 10);
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(500), [this]() { publish_status(); });

    RCLCPP_INFO(get_logger(), "BT executor ready.");
  }

  ~BtExecutorNode() override
  {
    halt_tree();
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
  }

private:
  void handle_execute(const ExecuteMission::Request::SharedPtr & req,
                      ExecuteMission::Response::SharedPtr & res)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (running_.load()) {
      res->accepted = false;
      res->message = "A mission is already running";
      return;
    }

    // Generate execution ID
    execution_id_++;
    std::string exec_id = "exec_" + std::to_string(execution_id_);

    try {
      tree_ = factory_.createTreeFromText(req->tree_xml);
    } catch (const std::exception & e) {
      res->accepted = false;
      res->message = std::string("Failed to parse BT XML: ") + e.what();
      return;
    }

    mission_name_ = req->mission_name;
    current_exec_id_ = exec_id;
    cancel_requested_.store(false);
    running_.store(true);
    tree_status_ = 0;  // running

    res->accepted = true;
    res->message = "Mission started";
    res->execution_id = exec_id;

    RCLCPP_INFO(get_logger(), "Mission '%s' started (id=%s)", mission_name_.c_str(),
                exec_id.c_str());

    // Run the tree in a background thread
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    execution_thread_ = std::thread([this]() { run_tree(); });
  }

  void handle_cancel(const CancelMission::Request::SharedPtr & req,
                     CancelMission::Response::SharedPtr & res)
  {
    if (!running_.load()) {
      res->success = false;
      res->message = "No mission running";
      return;
    }
    if (!req->execution_id.empty() && req->execution_id != current_exec_id_) {
      res->success = false;
      res->message = "Execution ID mismatch";
      return;
    }

    cancel_requested_.store(true);
    halt_tree();

    res->success = true;
    res->message = "Cancel requested";
    RCLCPP_INFO(get_logger(), "Mission cancel requested.");
  }

  void run_tree()
  {
    BT::NodeStatus result = BT::NodeStatus::RUNNING;

    while (result == BT::NodeStatus::RUNNING && rclcpp::ok()) {
      if (cancel_requested_.load()) {
        halt_tree();
        tree_status_ = 3;  // cancelled
        break;
      }
      result = tree_.tickOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (result == BT::NodeStatus::SUCCESS) {
      tree_status_ = 1;  // succeeded
      error_message_.clear();
      RCLCPP_INFO(get_logger(), "Mission succeeded.");
    } else if (result == BT::NodeStatus::FAILURE) {
      tree_status_ = 2;  // failed
      // Collect names of failed nodes for error message
      std::string failed_nodes;
      try {
        tree_.applyVisitor([&failed_nodes](const BT::TreeNode * node) {
          if (node->status() == BT::NodeStatus::FAILURE) {
            if (!failed_nodes.empty()) failed_nodes += ", ";
            failed_nodes += node->name();
          }
        });
      } catch (...) {}
      error_message_ = failed_nodes.empty() ? "Mission failed" : "Failed at: " + failed_nodes;
      RCLCPP_WARN(get_logger(), "Mission failed. %s", error_message_.c_str());
    }

    running_.store(false);
  }

  void halt_tree()
  {
    try {
      tree_.haltTree();
    } catch (...) {
    }
  }

  void publish_status()
  {
    MissionStatus msg;
    msg.execution_id = current_exec_id_;
    msg.mission_name = mission_name_;
    msg.status = tree_status_;
    msg.error_message = error_message_;

    // Collect node statuses from the tree
    if (running_.load()) {
      try {
        tree_.applyVisitor([&msg](const BT::TreeNode * node) {
          auto status = node->status();
          std::string id = node->name() + "_" + std::to_string(node->UID());
          if (status == BT::NodeStatus::RUNNING) {
            msg.active_node_ids.push_back(id);
          } else if (status == BT::NodeStatus::SUCCESS) {
            msg.succeeded_node_ids.push_back(id);
          } else if (status == BT::NodeStatus::FAILURE) {
            msg.failed_node_ids.push_back(id);
          }
        });
      } catch (...) {
      }
    }

    status_pub_->publish(msg);
  }

  rclcpp::Node::SharedPtr action_client_node_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::mutex mutex_;
  std::thread execution_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> cancel_requested_{false};

  rclcpp::Service<ExecuteMission>::SharedPtr execute_srv_;
  rclcpp::Service<CancelMission>::SharedPtr cancel_srv_;
  rclcpp::Publisher<MissionStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::string mission_name_;
  std::string current_exec_id_;
  std::string error_message_;
  uint64_t execution_id_{0};
  uint8_t tree_status_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BtExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
