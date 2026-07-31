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

#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <duatic_helper_msgs/srv/get_joint_states.hpp>

using GetJointStates = duatic_helper_msgs::srv::GetJointStates;

static std::vector<std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>>> joint_state_subscribers_{};
static std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_state_publisher_{nullptr};
static std::shared_ptr<rclcpp::Service<GetJointStates>> joint_state_service_{nullptr};
static std::shared_ptr<rclcpp::TimerBase> update_timer_{nullptr};
static std::shared_ptr<rclcpp::Node> node_{nullptr};

// Internal representation of the current state of a single joint
struct JointState
{
  std::string name;
  double position;
  // These field are also in the official ros2 JointState message definition optional
  std::optional<double> velocity;
  std::optional<double> effort;
  // Producer time - obtained from JointState message header
  rclcpp::Time produce_time;
  // Receive time - point of time at which we received this message
  rclcpp::Time receive_time;
};

static std::map<std::string, JointState> joint_states_;

static void on_new_joint_state(const sensor_msgs::msg::JointState& msg)
{
  // Check if the message is valid - all fields must have the same size
  if (msg.position.size() != msg.name.size() ||
      (!msg.velocity.empty() && msg.velocity.size() != msg.name.size()) ||
      (!msg.effort.empty() && msg.effort.size() != msg.name.size())) {
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 100,
        "Received invalid joint state message - fields do not have the same size");
    return;
  }

  const auto now = node_->now();

  for (std::size_t i = 0; i < msg.name.size(); i++) {
    JointState state;

    state.name = msg.name[i];
    state.position = msg.position[i];
    state.receive_time = now;
    state.produce_time = msg.header.stamp;

    if (msg.velocity.size() > 0)
      state.velocity = msg.velocity[i];
    if (msg.effort.size() > 0)
      state.effort = msg.effort[i];

    if (joint_states_.count(msg.name[i])) {
      joint_states_[msg.name[i]] = state;
    } else {
      joint_states_.insert({ msg.name[i], state });
    }
  }
}
static void on_joint_state_request(const GetJointStates::Request::SharedPtr& request [[maybe_unused]],
                                   GetJointStates::Response::SharedPtr response)
{
  // Produce a single joint state message from our internal state and set it in the response
  // NOTE as we use a singlethreaded executor there is no need for locking
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->get_clock()->now();

  for (const auto& elem : joint_states_) {
    msg.name.push_back(elem.first);

    msg.position.push_back(elem.second.position);
    if (elem.second.velocity)
      msg.velocity.push_back(elem.second.velocity.value());
    if (elem.second.effort)
      msg.effort.push_back(elem.second.effort.value());

    // Additionally we provide the produce and receive time stamp
    response->produce_time_stamps.push_back(elem.second.produce_time);
    response->receive_time_stamps.push_back(elem.second.receive_time);
  }

  response->state = msg;
}

// Regular update at which the full current state is republished
static void on_update()
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->get_clock()->now();
  for (const auto& elem : joint_states_) {
    msg.name.push_back(elem.first);

    msg.position.push_back(elem.second.position);
    if (elem.second.velocity)
      msg.velocity.push_back(elem.second.velocity.value());
    if (elem.second.effort)
      msg.effort.push_back(elem.second.effort.value());
  }
  // If for some reason the publisher was not created successfully but the update was started -> better check it
  if (joint_state_publisher_)
    joint_state_publisher_->publish(msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  node_ = std::make_shared<rclcpp::Node>("joint_state_provider_node");
  // Declare all the parameters with usable defaults
  node_->declare_parameter<double>("publish_rate", 100.0);
  node_->declare_parameter<std::vector<std::string>>("listen_topics");
  node_->declare_parameter<std::string>("publish_topic", "/joint_states");

  // The rate in Hz at which the combined joint state message gets published
  const auto publish_rate = node_->get_parameter("publish_rate").as_double();
  // Topics we listen for new joint state messages
  const auto listen_topic_names = node_->get_parameter("listen_topics").as_string_array();
  // Name of the topic we publish the joint states to (This is per default /joint_states)
  const auto publish_topic_name = node_->get_parameter("publish_topic").as_string();

  // Subscribe to all configured listen topics.
  for (const auto& listen_topic : listen_topic_names) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Subscribing to topic: " << listen_topic);
    joint_state_subscribers_.push_back(
        node_->create_subscription<sensor_msgs::msg::JointState>(listen_topic, 10, &on_new_joint_state));
  }

  RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing to topic: " << publish_topic_name);
  joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(publish_topic_name, 10);
  // Service for reading the latest joint states state
  joint_state_service_ = node_->create_service<GetJointStates>("/joint_states/get", &on_joint_state_request);
  RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing rate: " << publish_rate << "Hz");

  update_timer_ =
      node_->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(1000.0 / publish_rate)), &on_update);

  rclcpp::spin(node_);

  rclcpp::shutdown();
}
