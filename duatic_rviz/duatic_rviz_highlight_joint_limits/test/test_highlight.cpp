/*
 * Copyright 2026 Duatic AG
 * Duatic Commercial License 1.0 (DCL-1)
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "duatic_rviz_highlight_joint_limits/joint_limits_highlight.hpp"

namespace
{

using duatic::rviz_highlight::JointLimitsHighlight;
using duatic::rviz_highlight::LinkColors;
namespace palette = duatic::rviz_highlight::palette;

constexpr char kPrefix[] = "highlight.joint_limits.";

// A revolute joint with a range of 0.2 rad, about 11.5 degrees. Its lower limit is 0, so a position that is wrongly
// assumed to be 0 would be critical
constexpr char kSmallRangeUrdf[] = R"(
<robot name="r">
  <link name="base"/>
  <link name="arm"/>
  <joint name="small" type="revolute">
    <parent link="base"/><child link="arm"/>
    <limit lower="0.0" upper="0.2" effort="1" velocity="1"/>
  </joint>
</robot>)";

std::vector<std::string> g_messages;

void captureLog(const rcutils_log_location_t*, int, const char*, rcutils_time_point_value_t, const char* format,
                va_list* args)
{
  char buffer[1024];
  vsnprintf(buffer, sizeof(buffer), format, *args);
  g_messages.emplace_back(buffer);
}

class HighlightTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    g_messages.clear();
    previous_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(captureLog);
    node = std::make_shared<rclcpp::Node>("highlight_test");
  }
  void TearDown() override
  {
    rcutils_logging_set_output_handler(previous_handler);
  }

  bool logged(const std::string& text) const
  {
    return std::any_of(g_messages.begin(), g_messages.end(),
                       [&text](const std::string& message) { return message.find(text) != std::string::npos; });
  }

  /**
   * @brief Publish the position of the joint `small` until the highlight colors its link `arm` as `expected`.
   * @param position The position of the joint, in rad.
   * @param expected The color of the link, or no color.
   * @param timeout How long to wait for the expected color.
   * @return True if the link had the expected color within the timeout.
   */
  bool showsEventually(double position, const std::optional<duatic::rviz_highlight::Color>& expected,
                       std::chrono::milliseconds timeout = std::chrono::seconds(2))
  {
    const auto publisher =
        node->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
    sensor_msgs::msg::JointState state;
    state.name = { "small" };
    state.position = { position };
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
      publisher->publish(state);
      rclcpp::spin_some(node);
      LinkColors colors;
      highlight.parametersChanged();
      highlight.update(colors);
      const auto found = colors.find("arm");
      if (expected ? (found != colors.end() && found->second == *expected) : colors.empty()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  }

  rcutils_logging_output_handler_t previous_handler = nullptr;
  rclcpp::Node::SharedPtr node;
  JointLimitsHighlight highlight;
};

}  // namespace

TEST_F(HighlightTest, WarnsIfTheWarningMarginIsHalfTheRangeOfAJointOrMore)
{
  highlight.initialize(node, kPrefix);
  highlight.onRobotDescription(kSmallRangeUrdf);

  // The default warning margin is 15 degrees: more than half of the 11.5 degrees of the joint
  LinkColors colors;
  highlight.update(colors);
  EXPECT_TRUE(logged("always highlighted: small"));
}

TEST_F(HighlightTest, NoWarningIfTheMarginsFitTheRange)
{
  node->declare_parameter<double>(std::string(kPrefix) + "margin.absolute_deg.warning", 2.0);
  node->declare_parameter<double>(std::string(kPrefix) + "margin.absolute_deg.critical", 1.0);
  highlight.initialize(node, kPrefix);
  highlight.onRobotDescription(kSmallRangeUrdf);

  LinkColors colors;
  highlight.update(colors);
  EXPECT_FALSE(logged("always highlighted"));
  EXPECT_FALSE(logged("smaller than the critical margin"));
}

TEST_F(HighlightTest, WarnsIfTheWarningMarginIsSmallerThanTheCriticalOne)
{
  node->declare_parameter<double>(std::string(kPrefix) + "margin.absolute_deg.warning", 1.0);
  node->declare_parameter<double>(std::string(kPrefix) + "margin.absolute_deg.critical", 2.0);
  highlight.initialize(node, kPrefix);

  LinkColors colors;
  highlight.update(colors);
  EXPECT_TRUE(logged("smaller than the critical margin"));
}

TEST_F(HighlightTest, MarginsFollowTheParametersOnceRefreshed)
{
  node->declare_parameter<double>(std::string(kPrefix) + "margin.absolute_deg.warning", 2.0);
  node->declare_parameter<double>(std::string(kPrefix) + "margin.absolute_deg.critical", 1.0);
  highlight.initialize(node, kPrefix);
  highlight.onRobotDescription(kSmallRangeUrdf);
  EXPECT_TRUE(showsEventually(0.18, palette::kOrange));  // 1.1 degrees from the upper limit: within the warning margin

  node->set_parameter(rclcpp::Parameter(std::string(kPrefix) + "margin.absolute_deg.warning", 0.5));
  EXPECT_TRUE(showsEventually(0.18, std::nullopt));  // the same position is now outside of the narrower margin
}

TEST_F(HighlightTest, ColorsTheLinkOfAJointByItsLatestPosition)
{
  // Margins that fit the 11.5 degrees of the joint: a warning within 2 degrees of a limit, critical within 1 degree
  node->declare_parameter<double>(std::string(kPrefix) + "margin.absolute_deg.warning", 2.0);
  node->declare_parameter<double>(std::string(kPrefix) + "margin.absolute_deg.critical", 1.0);
  highlight.initialize(node, kPrefix);
  highlight.onRobotDescription(kSmallRangeUrdf);

  LinkColors colors;
  highlight.update(colors);
  EXPECT_TRUE(colors.empty());  // no joint state yet: nothing is known about the joint

  EXPECT_TRUE(showsEventually(0.18, palette::kOrange));  // 1.1 degrees from the upper limit
  EXPECT_TRUE(showsEventually(0.195, palette::kRed));    // 0.3 degrees from the upper limit
  EXPECT_TRUE(showsEventually(0.1, std::nullopt));       // back in the middle: the color goes away again
}

TEST_F(HighlightTest, ModeAllUsesTheMoreSevereOfBothMargins)
{
  // Absolute: 0.1 degrees (no warning). Relative: warning within 0.2 of the range, i.e. 2.3 degrees
  node->declare_parameter<std::string>(std::string(kPrefix) + "mode", "all");
  node->declare_parameter<double>(std::string(kPrefix) + "margin.absolute_deg.warning", 0.1);
  node->declare_parameter<double>(std::string(kPrefix) + "margin.absolute_deg.critical", 0.05);
  node->declare_parameter<double>(std::string(kPrefix) + "margin.relative.warning", 0.2);
  node->declare_parameter<double>(std::string(kPrefix) + "margin.relative.critical", 0.01);
  highlight.initialize(node, kPrefix);
  highlight.onRobotDescription(kSmallRangeUrdf);

  EXPECT_TRUE(showsEventually(0.18, palette::kOrange));  // only the relative margin triggers
  EXPECT_TRUE(showsEventually(0.1, std::nullopt));       // neither triggers
}
