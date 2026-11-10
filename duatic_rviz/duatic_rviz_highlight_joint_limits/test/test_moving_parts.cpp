/*
 * Copyright 2026 Duatic AG
 * Duatic Commercial License 1.0 (DCL-1)
 */

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "duatic_rviz_highlight_joint_limits/moving_parts.hpp"

using namespace duatic::rviz_highlight;  // NOLINT(build/namespaces)

namespace
{

const char kUrdf[] = R"(
<robot name="r">
  <link name="base"/>
  <link name="a_link"/>
  <link name="c_link"/>
  <link name="e_link"/>
  <link name="p_link"/>
  <link name="a_tool"/>
  <link name="a_tip"/>
  <link name="b_link"/>
  <joint name="a" type="revolute">
    <parent link="base"/><child link="a_link"/>
    <limit lower="-1.0" upper="1.0" effort="1" velocity="1"/>
  </joint>
  <joint name="a_tool_fixed" type="fixed">
    <parent link="a_link"/><child link="a_tool"/>
  </joint>
  <joint name="a_tip_fixed" type="fixed">
    <parent link="a_tool"/><child link="a_tip"/>
  </joint>
  <joint name="b" type="revolute">
    <parent link="a_link"/><child link="b_link"/>
    <limit lower="-1.0" upper="1.0" effort="1" velocity="1"/>
  </joint>
  <joint name="c" type="continuous">
    <parent link="base"/><child link="c_link"/>
  </joint>
  <joint name="e" type="revolute">
    <parent link="base"/><child link="e_link"/>
    <limit effort="1" velocity="1"/>
  </joint>
  <joint name="p" type="prismatic">
    <parent link="base"/><child link="p_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="0.0" upper="0.5" effort="1" velocity="1"/>
  </joint>
</robot>
)";
}  // namespace

TEST(ParseMovingParts, KeepsOnlyRevoluteJointsWithARange)
{
  const auto parts = parseMovingParts(kUrdf);

  ASSERT_EQ(parts.size(), 2U);  // a and b, not the continuous, the unlimited and the prismatic joint
  EXPECT_DOUBLE_EQ(parts.at("a").limits.lower, -1.0);
  EXPECT_DOUBLE_EQ(parts.at("a").limits.upper, 1.0);
}

TEST(ParseMovingParts, JointMovesLinksUpToTheNextMovableJoint)
{
  const auto parts = parseMovingParts(kUrdf);

  // The fixed joints below a are followed, the revolute joint b starts a part of its own
  EXPECT_EQ(parts.at("a").links, (std::vector<std::string>{ "a_link", "a_tool", "a_tip" }));
  EXPECT_EQ(parts.at("b").links, (std::vector<std::string>{ "b_link" }));
}

TEST(ParseMovingParts, InvalidUrdfGivesNoParts)
{
  EXPECT_TRUE(parseMovingParts("not a urdf").empty());
}

TEST(MovedLinks, UnknownRootGivesNoLinks)
{
  urdf::Model model;
  ASSERT_TRUE(model.initString(kUrdf));

  EXPECT_TRUE(movedLinks(model, "not_a_link").empty());
}
