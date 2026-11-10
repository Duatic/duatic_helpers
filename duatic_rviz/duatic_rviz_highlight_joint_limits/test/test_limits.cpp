/*
 * Copyright 2026 Duatic AG
 * Duatic Commercial License 1.0 (DCL-1)
 */

#include <angles/angles.h>
#include <gtest/gtest.h>

#include <stdexcept>

#include "duatic_rviz_highlight_joint_limits/limits.hpp"

using namespace duatic::rviz_highlight;  // NOLINT(build/namespaces)

namespace
{
// A joint that moves between -90 and 90 degrees
const JointLimits kLimits{ angles::from_degrees(-90.0), angles::from_degrees(90.0) };
}  // namespace

TEST(SeverityAt, AbsoluteMarginsAreDegrees)
{
  // Margins: warning within 10 degrees, critical within 2.5 degrees
  const auto severity = [](double position) {
    return kLimits.severityAt(position, 10.0, 2.5, MarginMode::kAbsoluteDeg);
  };
  EXPECT_EQ(severity(0.0), Severity::kNone);
  EXPECT_EQ(severity(angles::from_degrees(85.0)), Severity::kWarning);  // 5 degrees left
  EXPECT_EQ(severity(angles::from_degrees(-85.0)), Severity::kWarning);
  EXPECT_EQ(severity(angles::from_degrees(89.5)), Severity::kCritical);  // 0.5 degrees left
  EXPECT_EQ(severity(angles::from_degrees(95.0)), Severity::kCritical);  // beyond the limit
}

TEST(SeverityAt, MarginsAreInclusive)
{
  const JointLimits unit{ 0.0, 1.0 };  // exact binary fractions: no rounding in the distances
  // Exactly the warning margin, exactly the critical margin, and just outside
  EXPECT_EQ(unit.severityAt(0.25, 0.25, 0.125, MarginMode::kRelative), Severity::kWarning);
  EXPECT_EQ(unit.severityAt(0.125, 0.25, 0.125, MarginMode::kRelative), Severity::kCritical);
  EXPECT_EQ(unit.severityAt(0.26, 0.25, 0.125, MarginMode::kRelative), Severity::kNone);
}

TEST(SeverityAt, RelativeMarginsAreFractionsOfTheRange)
{
  const JointLimits wide{ angles::from_degrees(-180.0), angles::from_degrees(180.0) };  // 360 degrees range
  // 30 degrees left: not close in degrees, but within 0.1 of the range (0.083)
  EXPECT_EQ(wide.severityAt(angles::from_degrees(150.0), 10.0, 2.5, MarginMode::kAbsoluteDeg), Severity::kNone);
  EXPECT_EQ(wide.severityAt(angles::from_degrees(150.0), 0.1, 0.025, MarginMode::kRelative), Severity::kWarning);
  // 3 degrees left are 0.0083 of the range
  EXPECT_EQ(wide.severityAt(angles::from_degrees(177.0), 0.1, 0.025, MarginMode::kRelative), Severity::kCritical);
}
