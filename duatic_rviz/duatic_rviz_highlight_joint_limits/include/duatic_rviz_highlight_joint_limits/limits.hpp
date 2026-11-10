/*
 * Copyright 2026 Duatic AG
 * Duatic Commercial License 1.0 (DCL-1)
 */

#pragma once

#include <angles/angles.h>

#include <algorithm>
#include <optional>
#include <stdexcept>

#include "duatic_rviz_highlight/common/color.hpp"

namespace duatic::rviz_highlight
{

/** @brief What closeness margins are used: degrees, or fractions of the joint's range from 0 to 0.5 */
enum class MarginMode
{
  kAbsoluteDeg,
  kRelative
};

/** @brief The margins of a mode, in the unit of that mode: the distances to a limit for a warning and for critical */
struct Margins
{
  MarginMode mode;
  double warning;
  double critical;
};

/** @brief How close a joint is to a limit: within the first (warning) or the second (critical) margin, or not close */
enum class Severity
{
  kNone,
  kWarning,
  kCritical
};

/**
 * @brief The color to show a severity in, if any.
 * @param severity The severity.
 * @return The color, or `std::nullopt` for `Severity::kNone`.
 * @throws std::logic_error if `severity` is none of the enumerators.
 */
inline std::optional<Color> severityColor(Severity severity)
{
  switch (severity) {
    case Severity::kCritical:
      return palette::kRed;
    case Severity::kWarning:
      return palette::kOrange;
    case Severity::kNone:
      return std::nullopt;
  }
  throw std::logic_error("Invalid Severity");
}

/** @brief The positions a joint can take, in rad */
struct JointLimits
{
  double lower;
  double upper;

  /**
   * @brief How close a position is to a limit.
   *
   * The distance is the smaller one to the two limits. A position beyond a limit counts as critical.
   * @param position The position of the joint, in rad.
   * @param warning_margin The distance to a limit within which the severity is a warning.
   * @param critical_margin The distance to a limit within which the severity is critical.
   * @param mode The unit of the margins: degrees, or fractions of the range (0 to 0.5).
   * @return The severity.
   */
  Severity severityAt(double position, double warning_margin, double critical_margin, MarginMode mode) const
  {
    const double remaining_rad = std::min(position - lower, upper - position);
    const double remaining = mode == MarginMode::kAbsoluteDeg ? angles::to_degrees(remaining_rad) :
                                                             remaining_rad / (upper - lower);
    if (remaining <= critical_margin) {
      return Severity::kCritical;
    }
    if (remaining <= warning_margin) {
      return Severity::kWarning;
    }
    return Severity::kNone;
  }
};

}  // namespace duatic::rviz_highlight
