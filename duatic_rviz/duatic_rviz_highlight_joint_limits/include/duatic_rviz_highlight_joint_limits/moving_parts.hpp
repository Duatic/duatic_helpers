/*
 * Copyright 2026 Duatic AG
 * Duatic Commercial License 1.0 (DCL-1)
 */

#pragma once

#include <urdf/model.h>

#include <deque>
#include <map>
#include <string>
#include <vector>

#include "duatic_rviz_highlight_joint_limits/limits.hpp"

namespace duatic::rviz_highlight
{

/** @brief What a joint moves, with the limits of that joint, see `movedLinks` */
struct MovingPart
{
  JointLimits limits;
  std::vector<std::string> links;
};

/**
 * @brief Return `root` and all links below it that are connected by fixed joints, i.e. up to the next movable joint.
 * @param model The robot model.
 * @param root The link to start from.
 * @return The names of the links, empty if `root` is not in the model.
 */
inline std::vector<std::string> movedLinks(const urdf::Model& model, const std::string& root)
{
  std::vector<std::string> links;
  std::deque<std::string> pending{ root };
  while (!pending.empty()) {
    // Pop the next link to process
    const auto link = model.getLink(pending.front());
    pending.pop_front();
    if (!link) continue;  // only really possible for an unknown `root`
    links.push_back(link->name);

    // Add the child links of all fixed joints to the pending list
    for (const auto& joint : link->child_joints) {
      if (joint->type == urdf::Joint::FIXED) {
        pending.push_back(joint->child_link_name);
      }
    }
  }
  return links;
}

/**
 * @brief Return the `MovingPart` that each revolute joint with a lower/upper limit moves, by joint name.
 * @param urdf_xml The URDF of the robot. If it cannot be parsed, there are no parts.
 * @return The parts, by joint name.
 */
inline std::map<std::string, MovingPart> parseMovingParts(const std::string& urdf_xml)
{
  std::map<std::string, MovingPart> parts;

  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    return parts;
  }
  for (const auto& [name, joint] : model.joints_) {
    const auto& limits = joint->limits;
    if (joint->type != urdf::Joint::REVOLUTE || !limits || limits->lower >= limits->upper) {
      continue;
    }
    parts[name] = MovingPart{ { limits->lower, limits->upper }, movedLinks(model, joint->child_link_name) };
  }
  return parts;
}

}  // namespace duatic::rviz_highlight
