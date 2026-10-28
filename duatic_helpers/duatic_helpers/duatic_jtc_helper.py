# Copyright 2026 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import annotations

import re
from typing import TYPE_CHECKING

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from duatic_helpers.duatic_param_helper import DuaticParamHelper

if TYPE_CHECKING:
    from typing import Any

    from rclpy.node import Node


class DuaticJTCHelper:
    """Helper class for Joint Trajectory Controller topic discovery and management."""

    def __init__(self, node: Node) -> None:
        """
        Construct the helper.

        Args:
            node: Node used to query the ROS graph and the controllers' parameters.
        """
        self.node = node
        self.duatic_param_helper = DuaticParamHelper(self.node)

    def process_topics_and_extract_joint_names(
        self, found_topics: list[tuple[str, list[str]]]
    ) -> tuple[dict[str, list[str]], dict[str, Any]]:
        """
        Look up the joint names of the controllers behind the given topics.

        The controller is taken from the second-to-last topic segment, and its 'joints'
        parameter is read. Topics for which no non-empty 'joints' parameter is found are
        skipped.

        Args:
            found_topics: Topic names with their types, e.g. as returned by
                find_topics_for_controller().

        Returns:
            Tuple of a mapping from topic to the joint names of its controller, and a mapping
            from topic to an empty placeholder for the commanded positions of that topic, which
            the caller is expected to fill.
        """
        topic_to_joint_names = {}  # Dictionary to map topics to joint names
        topic_to_commanded_positions = {}  # Dictionary to hold commanded positions for each topic

        # Discover all topics and joint names, extract prefix
        for topic, types in found_topics:

            # Normalize topic and obtain the controller segment (the token before the final identifier)
            # e.g. '/robo2/joint_trajectory_controller_arm_left/joint_trajectory' ->
            # segments = ['robo2', 'joint_trajectory_controller_arm_left', 'joint_trajectory']
            segments = topic.strip("/").split("/")
            if len(segments) < 2:
                self.node.get_logger().error(f"Unexpected topic format: {topic}")
                continue

            controller_ns = segments[-2]

            # Try several ways to fetch the 'joints' parameter (namespaced vs non-namespaced)
            param_result = None
            try_names = [
                controller_ns,  # 'joint_trajectory_controller_arm_left'
                f"/{controller_ns}",  # '/joint_trajectory_controller_arm_left'
                "/".join(segments[:-1]),  # 'robo2/joint_trajectory_controller_arm_left'
                f"/{'/'.join(segments[:-1])}",  # '/robo2/joint_trajectory_controller_arm_left'
            ]

            tried = []
            for name in try_names:
                if name in tried:
                    continue
                tried.append(name)
                param_result = self.duatic_param_helper.get_param_values(name, "joints")
                if param_result:
                    self.node.get_logger().debug(
                        f"Found 'joints' param under '{name}' for topic {topic}"
                    )
                    break

            if param_result is None or not param_result:
                self.node.get_logger().error(
                    f"Parameter 'joints' not found for controller '{controller_ns}'. Tried: {try_names}"
                )
                continue  # skip this topic but continue processing others

            # param_result is expected to be a list of parameter value objects; take first
            try:
                joint_names = list(param_result[0].string_array_value)
            except Exception as e:
                self.node.get_logger().error(
                    f"Failed to read 'joints' parameter for {controller_ns}: {e}"
                )
                continue

            self.node.get_logger().debug(
                f"Retrieved joint names for {controller_ns}: {joint_names}"
            )
            if joint_names:
                topic_to_joint_names[topic] = joint_names
                topic_to_commanded_positions[topic] = {}
            else:
                self.node.get_logger().warning(
                    f"Parameter 'joints' empty for controller {controller_ns} (topic {topic})"
                )

        return topic_to_joint_names, topic_to_commanded_positions

    def get_topic_names_and_types_function(self, by_name: str) -> list[tuple[str, list[str]]]:
        """
        Get topic names and types, filtering by a given name pattern.

        Args:
            by_name: Pattern to search for in the topic names. If it contains '*', each '*'
                matches any characters and the pattern is searched as a regular expression,
                with a leading '/' added if missing. Otherwise it is matched literally.

        Returns:
            Topic names with their types, for all topics that match the pattern.
        """
        # Convert wildcard pattern to regex pattern
        # Handle the specific case where we search for controller patterns
        if "*" in by_name:
            # Convert patterns like "joint_trajectory_controller*/joint_trajectory"
            # to match "/joint_trajectory_controller_arm_left/joint_trajectory"
            pattern_str = by_name.replace("*", ".*")
            # Ensure we match from the beginning of the topic name
            if not pattern_str.startswith("/"):
                pattern_str = "/" + pattern_str
            pattern = re.compile(pattern_str)
        else:
            pattern = re.compile(re.escape(by_name))

        topics_and_types = self.node.get_topic_names_and_types()
        self.node.get_logger().debug(f"Found topics and types: {topics_and_types}")

        matches = [(topic, types) for topic, types in topics_and_types if pattern.search(topic)]
        self.node.get_logger().debug(f"Filtered topics matching '{by_name}': {matches}")
        return matches

    def find_topics_for_controller(
        self, controller_name: str, identifier: str, component_names: list[str] = []
    ) -> list[tuple[str, list[str]]]:
        """
        Find topics for a given controller name and identifier, filtering by component names.

        Retries for up to 10 s until at least as many topics as component names are found.

        Args:
            controller_name: Name prefix of the controllers, e.g. 'joint_trajectory_controller'.
            identifier: Last topic segment to look for, e.g. 'joint_trajectory'.
            component_names: Component names that the controller namespaces must end with
                (an empty name matches every controller).

        Returns:
            Topic names with their types, or an empty list if no component names were given or
            fewer topics than component names were found.
        """
        if not component_names:
            self.node.get_logger().warning("No component names provided, returning empty list")
            return []

        found_topics = []

        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        topics_deadline = steady_clock.now() + Duration(seconds=10.0)

        # Construct the search pattern
        namespace = self.node.get_namespace().strip("/")
        if namespace:
            search_pattern = f"/{namespace}/{controller_name}*/{identifier}"
        else:
            search_pattern = f"/{controller_name}*/{identifier}"

        # Find all topics matching the controller name and identifier
        while steady_clock.now() < topics_deadline:
            all_matched_topics = self.get_topic_names_and_types_function(search_pattern)

            # Filter topics to only include those with the component_names in their controller namespace
            found_topics = [
                (topic, types)
                for topic, types in all_matched_topics
                if any(
                    f"_{component_name}/" in topic if component_name else True
                    for component_name in component_names
                )
            ]

            if len(found_topics) >= len(component_names):
                break

            self.node.get_logger().info(
                f"Found {len(found_topics)} topics, expecting {len(component_names)}. Retrying...",
                throttle_duration_sec=10,
            )
            rclpy.spin_once(self.node, timeout_sec=0.5)

        if len(found_topics) < len(component_names):
            self.node.get_logger().error(
                f"Expected {len(component_names)} controller topics, but found {len(found_topics)}"
            )
            self.node.get_logger().error(
                f"Available topics: {[topic for topic, _ in self.node.get_topic_names_and_types()]}"
            )
            self.node.get_logger().error(
                f"Search pattern: {search_pattern}, Component names: {component_names}"
            )
            self.node.get_logger().error(f"Found topics: {[topic for topic, _ in found_topics]}")
            # Return empty list instead of raising exception to prevent crash
            return []
        self.node.get_logger().debug(f"Found topics: {[topic for topic, _ in found_topics]}")
        return found_topics
