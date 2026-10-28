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

from typing import TYPE_CHECKING

import rclpy
from rclpy.parameter_client import AsyncParameterClient

if TYPE_CHECKING:
    from rcl_interfaces.msg import ParameterValue
    from rclpy.node import Node


class DuaticParamHelper:
    """Helper class for reading parameters (e.g. the robot URDF) from other nodes."""

    def __init__(self, node: Node) -> None:
        """
        Construct the helper.

        Args:
            node: Node used to create parameter clients and to spin while waiting.
        """
        self.node = node

    def get_param_values(
        self, controller_ns: str, param_name: str, timeout_sec: float = 2.0
    ) -> list[ParameterValue] | None:
        """
        Retrieve parameter values from another node.

        Spins the node while waiting, with a fixed 5 s timeout for the reply.

        Args:
            controller_ns: Name of the node that holds the parameter.
            param_name: Name of the parameter to read.
            timeout_sec: How long to wait for the node's parameter services, in seconds.

        Returns:
            The values of the requested parameters, or None if the services are not available,
            the request timed out or no value was returned.
        """
        param_client = AsyncParameterClient(self.node, controller_ns)

        # Check if services are available before requesting
        self.node.get_logger().debug(f"Waiting for parameter services on {controller_ns}...")
        if not param_client.wait_for_services(timeout_sec=timeout_sec):
            self.node.get_logger().warning(
                f"Parameter services on '{controller_ns}' not available after {timeout_sec}s"
            )
            return None

        future = param_client.get_parameters([param_name])

        self.node.get_logger().debug(f"Requesting parameter {param_name} from {controller_ns}")

        # Add a hard timeout to the spin loop to avoid infinite hangs
        start_time = self.node.get_clock().now()
        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.05)

            # 5 second safety timeout for the future itself
            if (self.node.get_clock().now() - start_time).nanoseconds > 5e9:
                self.node.get_logger().error(
                    f"Timeout waiting for parameter {param_name} from {controller_ns}"
                )
                return None

        self.node.get_logger().debug(
            f"Parameter {param_name} retrieval completed for {controller_ns}"
        )

        if future.result() is not None and future.result().values:  # type: ignore
            return future.result().values  # type: ignore

        return None

    def get_urdf(
        self, node_name: str = "robot_state_publisher", param_name: str = "robot_description"
    ) -> str | None:
        """
        Retrieve the robot URDF from the parameter server.

        If the node is not found under the given name, the ROS graph is searched for a single
        node with that base name in any namespace.

        Args:
            node_name: Name of the node holding the URDF. Pass the fully-qualified name to
                select one robot if several nodes of the same name exist.
            param_name: Name of the parameter holding the URDF.

        Returns:
            The URDF XML string, or None if it could not be retrieved (not found, empty or
            ambiguous).
        """
        try:
            urdf_values = self.get_param_values(node_name, param_name)
            if urdf_values and urdf_values[0].string_value:
                self.node.get_logger().info(f"Successfully loaded URDF from {node_name}")
                return urdf_values[0].string_value

            # Fall back: search the ROS graph for the node in any namespace. Return error if multiple matches are found.
            matches = []
            for name, ns in self.node.get_node_names_and_namespaces():
                if name == node_name:
                    full = ns.rstrip("/") + "/" + name if ns != "/" else "/" + name
                    if full != node_name:
                        matches.append(full)

            if len(matches) > 1:
                self.node.get_logger().error(
                    f"Ambiguous robot_description: found multiple '{node_name}' nodes: "
                    f"{matches}. Pass the fully-qualified node name to get_urdf() to "
                    f"select the correct robot."
                )
                return None

            if len(matches) == 1:
                self.node.get_logger().info(
                    f"Retrying URDF fetch from discovered node '{matches[0]}'"
                )
                urdf_values = self.get_param_values(matches[0], param_name)
                if urdf_values and urdf_values[0].string_value:
                    self.node.get_logger().info(f"Successfully loaded URDF from {matches[0]}")
                    return urdf_values[0].string_value

            self.node.get_logger().warning(
                f"URDF parameter '{param_name}' at '{node_name}' is empty"
            )
            return None

        except Exception as e:
            self.node.get_logger().warn(f"Failed to get URDF from {node_name}: {e}")
            return None
