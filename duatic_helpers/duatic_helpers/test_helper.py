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

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from typing import Union, List


def wait_for_node(node, node_name: str, timeout: float = 10.0):
    """
    Waits until the specified node name appears in the ROS graph.

    Args:
        node: rclpy Node object used to query node names.
        node_name: Name of the node to look for.
        timeout: How long to wait before failing.
        poll_interval: How often to check.

    Raises:
        AssertionError: If the node is not found within the timeout.
    """
    found = False
    start = time.time()

    while time.time() - start < timeout:
        if node_name in node.get_node_names():
            found = True
            break
        time.sleep(0.1)

    assert (
        found
    ), f"Node '{node_name}' not found within {timeout} seconds. Nodes: {node.get_node_names()}"


def wait_for_node_with_namespace(
    node: Node, node_name: str, node_namespace: str, timeout: float = 10.0
):
    """
    Waits until the specified node with the exact name and namespace appears in the ROS graph.

    Args:
        node: rclpy Node object used to query node names and namespaces.
        node_name: Name of the node to look for (e.g., 'my_node').
        node_namespace: Expected namespace of the node (e.g., '/robot1').
        timeout: Max time to wait for the node.

    Raises:
        AssertionError: If the node with the namespace is not found within the timeout.
    """
    found = False
    start = time.time()

    while time.time() - start < timeout:
        node_tuples = node.get_node_names_and_namespaces()  # List of (name, namespace)
        for name, namespace in node_tuples:
            if name == node_name and namespace == node_namespace:
                found = True
                break
        if found:
            break
        time.sleep(0.1)

    assert found, (
        f"Node '{node_name}' with namespace '{node_namespace}' not found within {timeout} seconds.\n"
        f"Available nodes: {node_tuples}"
    )


def wait_for_topic(node, topic_name: str, timeout: float = 10.0):
    """
    This will get deprecated soon. Use wait_for_topics() instead.
    """
    wait_for_topics(node, [topic_name], timeout)


def wait_for_topics(node, topic_names: Union[str, List[str]], timeout: float = 10.0):
    """
    Waits until the specified topic(s) are advertised in the ROS graph.

    Args:
        node: rclpy Node object used to query topics.
        topic_names: A single topic name or a list of topic names to look for.
        timeout: How long to wait before failing.

    Raises:
        AssertionError: If not all topics are found within the timeout.
    """
    # Normalize input to a set for easier tracking
    if isinstance(topic_names, str):
        missing_topics = {topic_names}
    else:
        missing_topics = set(topic_names)

    start = time.time()
    current_topics = []

    while time.time() - start < timeout:
        # Get list of currently advertised topics
        current_topics_info = node.get_topic_names_and_types()
        # Extract just the names
        current_topics = {name for name, _ in current_topics_info}

        # Remove found topics from our missing set
        missing_topics = missing_topics - current_topics

        # If no topics are missing, we are done
        if not missing_topics:
            return

        time.sleep(0.1)

    assert not missing_topics, (
        f"Topics {missing_topics} not advertised within {timeout} seconds. "
        f"Available topics: {current_topics}"
    )


def wait_for_message(
    node: Node,
    topic_name: str,
    msg_type_str: str,
    timeout: float = 10.0,
    qos_profile: QoSProfile = 10,
):
    """Waits until a message is received on the topic within the timeout."""
    received_msg = []

    def callback(msg):
        received_msg.append(msg)

    sub = node.create_subscription(msg_type_str, topic_name, callback, qos_profile)

    start = time.time()
    while time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if received_msg:
            break

    node.destroy_subscription(sub)
    assert received_msg, f"No message received on {topic_name} within {timeout} seconds."
