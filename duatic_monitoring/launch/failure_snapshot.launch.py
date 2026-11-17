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

"""Records a rosbag2 snapshot of the topic data around a hardware e-stop.

Starts a recorder in snapshot mode that holds the most recent topic data in memory, a
second recorder for the latched topics the first one does not retain, and the node that
triggers the snapshot. Each launch writes to its own directory under snapshot_root.
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from duatic_monitoring.failure_snapshot_trigger_node import SESSION_NAME_FORMAT

EXCLUDED_TYPES = [
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/PointCloud2",
]
LATCHED_TOPICS_REGEX = ".*/(tf_static|robot_description)$"


def launch_setup(context):
    root = os.path.expanduser(LaunchConfiguration("snapshot_root").perform(context))
    session_dir = os.path.join(root, datetime.now().strftime(SESSION_NAME_FORMAT))
    os.makedirs(session_dir, exist_ok=True)

    return [
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "--all-topics",
                "--snapshot-mode",
                "--max-cache-size",
                LaunchConfiguration("snapshot_buffer_bytes"),
                "--exclude-topic-types",
                *EXCLUDED_TYPES,
                "--storage",
                "mcap",
                "--node-name",
                "failure_snapshot_recorder",
                "--output",
                os.path.join(session_dir, "snapshot"),
            ],
        ),
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "--regex",
                LATCHED_TOPICS_REGEX,
                "--storage",
                "mcap",
                "--node-name",
                "failure_static_recorder",
                "--output",
                os.path.join(session_dir, "static"),
            ],
        ),
        Node(
            package="duatic_monitoring",
            executable="failure_snapshot_trigger_node",
            name="failure_snapshot_trigger",
            parameters=[{"session_dir": session_dir}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "snapshot_root",
                default_value=EnvironmentVariable(
                    "DUATIC_SNAPSHOT_DIR", default_value="~/.ros/duatic_snapshots"
                ),
                description="Directory the snapshot sessions are written under",
            ),
            DeclareLaunchArgument(
                "snapshot_buffer_bytes",
                default_value="100000000",
                description="Size of the in-memory buffer a snapshot is written from",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
