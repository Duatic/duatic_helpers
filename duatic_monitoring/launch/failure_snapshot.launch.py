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

"""Records a rosbag2 snapshot of the topic data around a failure.

Starts a recorder in snapshot mode that holds the most recent topic data in memory, a
second recorder for the latched topics the first one does not retain, the failure_snapshot
node that has the recorder write its buffer when its ~/trigger service is called, and the
estop_monitor node that calls it on a hardware e-stop. Each launch writes to its own
directory under snapshot_root.

Open the snapshot files one at a time, or run `ros2 bag reindex` on the bag directory
first: its metadata can be missing, or list snapshots that were removed since.
`ros2 run duatic_monitoring failure_snapshot_logs <session dir>` prints the events and the
recorded ROS log messages as text.
"""

import fcntl
import os
from datetime import datetime

import launch.logging
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from duatic_monitoring.failure_snapshot_node import (
    LOCK_FILE,
    SESSION_NAME_FORMAT,
    SNAPSHOT_BAG,
    STATIC_BAG,
)

EXCLUDED_TYPES = [
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/PointCloud2",
]
LATCHED_TOPICS_REGEX = ".*/(tf_static|robot_description)$"


def launch_setup(context):
    root = os.path.expanduser(LaunchConfiguration("snapshot_root").perform(context))
    session_dir = os.path.join(root, datetime.now().strftime(SESSION_NAME_FORMAT))
    try:
        os.makedirs(session_dir)
        # A raw descriptor, never closed: the lock lasts until the launch process exits,
        # so another launch does not prune this session.
        lock = os.open(os.path.join(session_dir, LOCK_FILE), os.O_CREAT | os.O_WRONLY, 0o644)
        fcntl.flock(lock, fcntl.LOCK_EX)
    except OSError as e:
        # The robot starts without it.
        launch.logging.get_logger("failure_snapshot").error(
            f"Failure snapshots are off, cannot create {session_dir}: {e}"
        )
        return []

    # The recorders are not namespaced by launch, so the namespace goes into their names
    # to keep two robots on one domain apart.
    namespace = context.launch_configurations.get("ros_namespace", "").strip("/")
    suffix = "_" + namespace.replace("/", "_") if namespace else ""
    snapshot_recorder = f"failure_snapshot_recorder{suffix}"
    static_recorder = f"failure_static_recorder{suffix}"

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
                snapshot_recorder,
                "--output",
                os.path.join(session_dir, SNAPSHOT_BAG),
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
                static_recorder,
                "--output",
                os.path.join(session_dir, STATIC_BAG),
            ],
        ),
        Node(
            package="duatic_monitoring",
            executable="failure_snapshot_node",
            name="failure_snapshot",
            parameters=[
                {
                    "session_dir": session_dir,
                    "snapshot_service": f"/{snapshot_recorder}/snapshot",
                    "static_split_service": f"/{static_recorder}/split_bagfile",
                    "max_disk_bytes": LaunchConfiguration("snapshot_disk_bytes"),
                }
            ],
        ),
        Node(
            package="duatic_monitoring",
            executable="estop_monitor_node",
            name="estop_monitor",
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
                default_value="250000000",
                description="Size of the in-memory buffer a snapshot is written from",
            ),
            DeclareLaunchArgument(
                "snapshot_disk_bytes",
                default_value="20000000000",
                description="Disk space the snapshots under snapshot_root may take, oldest removed first",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
