# Copyright 2026 Duatic AG
# SPDX-License-Identifier: BSD-3-Clause

"""Launch actions that start the bringup supervisor next to a flat list of processes."""

import glob
import os
from typing import Iterable, Mapping, Sequence

from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from duatic_bringup_supervisor import graph

TIMEOUT_SCALE_ARGUMENT = "stage_timeout_scale"


def stage_names(stage_files: Iterable[str]) -> list[str]:
    """Every stage named in any of the files, in first-seen order."""
    names: list[str] = []
    for path in stage_files:
        for name in graph.load_files([path])["stages"]:
            if name not in names:
                names.append(name)
    return names


def stage_arguments(directory: str) -> list[DeclareLaunchArgument]:
    """Declare ``require_<stage>`` for every stage in a directory's stage files, and the scale.

    An empty ``require_<stage>`` keeps what the stage files say, so robot config overlays apply
    unless the launch command sets the argument.
    """
    return [
        *[
            DeclareLaunchArgument(
                f"require_{name}",
                default_value="",
                choices=["", "true", "false"],
                description=f"Stop the robot if stage '{name}' fails ('true'), continue without "
                "it ('false'), or keep what the stage files say (empty)",
            )
            for name in stage_names(sorted(glob.glob(os.path.join(directory, "*.yaml"))))
        ],
        DeclareLaunchArgument(
            TIMEOUT_SCALE_ARGUMENT,
            default_value="1.0",
            description="Factor applied to every stage timeout, for slow starts such as a simulator",
        ),
    ]


def supervisor(
    stage_files: Sequence[str],
    variables: Mapping[str, str] | None = None,
    optional_files: Sequence[str] = (),
) -> Node:
    """Start the supervisor, and end the launch when it exits.

    :param stage_files: The base stage file, then the overlays in the order they apply.
    :param variables: Values for ``${name}`` in the stage files.
    :param optional_files: Overlays applied last when they exist, such as a robot's own config.
    :returns: The supervisor node, which ends the launch with its exit code.
    """
    files = [*stage_files, *[path for path in optional_files if os.path.exists(path)]]
    return Node(
        package="duatic_bringup_supervisor",
        executable="supervisor",
        name="bringup_supervisor",
        output="screen",
        on_exit=_end_launch,
        parameters=[
            {
                "stage_files": files,
                "timeout_scale": LaunchConfiguration(TIMEOUT_SCALE_ARGUMENT, default="1.0"),
                **{
                    f"require.{name}": LaunchConfiguration(f"require_{name}", default="")
                    # Every stage named in any file: setting one an overlay removed is an error
                    for name in stage_names(files)
                },
                **{f"vars.{key}": value for key, value in (variables or {}).items()},
            }
        ],
    )


def _end_launch(event, context):
    """End the launch when the supervisor exits; a non-zero exit makes ``ros2 launch`` fail."""
    if context.is_shutdown:
        return None
    if event.returncode == 0:
        return [Shutdown(reason="The bringup supervisor stopped the robot")]

    def fail(_context):
        """Raise, so that the launch shuts down with exit code 1."""
        raise RuntimeError(
            f"The bringup supervisor exited with code {event.returncode}, see its log for the stage"
        )

    return [OpaqueFunction(function=fail)]
