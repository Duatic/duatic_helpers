# Copyright 2026 Duatic AG
# SPDX-License-Identifier: BSD-3-Clause

"""Launch actions that start the bringup supervisor next to a flat list of processes."""

import functools
import os
from typing import Any, Mapping, Sequence

from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from duatic_bringup_supervisor import graph

TIMEOUT_SCALE_ARGUMENT = "stage_timeout_scale"


# A robot's own overrides, for example a stage that this robot does not need
ROBOT_STAGES = "/ros2_ws/robot_config/stages.yaml"


@functools.lru_cache
def stage_names(package: str) -> tuple[str, ...]:
    """Every stage that a robot's stage files produce, alone or with any one overlay name.

    Included robots count with their prefixed names, and with the overlays they inherit.
    """
    directory = graph.package_stage_directory(package)
    names: list[str] = []
    for selected in [[], *([name] for name in graph.overlay_names(directory))]:
        document = graph.load_files(graph.stage_files(directory, selected))
        for name in graph.expand(document, overlays=selected)["stages"]:
            if name not in names:
                names.append(name)
    return tuple(names)


def stage_arguments(package: str) -> list[DeclareLaunchArgument]:
    """Declare ``require_<stage>`` for every stage of a robot, and the timeout scale.

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
            for name in stage_names(package)
        ],
        DeclareLaunchArgument(
            TIMEOUT_SCALE_ARGUMENT,
            default_value="1.0",
            description="Factor applied to every stage timeout, for slow starts such as a simulator",
        ),
    ]


def supervisor(
    context,
    package: str,
    overlays: Sequence[str] = (),
    variables: Mapping[str, Any] | None = None,
    optional_files: Sequence[str] = (ROBOT_STAGES,),
) -> list[Node]:
    """Start the supervisor for a robot, unless the robot runs as a subcomponent.

    A subcomponent starts none: the robot that includes it runs its stages in its own graph.
    The decision is read from ``start_as_subcomponent`` when this is called, before any include
    of the calling launch file can change it.

    :param context: The launch context of the calling ``OpaqueFunction``.
    :param package: The robot's bringup package, holding ``config/stages/``.
    :param overlays: Names of the overlays to apply over ``stages.yaml``, in order. A robot
        without one of them skips it, and included robots get the same names.
    :param variables: Values for ``${name}`` in the stage files: strings or launch substitutions.
    :param optional_files: Overlays applied last when they exist, such as a robot's own config.
    :returns: The supervisor node, which ends the launch with its exit code, or nothing.
    """
    subcomponent = context.launch_configurations.get("start_as_subcomponent", "false")
    if subcomponent.lower() == "true":
        return []
    return [
        Node(
            package="duatic_bringup_supervisor",
            executable="supervisor",
            name="bringup_supervisor",
            output="screen",
            on_exit=_end_launch,
            parameters=[
                {
                    "directory": graph.package_stage_directory(package),
                    "overlays": ",".join(overlays),
                    "optional_files": ",".join(p for p in optional_files if os.path.exists(p)),
                    "timeout_scale": LaunchConfiguration(TIMEOUT_SCALE_ARGUMENT, default="1.0"),
                    # Every set require_<x>: the supervisor rejects one that names no stage
                    **{
                        f"require.{key[len('require_'):]}": value
                        for key, value in context.launch_configurations.items()
                        if key.startswith("require_") and value
                    },
                    **{f"vars.{key}": value for key, value in (variables or {}).items()},
                }
            ],
        )
    ]


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
