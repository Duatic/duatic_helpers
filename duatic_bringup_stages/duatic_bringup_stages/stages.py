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

"""Start launch actions in stages, each one confirmed by a gate process before its dependents start."""

import os
import re
import uuid
from dataclasses import dataclass, field
from typing import Iterable, List, Sequence

from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.actions.execute_local import ExecuteLocal
from launch.event_handlers import OnProcessExit
from launch.events.process import ShutdownProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


@dataclass
class Stage:
    """
    Launch actions that start together, and the gate that decides whether they came up.

    The actions start once every stage named in ``after`` is ready. The stage is ready when its gate
    exits with code 0. Any other exit code fails the stage.
    """

    name: str
    actions: List
    gate: ExecuteProcess
    after: List[str] = field(default_factory=list)
    # A stage that is not configurable is always required, it gets no require_<name> argument
    configurable: bool = True


TIMEOUT_SCALE_ARGUMENT = "stage_timeout_scale"


def require_argument(stage_name: str) -> str:
    return f"require_{stage_name}"


def stage_arguments(stage_names: Iterable[str]) -> List[DeclareLaunchArgument]:
    """Declare the require_<stage> argument of each configurable stage, and the timeout scale."""
    return [
        *[
            DeclareLaunchArgument(
                require_argument(name),
                default_value="true",
                choices=["true", "false"],
                description=f"Abort the launch if the '{name}' stage does not come up. When false, "
                "continue without it and without the stages that need it",
            )
            for name in stage_names
        ],
        DeclareLaunchArgument(
            TIMEOUT_SCALE_ARGUMENT,
            default_value="1.0",
            description="Factor applied to the timeout of every stage gate, for slow starts "
            "such as a simulator",
        ),
    ]


def gate(
    stage_name: str,
    timeout: float,
    *,
    topics: Sequence[str] = (),
    controllers_config: str = "",
    hardware_components_active: bool = False,
    lifecycle_nodes: Sequence[str] = (),
    controller_manager: str = "controller_manager",
) -> Node:
    """Gate process that exits 0 once every given check passes, or 1 after the timeout."""
    return Node(
        package="duatic_bringup_stages",
        executable="stage_gate.py",
        name=f"{stage_name}_gate",
        output="screen",
        parameters=[
            {
                "stage": stage_name,
                "timeout": float(timeout),
                # A ROS parameter cannot be an empty list, the gate ignores empty entries
                "topics": list(topics) or [""],
                "controllers_config": controllers_config,
                "hardware_components_active": hardware_components_active,
                "lifecycle_nodes": list(lifecycle_nodes) or [""],
                "controller_manager": controller_manager,
                "timeout_scale": ParameterValue(
                    LaunchConfiguration(TIMEOUT_SCALE_ARGUMENT, default="1.0"), value_type=str
                ),
            }
        ],
    )


def abort(reason: str) -> OpaqueFunction:
    """Shut the launch down with the reason, so that ros2 launch exits with code 1."""

    def raise_reason(context):
        if context.is_shutdown:
            return None
        raise RuntimeError(reason)

    return OpaqueFunction(function=raise_reason)


# Every process a stage starts carries the stage's token, so a failed stage can stop its processes
STAGE_TOKENS = "DUATIC_BRINGUP_STAGES"


def _processes_of(token):
    def matches(action):
        details = action.process_details if isinstance(action, ExecuteLocal) else None
        return bool(details) and token in details["env"].get(STAGE_TOKENS, "").split(":")

    return matches


def staged(stages: Sequence[Stage]) -> OpaqueFunction:
    """Return the action that runs the stages. Place it where the stages should take their namespace from."""
    _validate(stages)
    return OpaqueFunction(function=_run, args=[list(stages)])


def _validate(stages: Sequence[Stage]) -> None:
    names = [stage.name for stage in stages]
    # The name is used in the gate node name and the require_<name> argument
    invalid = [name for name in names if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name)]
    if invalid:
        raise ValueError(f"Stage names must be letters, digits and underscores, got: {invalid}")
    duplicates = {name for name in names if names.count(name) > 1}
    if duplicates:
        raise ValueError(f"Stage names must be unique, duplicated: {sorted(duplicates)}")
    for stage in stages:
        unknown = [name for name in stage.after if name not in names]
        if unknown:
            raise ValueError(f"Stage '{stage.name}' comes after unknown stages {unknown}")

    by_name = {stage.name: stage for stage in stages}
    done = set()

    def visit(name, path):
        if name in path:
            raise ValueError(
                f"Stages depend on each other in a cycle: {' -> '.join(path + [name])}"
            )
        if name in done:
            return
        for dependency in by_name[name].after:
            visit(dependency, path + [name])
        done.add(name)

    for name in names:
        visit(name, [])


def _run(context, stages: List[Stage]):
    # Actions returned by an exit handler run in the scope of the whole launch and would lose the namespace,
    # arguments and environment of this point, so they are started with a copy of them
    configurations = dict(context.launch_configurations)
    environment = dict(os.environ)
    status = {stage.name: "pending" for stage in stages}
    tokens = {stage.name: uuid.uuid4().hex for stage in stages}

    def in_scope(actions):
        def restore(scope_context):
            # Lists such as global_params are copied, so a stage cannot extend them for the others
            scope_context.launch_configurations.update(
                {
                    key: list(value) if isinstance(value, list) else value
                    for key, value in configurations.items()
                }
            )
            scope_context.environment.update(environment)

        return [GroupAction([OpaqueFunction(function=restore), *actions])]

    def required(stage):
        if not stage.configurable:
            return True
        return configurations.get(require_argument(stage.name), "true") == "true"

    def start(stage):
        status[stage.name] = "starting"

        def on_gate_exit(event, exit_context):
            if exit_context.is_shutdown:
                return None
            if event.returncode == 0:
                status[stage.name] = "ready"
                return in_scope([LogInfo(msg=f"Stage '{stage.name}' is ready"), *start_ready()])
            return in_scope(fail(stage, f"its gate exited with code {event.returncode}"))

        # Its own scope, so that stages starting in the same step do not see each other's launch configurations
        return [
            GroupAction(
                [
                    LogInfo(msg=f"Starting stage '{stage.name}'"),
                    SetEnvironmentVariable(
                        STAGE_TOKENS,
                        [
                            EnvironmentVariable(STAGE_TOKENS, default_value=""),
                            ":",
                            tokens[stage.name],
                        ],
                    ),
                    RegisterEventHandler(
                        OnProcessExit(target_action=stage.gate, on_exit=on_gate_exit)
                    ),
                    *stage.actions,
                    stage.gate,
                ]
            )
        ]

    def start_ready():
        actions = []
        for stage in stages:
            if status[stage.name] == "pending" and all(
                status[name] == "ready" for name in stage.after
            ):
                actions += start(stage)
        return actions

    def fail(stage, reason):
        status_before_failure = status[stage.name]
        status[stage.name] = "failed"
        if required(stage):
            return [abort(f"Stage '{stage.name}' failed, {reason}. Aborting the launch")]
        actions = [
            LogInfo(
                msg=f"Stage '{stage.name}' failed, {reason}. Stopping it and continuing without it "
                f"({require_argument(stage.name)} is false)"
            )
        ]
        if status_before_failure == "starting":
            actions.append(
                EmitEvent(event=ShutdownProcess(process_matcher=_processes_of(tokens[stage.name])))
            )
        for dependent in stages:
            if status[dependent.name] == "pending" and stage.name in dependent.after:
                actions += fail(dependent, f"it needs stage '{stage.name}'")
        return actions

    def describe(stage):
        text = stage.name
        if stage.after:
            text += f" after {', '.join(stage.after)}"
        if not required(stage):
            text += " (optional)"
        return text

    return [
        LogInfo(msg="Stages: " + "; ".join(describe(stage) for stage in stages)),
        *start_ready(),
    ]
