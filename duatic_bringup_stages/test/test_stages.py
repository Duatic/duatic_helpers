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

"""Stage ordering, abort and skip behaviour, with stub gate processes instead of ROS nodes."""

import sys

import pytest
from launch import LaunchDescription, LaunchService
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.event_handlers import OnShutdown
from launch import LaunchDescriptionSource
from launch_ros.actions import PushRosNamespace

from duatic_bringup_stages.stages import Stage, stage_arguments, staged


def stub_gate(exit_code, delay=0.1):
    return ExecuteProcess(
        cmd=[sys.executable, "-c", f"import sys, time; time.sleep({delay}); sys.exit({exit_code})"]
    )


def run_with_arguments(entities, arguments):
    # ros2 launch passes command line arguments the same way, through an include
    service = LaunchService()
    service.include_launch_description(
        IncludeLaunchDescription(
            LaunchDescriptionSource(LaunchDescription(entities)), launch_arguments=list(arguments)
        )
    )
    return service.run()


class Recorder:
    def __init__(self):
        self.started = []
        self.namespaces = {}
        self.environments = {}
        self.shutdown_reasons = []

    def marker(self, name):
        def record(context):
            self.started.append(name)
            self.namespaces[name] = context.launch_configurations.get("ros_namespace")
            self.environments[name] = context.environment.get("STAGE_TEST_VARIABLE")

        return OpaqueFunction(function=record)

    def stage(self, name, exit_code, after=(), configurable=True, delay=0.1):
        return Stage(
            name=name,
            actions=[self.marker(name)],
            gate=stub_gate(exit_code, delay),
            after=list(after),
            configurable=configurable,
        )

    def run(self, entities, arguments=()):
        def on_shutdown(event, _context):
            self.shutdown_reasons.append(event.reason)

        return run_with_arguments(
            [RegisterEventHandler(OnShutdown(on_shutdown=on_shutdown)), *entities], arguments
        )

    @property
    def aborted(self):
        return any("Aborting the launch" in str(reason) for reason in self.shutdown_reasons)


def declared(recorder_stages):
    names = [stage.name for stage in recorder_stages if stage.configurable]
    return stage_arguments(names)


def test_stages_start_in_dependency_order():
    r = Recorder()
    stages = [
        r.stage("navigation", 0, after=["estimation"]),
        r.stage("estimation", 0, after=["core", "imu"]),
        r.stage("core", 0, configurable=False),
        r.stage("imu", 0, delay=0.5),
    ]
    assert r.run([*declared(stages), staged(stages)]) == 0

    assert set(r.started) == {"core", "imu", "estimation", "navigation"}
    assert r.started.index("estimation") > max(r.started.index("core"), r.started.index("imu"))
    assert r.started.index("navigation") > r.started.index("estimation")
    assert not r.aborted


def test_stage_waits_for_its_slowest_dependency():
    r = Recorder()
    stages = [
        r.stage("fast", 0, delay=0.1),
        r.stage("slow", 0, delay=1.0),
        r.stage("both", 0, after=["fast", "slow"]),
    ]
    r.run([*declared(stages), staged(stages)])

    assert r.started == ["fast", "slow", "both"]


def test_required_stage_failure_aborts_and_starts_nothing_after_it():
    r = Recorder()
    stages = [
        r.stage("core", 0, configurable=False),
        r.stage("imu", 1),
        r.stage("estimation", 0, after=["imu"]),
    ]
    assert r.run([*declared(stages), staged(stages)]) == 1

    assert "estimation" not in r.started
    assert r.aborted
    assert any("'imu'" in str(reason) for reason in r.shutdown_reasons)


def test_core_cannot_be_made_optional():
    r = Recorder()
    stages = [r.stage("core", 1, configurable=False), r.stage("arms", 0, after=["core"])]
    assert r.run([*declared(stages), staged(stages)], arguments=[("require_core", "false")]) == 1

    assert "arms" not in r.started
    assert r.aborted


def test_optional_stage_failure_skips_optional_dependents_and_keeps_the_rest():
    r = Recorder()
    stages = [
        r.stage("core", 0, configurable=False),
        r.stage("imu", 1),
        r.stage("estimation", 0, after=["core", "imu"]),
        r.stage("navigation", 0, after=["estimation"]),
        r.stage("arms", 0, after=["core"], delay=0.5),
    ]
    r.run(
        [*declared(stages), staged(stages)],
        arguments=[
            ("require_imu", "false"),
            ("require_estimation", "false"),
            ("require_navigation", "false"),
        ],
    )

    assert "estimation" not in r.started
    assert "navigation" not in r.started
    assert "arms" in r.started
    assert not r.aborted


def test_required_dependent_of_an_optional_failure_aborts():
    r = Recorder()
    stages = [r.stage("imu", 1), r.stage("estimation", 0, after=["imu"])]
    r.run([*declared(stages), staged(stages)], arguments=[("require_imu", "false")])

    assert "estimation" not in r.started
    assert r.aborted
    assert any("'estimation'" in str(reason) for reason in r.shutdown_reasons)


def test_later_stages_keep_the_namespace():
    r = Recorder()
    stages = [r.stage("core", 0, configurable=False), r.stage("arms", 0, after=["core"])]
    r.run([*declared(stages), GroupAction([PushRosNamespace("robot1"), staged(stages)])])

    assert r.namespaces == {"core": "/robot1", "arms": "/robot1"}


def test_stages_starting_together_do_not_share_launch_configurations():
    r = Recorder()
    seen = {}

    def record(context):
        seen["arms"] = context.launch_configurations.get("set_by_core")

    stages = [
        Stage(
            name="core",
            actions=[SetLaunchConfiguration("set_by_core", "yes")],
            gate=stub_gate(0),
            configurable=False,
        ),
        Stage(name="arms", actions=[OpaqueFunction(function=record)], gate=stub_gate(0)),
    ]
    r.run([*declared(stages), staged(stages)])

    assert seen == {"arms": None}


def test_later_stages_keep_the_environment():
    r = Recorder()
    stages = [r.stage("core", 0, configurable=False), r.stage("arms", 0, after=["core"])]
    r.run(
        [
            *declared(stages),
            GroupAction([SetEnvironmentVariable("STAGE_TEST_VARIABLE", "set"), staged(stages)]),
        ]
    )

    assert r.environments == {"core": "set", "arms": "set"}


def test_stage_arguments_reject_other_values():
    r = Recorder()
    stages = [r.stage("imu", 0)]
    exit_code = run_with_arguments([*declared(stages), staged(stages)], [("require_imu", "maybe")])

    assert exit_code != 0
    assert "imu" not in r.started


@pytest.mark.parametrize(
    "after, message",
    [
        ({"a": ["missing"]}, "unknown stages"),
        ({"a": ["b"], "b": ["a"]}, "cycle"),
    ],
)
def test_invalid_stage_graphs_are_rejected(after, message):
    stages = [
        Stage(name=name, actions=[], gate=stub_gate(0), after=dependencies)
        for name, dependencies in after.items()
    ]
    with pytest.raises(ValueError, match=message):
        staged(stages)


def test_duplicate_stage_names_are_rejected():
    stages = [Stage(name="a", actions=[], gate=stub_gate(0)) for _ in range(2)]
    with pytest.raises(ValueError, match="unique"):
        staged(stages)


@pytest.mark.parametrize("name", ["state-estimation", "1st", "", "a b"])
def test_invalid_stage_names_are_rejected(name):
    with pytest.raises(ValueError, match="letters, digits and underscores"):
        staged([Stage(name=name, actions=[], gate=stub_gate(0))])


def test_stage_arguments_default_to_required():
    require, timeout_scale = stage_arguments(["imu"])

    assert isinstance(require, DeclareLaunchArgument)
    assert require.name == "require_imu"
    assert require.default_value[0].text == "true"
    assert require.choices == ["true", "false"]
    assert timeout_scale.name == "stage_timeout_scale"
    assert timeout_scale.default_value[0].text == "1.0"


CHECKER = """
import os, sys, time
time.sleep(3)
def count(arguments):
    found = []
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                if f.read().split(b"\\0")[:2] == [b"sleep", arguments.encode()]:
                    found.append(int(pid))
        except OSError:
            pass
    return found
failed, kept = count("61"), count("62")
with open(sys.argv[1], "w") as f:
    f.write(f"{len(failed)} {len(kept)}")
for pid in kept:
    os.kill(pid, 15)
"""


def test_failed_optional_stage_stops_its_processes(tmp_path):
    result = tmp_path / "alive.txt"
    failing = Stage(
        name="imu",
        actions=[
            ExecuteProcess(cmd=["sleep", "61"]),
            IncludeLaunchDescription(
                LaunchDescriptionSource(LaunchDescription([ExecuteProcess(cmd=["sleep", "61"])]))
            ),
        ],
        gate=stub_gate(1, delay=1.0),
    )
    kept = Stage(
        name="lidars",
        actions=[
            ExecuteProcess(cmd=["sleep", "62"]),
            ExecuteProcess(cmd=[sys.executable, "-c", CHECKER, str(result)]),
        ],
        gate=stub_gate(0),
    )
    r = Recorder()
    exit_code = r.run(
        [*stage_arguments(["imu", "lidars"]), staged([failing, kept])],
        arguments=[("require_imu", "false")],
    )

    assert exit_code == 0
    assert not r.aborted
    # No process of the failed stage is left, the other stage's process still ran
    assert result.read_text() == "0 1"
