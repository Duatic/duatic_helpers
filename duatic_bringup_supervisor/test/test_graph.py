# Copyright 2026 Duatic AG
# SPDX-License-Identifier: BSD-3-Clause

import textwrap

import pytest

from duatic_bringup_supervisor.graph import (
    Ros2Control,
    StageGraphError,
    TopicCheck,
    dependents,
    load_files,
    merge,
    parse,
    startable,
    stop_order,
    topological_order,
)

ROVER = {
    "stages": {
        "core": {"timeout": 120, "delegate": {"ros2_control": {"controllers_config": "${cfg}"}}},
        "imu": {"timeout": 30, "lifecycle_nodes": ["imu_driver"]},
        "lidars": {"timeout": 30, "ready": {"topics": [{"name": "scan", "max_age": 1.0}]}},
        "state_estimation": {
            "timeout": 60,
            "after": ["core", "imu"],
            "ready": {"topics": ["odom"]},
        },
        "navigation": {
            "timeout": 150,
            "after": ["state_estimation", "lidars"],
            "delegate": {"nav2": ["lifecycle_manager_navigation"]},
        },
    }
}


def write(tmp_path, name, text):
    """Write a stage file and return its path."""
    path = tmp_path / name
    path.write_text(textwrap.dedent(text))
    return str(path)


def test_parse_reads_every_kind_of_part():
    stages = parse(ROVER, variables={"cfg": "/robot/controllers.yaml"})
    assert stages["core"].ros2_control == Ros2Control(controllers_config="/robot/controllers.yaml")
    assert stages["imu"].lifecycle_nodes == ("imu_driver",)
    assert stages["lidars"].topics == (TopicCheck("scan", 1.0),)
    assert stages["state_estimation"].topics == (TopicCheck("odom"),)
    assert stages["navigation"].nav2_managers == ("lifecycle_manager_navigation",)
    assert all(stage.required for stage in stages.values())


def test_start_order_respects_dependencies():
    order = topological_order(parse(ROVER, variables={"cfg": ""}))
    assert order.index("core") < order.index("state_estimation") < order.index("navigation")
    assert order.index("lidars") < order.index("navigation")


def test_independent_stages_start_together():
    stages = parse(ROVER, variables={"cfg": ""})
    status = {name: "pending" for name in stages}
    assert startable(stages, status) == ["core", "imu", "lidars"]
    status.update(core="ready", imu="ready", lidars="starting")
    assert startable(stages, status) == ["state_estimation"]


def test_stop_order_stops_dependents_first():
    stages = parse(ROVER, variables={"cfg": ""})
    order = stop_order(stages, list(stages))
    assert order.index("navigation") < order.index("state_estimation") < order.index("core")
    assert order.index("navigation") < order.index("lidars")


def test_dependents_are_transitive():
    stages = parse(ROVER, variables={"cfg": ""})
    assert dependents(stages, "imu") == ["state_estimation", "navigation"]
    assert dependents(stages, "navigation") == []


def test_required_override_and_timeout_scale():
    stages = parse(
        ROVER, variables={"cfg": ""}, required={"lidars": "false", "imu": ""}, timeout_scale=2
    )
    assert not stages["lidars"].required
    assert stages["imu"].required
    assert stages["navigation"].timeout == 300


@pytest.mark.parametrize(
    "document, message",
    [
        ({"stages": {"a": {"timeout": 1, "after": ["b"]}}}, "unknown stages"),
        (
            {"stages": {"a": {"timeout": 1, "after": ["b"]}, "b": {"timeout": 1, "after": ["a"]}}},
            "cycle",
        ),
        ({"stages": {"a-b": {"timeout": 1}}}, "letters, digits"),
        ({"stages": {"a": {}}}, "needs a 'timeout'"),
        ({"stages": {"a": {"timeout": 1, "lifecycle_node": ["x"]}}}, "unknown keys"),
        ({"stages": {"a": {"timeout": 1, "delegate": {"nav": ["x"]}}}}, "unknown keys"),
        ({"stages": {"a": {"timeout": 1, "ready": {"topics": [{"max_age": 1}]}}}}, "'name'"),
        ({"stages": {"a": {"timeout": "${missing}"}}}, "Unknown variable"),
    ],
)
def test_invalid_graphs_are_rejected(document, message):
    with pytest.raises(StageGraphError, match=message):
        parse(document)


def test_invalid_overrides_are_rejected():
    with pytest.raises(StageGraphError, match="not in the graph"):
        parse(ROVER, variables={"cfg": ""}, required={"camera": "false"})
    # Unset overrides of stages an overlay removed are fine
    parse(ROVER, variables={"cfg": ""}, required={"camera": ""})
    with pytest.raises(StageGraphError, match="true, false or empty"):
        parse(ROVER, variables={"cfg": ""}, required={"imu": "maybe"})
    with pytest.raises(StageGraphError, match="positive"):
        parse(ROVER, variables={"cfg": ""}, timeout_scale=0)


def test_merge_replaces_lists_and_removes_null_keys():
    base = {"stages": {"a": {"timeout": 1, "after": ["x", "y"]}, "b": {"timeout": 2}}}
    overlay = {"stages": {"a": {"after": ["z"]}, "b": None, "c": {"timeout": 3}}}
    assert merge(base, overlay) == {
        "stages": {"a": {"timeout": 1, "after": ["z"]}, "c": {"timeout": 3}}
    }


def test_load_files_applies_overlays_in_order(tmp_path):
    base = write(
        tmp_path,
        "real.yaml",
        """
        stages:
          imu: {timeout: 30, lifecycle_nodes: [imu_driver]}
          odom: {timeout: 60, after: [imu]}
        """,
    )
    sim = write(
        tmp_path,
        "sim.yaml",
        """
        stages:
          imu: null
          odom: {after: []}
        """,
    )
    robot = write(tmp_path, "robot.yaml", "stages:\n  odom: {required: false}\n")
    stages = parse(load_files([base, sim, robot]))
    assert list(stages) == ["odom"]
    assert stages["odom"].after == ()
    assert not stages["odom"].required


def test_load_files_needs_a_stages_mapping(tmp_path):
    with pytest.raises(StageGraphError, match="top-level 'stages'"):
        load_files([write(tmp_path, "bad.yaml", "imu: {timeout: 1}\n")])
