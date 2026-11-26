# Copyright 2026 Duatic AG
# SPDX-License-Identifier: BSD-3-Clause

import textwrap

import pytest

from duatic_bringup_supervisor.graph import (
    Ros2Control,
    StageGraphError,
    TopicCheck,
    dependents,
    expand,
    load_files,
    merge,
    overlay_names,
    parse,
    stage_files,
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


def test_load_files_rejects_unknown_top_level_keys(tmp_path):
    with pytest.raises(StageGraphError, match="unknown keys"):
        load_files([write(tmp_path, "bad.yaml", "imu: {timeout: 1}\n")])


def robot(tmp_path, name, files):
    """Write a robot's stage directory and return it."""
    directory = tmp_path / name
    directory.mkdir()
    for file_name, text in files.items():
        write(directory, file_name, text)
    return str(directory)


def packages(tmp_path, **robots):
    """Write stage directories for fake packages and return a resolver for them."""
    directories = {name: robot(tmp_path, name, files) for name, files in robots.items()}
    return lambda package: directories[package]


ROVER_FILES = {
    "stages.yaml": """
        stages:
          core:
            timeout: 10
            delegate: {ros2_control: {controllers_config: "${cfg}"}}
          imu: {timeout: 5, lifecycle_nodes: [imu_driver]}
          nav:
            after: [core, imu]
            timeout: 20
            delegate: {nav2: [lifecycle_manager]}
            ready: {topics: [odom]}
        """,
    "sim.yaml": """
        stages:
          imu: null
          nav: {after: [core]}
        """,
}


def test_include_prefixes_and_binds_stages(tmp_path):
    find = packages(tmp_path, rover=ROVER_FILES)
    document = {
        "include": {"rover": {"package": "rover", "bind": {"core": "core"}}},
        "stages": {"core": {"timeout": 30}, "rover_nav": {"required": False}},
    }
    stages = parse(document, variables={"cfg": ""}, find_directory=find)
    assert list(stages) == ["rover_imu", "rover_nav", "core"]
    assert stages["rover_nav"].after == ("core", "rover_imu")
    assert stages["rover_imu"].lifecycle_nodes == ("imu_driver",)
    assert not stages["rover_nav"].required
    assert stages["core"].timeout == 30


def test_included_robots_get_the_overlays_they_have(tmp_path):
    find = packages(tmp_path, rover=ROVER_FILES)
    document = {"include": {"rover": {"package": "rover"}}}
    # The rover has a sim overlay but no mapping overlay, which is skipped
    stages = parse(
        document, variables={"cfg": ""}, find_directory=find, overlays=["mapping", "sim"]
    )
    assert list(stages) == ["rover_core", "rover_nav"]
    assert stages["rover_nav"].after == ("rover_core",)


def test_includes_nest(tmp_path):
    find = packages(
        tmp_path,
        rover=ROVER_FILES,
        torso={
            "stages.yaml": """
                include:
                  rover: {package: rover}
                stages:
                  arms: {timeout: 5, after: [rover_core]}
                """
        },
    )
    stages = parse(
        {"include": {"torso": {"package": "torso"}}}, {"cfg": "/c.yaml"}, find_directory=find
    )
    assert stages["torso_rover_core"].ros2_control.controllers_config == "/c.yaml"
    assert stages["torso_arms"].after == ("torso_rover_core",)
    assert stages["torso_rover_nav"].after == ("torso_rover_core", "torso_rover_imu")


def test_overlay_names_include_those_of_included_robots(tmp_path):
    find = packages(
        tmp_path,
        rover=ROVER_FILES,
        dxtr={"stages.yaml": "include:\n  rover: {package: rover}\n", "mock.yaml": "stages: {}\n"},
    )
    assert overlay_names(find("dxtr"), find) == {"mock", "sim"}


def test_expand_without_variables_lists_names_only(tmp_path):
    find = packages(tmp_path, rover=ROVER_FILES)
    document = {"include": {"a": {"package": "rover"}, "b": {"package": "${tool_package}"}}}
    assert list(expand(document, find_directory=find)["stages"]) == ["a_core", "a_imu", "a_nav"]


@pytest.mark.parametrize(
    "include, message",
    [
        ({"directory": "/somewhere"}, "unknown keys"),
        ({}, "needs a 'package'"),
        ({"package": "rover", "bind": {"core": "core"}}, "bound to stages that do not exist"),
    ],
)
def test_invalid_includes_are_rejected(tmp_path, include, message):
    find = packages(tmp_path, rover=ROVER_FILES)
    with pytest.raises(StageGraphError, match=message):
        parse({"include": {"rover": include}}, variables={"cfg": ""}, find_directory=find)


def test_included_stage_names_must_not_collide(tmp_path):
    find = packages(
        tmp_path,
        first={"stages.yaml": "stages:\n  b_c: {timeout: 1}\n"},
        second={"stages.yaml": "stages:\n  c: {timeout: 1}\n"},
    )
    # Prefix a with stage b_c and prefix a_b with stage c both make a_b_c
    document = {"include": {"a": {"package": "first"}, "a_b": {"package": "second"}}}
    with pytest.raises(StageGraphError, match="exists twice"):
        parse(document, find_directory=find)


def test_stage_files_skips_overlays_a_robot_does_not_have(tmp_path):
    rover = robot(tmp_path, "rover", ROVER_FILES)
    assert stage_files(rover, ["mapping", "sim"]) == [f"{rover}/stages.yaml", f"{rover}/sim.yaml"]
    with pytest.raises(StageGraphError, match="not found"):
        stage_files(str(tmp_path))
