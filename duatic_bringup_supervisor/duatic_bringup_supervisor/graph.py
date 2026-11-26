# Copyright 2026 Duatic AG
# SPDX-License-Identifier: BSD-3-Clause

"""Stage graph of a robot bringup: loading, merging, composing and checking stage files.

A robot keeps its stage files in ``<package share>/config/stages/``: ``stages.yaml`` is the base,
every other file is an overlay picked by name. A robot built from other robots includes their
stage files under a prefix, so that one supervisor runs the whole graph.

This module does not use ROS, so the graph rules can be tested without a running system.
"""

import copy
import functools
import glob
import os
import re
from graphlib import CycleError, TopologicalSorter
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping, Sequence

import yaml

STAGE_NAME = re.compile(r"[A-Za-z][A-Za-z0-9_]*")
VARIABLE = re.compile(r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}")

BASE_FILE = "stages.yaml"
DOCUMENT_KEYS = {"include", "stages"}
INCLUDE_KEYS = {"package", "bind"}
STAGE_KEYS = {"after", "required", "timeout", "lifecycle_nodes", "delegate", "ready"}
DELEGATE_KEYS = {"nav2", "ros2_control"}
ROS2_CONTROL_KEYS = {"controller_manager", "controllers_config", "manage"}
READY_KEYS = {"topics"}
TOPIC_KEYS = {"name", "max_age"}


class StageGraphError(ValueError):
    """A stage file or the graph it describes is invalid."""


@dataclass(frozen=True)
class TopicCheck:
    """A topic that must publish before the stage is ready.

    ``max_age`` (seconds) also makes it a liveness check once the stage is ready: the stage fails
    when no message arrives for that long.
    """

    name: str
    max_age: float | None = None


@dataclass(frozen=True)
class Ros2Control:
    """A controller manager the stage hands its controllers and hardware to.

    With ``manage`` false the stage only checks the controller manager, because another bringup
    (for example a parent robot) owns it.
    """

    controller_manager: str = "controller_manager"
    controllers_config: str = ""
    manage: bool = True


@dataclass(frozen=True)
class Stage:
    """One stage: what it starts, when it may start and when it counts as ready."""

    name: str
    timeout: float
    after: tuple[str, ...] = ()
    required: bool = True
    lifecycle_nodes: tuple[str, ...] = ()
    nav2_managers: tuple[str, ...] = ()
    ros2_control: Ros2Control | None = None
    topics: tuple[TopicCheck, ...] = field(default_factory=tuple)


def merge(base: Any, overlay: Any) -> Any:
    """Merge an overlay into a base document.

    Mappings merge key by key, a ``null`` value removes the key, anything else replaces the base
    value. Lists are replaced, not appended.
    """
    if not isinstance(base, dict) or not isinstance(overlay, dict):
        return overlay
    merged = dict(base)
    for key, value in overlay.items():
        if value is None:
            merged.pop(key, None)
        elif key in merged:
            merged[key] = merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def stage_files(directory: str, overlays: Sequence[str] = ()) -> list[str]:
    """The base stage file of a directory, then those of the named overlays it has, in order.

    A robot only has the overlays it needs (no ``mapping.yaml`` for a robot without navigation),
    so names without a file are skipped.

    :raises StageGraphError: When the directory has no base stage file.
    """
    base = os.path.join(directory, BASE_FILE)
    if not os.path.exists(base):
        raise StageGraphError(f"Stage file not found: {base}")
    paths = [os.path.join(directory, name + ".yaml") for name in overlays]
    return [base, *[path for path in paths if os.path.exists(path)]]


@functools.lru_cache
def package_stage_directory(package: str) -> str:
    """The stage directory of a ROS package: ``<share>/config/stages``."""
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory(package), "config", "stages")


@functools.lru_cache
def _read(path: str) -> dict:
    """One stage file, checked; cached, so callers get a copy from :func:`load_files`."""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    _check_keys(path, data, DOCUMENT_KEYS)
    for key in DOCUMENT_KEYS & set(data):
        if not isinstance(data[key], dict):
            raise StageGraphError(f"{path}: '{key}' must be a mapping")
    return data


def load_files(paths: Iterable[str]) -> dict:
    """Load stage files and merge each one over the ones before it.

    :param paths: The base file first, then the overlays in the order they apply.
    :returns: The merged document.
    :raises StageGraphError: When a file has other top-level keys than ``include`` and ``stages``.
    """
    document: dict = {"stages": {}}
    for path in paths:
        document = merge(document, copy.deepcopy(_read(path)))
    return document


def includes(document: Mapping, find_directory=package_stage_directory):
    """The includes of a document whose location is known, with their stage directories.

    An include whose ``package`` still holds a ``${variable}`` is skipped, which only happens
    when names are listed without values.

    :yields: ``(prefix, bind, directory)`` for each include.
    :raises StageGraphError: For an invalid prefix or include entry.
    """
    for prefix, entry in (document.get("include") or {}).items():
        where = f"include '{prefix}'"
        if not STAGE_NAME.fullmatch(prefix):
            raise StageGraphError(f"{where}: a prefix is letters, digits and underscores")
        entry = _check_keys(where, entry, INCLUDE_KEYS)
        if "package" not in entry:
            raise StageGraphError(f"{where}: an include needs a 'package'")
        if not VARIABLE.search(entry["package"]):
            yield prefix, entry.get("bind") or {}, find_directory(entry["package"])


def overlay_names(directory: str, find_directory=package_stage_directory) -> set[str]:
    """The overlay names of a stage directory and of every robot it includes."""
    names = {
        os.path.splitext(os.path.basename(path))[0]
        for path in glob.glob(os.path.join(directory, "*.yaml"))
    } - {os.path.splitext(BASE_FILE)[0]}
    for _, _, child in includes(load_files(stage_files(directory)), find_directory):
        names |= overlay_names(child, find_directory)
    return names


def expand(
    document: Mapping,
    variables: Mapping[str, str] | None = None,
    find_directory=package_stage_directory,
    overlays: Sequence[str] = (),
) -> dict:
    """Replace the document's includes with the stages of the included robots.

    An include is keyed by its prefix: its stage ``core`` becomes ``<prefix>_core``, unless
    ``bind`` maps it to a stage of the including robot, which then takes its place. An included
    robot runs with the overlays the including robot runs with, those it has: a robot started in
    simulation runs its parts in simulation. The including robot's own stages are merged last,
    so that it can change any included stage.

    :param document: A merged document, see :func:`load_files`.
    :param variables: Values for ``${name}``, ``None`` to leave them in place (for listing names).
    :param find_directory: Resolves an include's ``package`` to its stage directory.
    :param overlays: Names of the overlays the including robot runs with.
    :returns: A document with only ``stages``.
    :raises StageGraphError: For a bad include entry, colliding names or a missing bind target.
    """
    if variables is not None:
        document = substitute(dict(document), variables)
    stages: dict = {}
    bound: set[str] = set()
    for prefix, bind, directory in includes(document, find_directory):
        child = expand(
            load_files(stage_files(directory, overlays)), variables, find_directory, overlays
        )
        bound.update(bind.values())

        def rename(name: str) -> str:
            """The name of an included stage in the including graph."""
            return bind.get(name, f"{prefix}_{name}")

        for name, data in child["stages"].items():
            if name in bind:
                continue
            if rename(name) in stages:
                raise StageGraphError(f"include '{prefix}': stage '{rename(name)}' exists twice")
            data = data or {}
            data["after"] = [rename(dependency) for dependency in data.get("after") or []]
            stages[rename(name)] = data
    expanded = merge({"stages": stages}, {"stages": document.get("stages") or {}})
    missing = sorted(bound - set(expanded["stages"]))
    if missing:
        raise StageGraphError(f"Included stages are bound to stages that do not exist: {missing}")
    return expanded


def substitute(value: Any, variables: Mapping[str, str]) -> Any:
    """Replace ``${name}`` in every string of a document with the value of that variable.

    :raises StageGraphError: When a string names a variable that is not given.
    """
    if isinstance(value, str):

        def replace(match: re.Match) -> str:
            """The value of the variable that ``match`` names."""
            if match.group(1) not in variables:
                raise StageGraphError(f"Unknown variable '${{{match.group(1)}}}'")
            return variables[match.group(1)]

        return VARIABLE.sub(replace, value)
    if isinstance(value, list):
        return [substitute(item, variables) for item in value]
    if isinstance(value, dict):
        return {key: substitute(item, variables) for key, item in value.items()}
    return value


def _check_keys(where: str, data: Any, allowed: set) -> dict:
    """Return ``data`` as a mapping, rejecting keys outside ``allowed`` so that typos fail early."""
    if not isinstance(data, dict):
        raise StageGraphError(f"{where}: expected a mapping, got {data!r}")
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise StageGraphError(f"{where}: unknown keys {unknown}, allowed are {sorted(allowed)}")
    return data


def _names(where: str, value: Any) -> tuple[str, ...]:
    """Return a list of names from the document, rejecting anything that is not a list of strings."""
    if value is None:
        return ()
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise StageGraphError(f"{where}: expected a list of names, got {value!r}")
    return tuple(value)


def _topic(where: str, entry: Any) -> TopicCheck:
    """Parse a topic entry, either a name or a mapping with ``name`` and ``max_age``."""
    if isinstance(entry, str):
        return TopicCheck(entry)
    entry = _check_keys(where, entry, TOPIC_KEYS)
    if "name" not in entry:
        raise StageGraphError(f"{where}: a topic needs a 'name'")
    max_age = entry.get("max_age")
    return TopicCheck(entry["name"], None if max_age is None else float(max_age))


def _stage(name: str, data: Any, required: str, timeout_scale: float) -> Stage:
    """Parse one stage from the merged document.

    :param required: ``"true"`` or ``"false"`` to override the file's ``required``, ``""`` to keep it.
    """
    where = f"stage '{name}'"
    data = _check_keys(where, data, STAGE_KEYS)
    if "timeout" not in data:
        raise StageGraphError(f"{where}: a stage needs a 'timeout'")
    delegate = _check_keys(f"{where} delegate", data.get("delegate") or {}, DELEGATE_KEYS)
    ready = _check_keys(f"{where} ready", data.get("ready") or {}, READY_KEYS)
    if not isinstance(data.get("required", True), bool):
        raise StageGraphError(
            f"{where}: 'required' must be true or false, got {data['required']!r}"
        )
    ros2_control = None
    if "ros2_control" in delegate:
        ros2_control = Ros2Control(
            **_check_keys(
                f"{where} ros2_control", delegate["ros2_control"] or {}, ROS2_CONTROL_KEYS
            )
        )
    return Stage(
        name=name,
        timeout=float(data["timeout"]) * timeout_scale,
        after=_names(f"{where} after", data.get("after")),
        required=data.get("required", True) if required == "" else required == "true",
        lifecycle_nodes=_names(f"{where} lifecycle_nodes", data.get("lifecycle_nodes")),
        nav2_managers=_names(f"{where} delegate nav2", delegate.get("nav2")),
        ros2_control=ros2_control,
        topics=tuple(_topic(f"{where} topic", entry) for entry in (ready.get("topics") or [])),
    )


def parse(
    document: Mapping,
    variables: Mapping[str, str] | None = None,
    required: Mapping[str, str] | None = None,
    timeout_scale: float = 1.0,
    find_directory=package_stage_directory,
    overlays: Sequence[str] = (),
) -> dict[str, Stage]:
    """Build and check the stage graph of a merged document.

    :param document: The result of :func:`load_files`.
    :param variables: Values for ``${name}`` in the document.
    :param required: Per stage ``"true"``, ``"false"`` or ``""`` (keep the file's value).
    :param timeout_scale: Factor applied to every stage timeout.
    :param find_directory: Resolves an include's ``package`` to its stage directory.
    :param overlays: Names of the overlays the document was loaded with, passed on to includes.
    :returns: The stages by name, in file order.
    :raises StageGraphError: For invalid names, unknown dependencies, cycles or bad entries.
    """
    if not timeout_scale > 0.0:
        raise StageGraphError(f"The timeout scale must be positive, got {timeout_scale}")
    document = expand(document, variables or {}, find_directory, overlays)
    required = required or {}
    # An empty override keeps the file's value, so only a set one needs its stage
    unknown_overrides = sorted(
        name for name, value in required.items() if value and name not in document["stages"]
    )
    if unknown_overrides:
        raise StageGraphError(f"Required overrides for stages not in the graph {unknown_overrides}")
    for override in required.values():
        if override not in ("", "true", "false"):
            raise StageGraphError(f"A required override must be true, false or empty: {override!r}")

    stages = {}
    for name, data in document["stages"].items():
        if not STAGE_NAME.fullmatch(name):
            raise StageGraphError(f"Stage names are letters, digits and underscores: '{name}'")
        stages[name] = _stage(name, data, required.get(name, ""), timeout_scale)
    for stage in stages.values():
        unknown = [dependency for dependency in stage.after if dependency not in stages]
        if unknown:
            raise StageGraphError(f"Stage '{stage.name}' comes after unknown stages {unknown}")
    topological_order(stages)
    return stages


def topological_order(stages: Mapping[str, Stage]) -> list[str]:
    """Order stages so that every stage comes after the stages it depends on.

    :raises StageGraphError: When the stages depend on each other in a cycle.
    """
    try:
        return list(TopologicalSorter({n: s.after for n, s in stages.items()}).static_order())
    except CycleError as error:
        cycle = " -> ".join(reversed(error.args[1]))
        raise StageGraphError(f"Stages depend on each other in a cycle: {cycle}") from None


def startable(stages: Mapping[str, Stage], status: Mapping[str, str]) -> list[str]:
    """Pending stages whose dependencies are all ready."""
    return [
        name
        for name, stage in stages.items()
        if status[name] == "pending" and all(status[dep] == "ready" for dep in stage.after)
    ]


def dependents(stages: Mapping[str, Stage], name: str) -> list[str]:
    """Every stage that needs ``name``, directly or through other stages, in topological order."""
    needed = {name}
    for other in topological_order(stages):
        if needed.intersection(stages[other].after):
            needed.add(other)
    return [other for other in topological_order(stages) if other in needed and other != name]


def stop_order(stages: Mapping[str, Stage], names: Sequence[str]) -> list[str]:
    """The given stages in the order to stop them: every stage before the stages it needs."""
    return [name for name in reversed(topological_order(stages)) if name in names]
