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

from pathlib import Path
from typing import Any

import pytest
import yaml

from duatic_helpers import add_controllers

BASE = {
    "/**": {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": 1000,
                "base_controller": {"type": "pkg/Base", "state": "active"},
            }
        },
        "base_controller": {"ros__parameters": {"joints": ["a", "b"]}},
    }
}

NEW_CONTROLLER = {
    "/**": {
        "controller_manager": {
            "ros__parameters": {"new_controller": {"type": "pkg/New", "state": "inactive"}}
        },
        "new_controller": {"ros__parameters": {"joints": ["c"]}},
    }
}


def manager(**params: Any) -> dict[str, Any]:
    """
    Build an extra file that holds only controller manager entries.

    :param params: Entries for ``/**.controller_manager.ros__parameters``.
    :returns: The file content.
    """
    return {"/**": {"controller_manager": {"ros__parameters": params}}}


def write_yaml(path: Path, data: Any) -> str:
    """
    Write data as YAML.

    :param path: File to create.
    :param data: Content to serialize.
    :returns: The path as a string.
    """
    path.write_text(yaml.safe_dump(data))
    return str(path)


def run(tmp_path: Path, extra: Any) -> dict[str, Any]:
    """
    Add the controllers of extra to BASE and load the result.

    :param tmp_path: Directory for the input files.
    :param extra: Content of the extra controllers file.
    :returns: The parsed combined file.
    """
    base_file = write_yaml(tmp_path / "base.yaml", BASE)
    extra_file = write_yaml(tmp_path / "extra.yaml", extra)
    with open(add_controllers(base_file, extra_file)) as f:
        return yaml.safe_load(f)  # type: ignore[no-any-return]


def test_empty_extra_returns_base_path(tmp_path: Path) -> None:
    base_file = write_yaml(tmp_path / "base.yaml", BASE)
    assert add_controllers(base_file, "") == base_file


def test_adds_new_controller(tmp_path: Path) -> None:
    result = run(tmp_path, NEW_CONTROLLER)

    cm_params = result["/**"]["controller_manager"]["ros__parameters"]
    assert cm_params["update_rate"] == 1000
    assert cm_params["base_controller"] == {"type": "pkg/Base", "state": "active"}
    assert cm_params["new_controller"] == {"type": "pkg/New", "state": "inactive"}
    assert result["/**"]["base_controller"] == {"ros__parameters": {"joints": ["a", "b"]}}
    assert result["/**"]["new_controller"] == {"ros__parameters": {"joints": ["c"]}}


def test_new_controller_without_parameter_block(tmp_path: Path) -> None:
    result = run(tmp_path, manager(new_controller={"type": "pkg/New"}))

    assert "new_controller" in result["/**"]["controller_manager"]["ros__parameters"]


@pytest.mark.parametrize(
    "extra",
    [
        pytest.param(None, id="empty_file"),
        pytest.param({"/arm/**": NEW_CONTROLLER["/**"]}, id="other_namespace"),
        pytest.param({"/**": {"new_controller": {"ros__parameters": {}}}}, id="no_declarations"),
        pytest.param(manager(update_rate=500), id="change_update_rate"),
        pytest.param(
            manager(base_controller={"type": "pkg/Other"}), id="redeclare_base_controller"
        ),
        pytest.param(
            manager(base_controller={"state": "inactive"}), id="change_base_controller_state"
        ),
        pytest.param(
            {
                "/**": {
                    **NEW_CONTROLLER["/**"],
                    "base_controller": {"ros__parameters": {"extra_param": 1}},
                }
            },
            id="add_parameter_to_base_controller",
        ),
        pytest.param(
            {
                "/**": {
                    **NEW_CONTROLLER["/**"],
                    "undeclared_controller": {"ros__parameters": {}},
                }
            },
            id="parameters_of_undeclared_controller",
        ),
    ],
)
def test_rejects_changes_to_base(tmp_path: Path, extra: Any) -> None:
    with pytest.raises(ValueError):
        run(tmp_path, extra)
