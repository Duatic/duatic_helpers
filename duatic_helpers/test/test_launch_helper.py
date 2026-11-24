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
import xml.etree.ElementTree as ET

from duatic_helpers import process_robot_description
import pytest

URDF_XACRO = """\
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test_robot">
  <xacro:arg name="prefix" default="default_"/>
  <link name="$(arg prefix)base_link"/>
  <link name="$(arg prefix)tip_link"/>
  <joint name="$(arg prefix)joint1" type="revolute">
    <parent link="$(arg prefix)base_link"/>
    <child link="$(arg prefix)tip_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""

SRDF_XACRO = """\
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test_robot">
  <xacro:arg name="prefix" default="default_"/>
  <group name="arm">
    <joint name="$(arg prefix)joint1"/>
  </group>
</robot>
"""


@pytest.fixture
def urdf_path(tmp_path: Path) -> str:
    path = tmp_path / "robot.urdf.xacro"
    path.write_text(URDF_XACRO)
    return str(path)


@pytest.fixture
def srdf_path(tmp_path: Path) -> str:
    path = tmp_path / "robot.srdf.xacro"
    path.write_text(SRDF_XACRO)
    return str(path)


def test_merge_urdf_and_srdf_with_mappings(urdf_path: str, srdf_path: str) -> None:
    prefix = "left_"
    description = process_robot_description(urdf_path, srdf_path, mappings={"prefix": prefix})

    # Parse the result like a robot description consumer would
    root = ET.fromstring(description)
    assert root.tag == "robot"
    assert root.get("name") == "test_robot"

    # URDF content with mapping applied
    assert {link.get("name") for link in root.findall("link")} == {
        f"{prefix}base_link",
        f"{prefix}tip_link",
    }
    assert root.find("joint").get("name") == f"{prefix}joint1"

    # SRDF content appended to the URDF robot tag, with mapping applied
    group = root.find("group")
    assert group is not None
    assert group.get("name") == "arm"
    assert group.find("joint").get("name") == f"{prefix}joint1"


def test_urdf_only(urdf_path: str) -> None:
    prefix = "right_"
    description = process_robot_description(urdf_path, mappings={"prefix": prefix})

    root = ET.fromstring(description)
    assert root.get("name") == "test_robot"
    assert {link.get("name") for link in root.findall("link")} == {
        f"{prefix}base_link",
        f"{prefix}tip_link",
    }
    assert root.find("group") is None


class TestMissingFiles:
    def test_missing_urdf(self, tmp_path: Path, srdf_path: str) -> None:
        with pytest.raises(FileNotFoundError, match="URDF"):
            process_robot_description(str(tmp_path / "missing.urdf.xacro"), srdf_path)

    def test_missing_srdf(self, urdf_path: str, tmp_path: Path) -> None:
        with pytest.raises(FileNotFoundError, match="SRDF"):
            process_robot_description(urdf_path, str(tmp_path / "missing.srdf.xacro"))


def test_extra_mappings_are_ignored(urdf_path: str, srdf_path: str) -> None:
    description = process_robot_description(
        urdf_path, srdf_path, mappings={"prefix": "left_", "undeclared_arg": "ignored"}
    )

    root = ET.fromstring(description)
    assert root.find("joint").get("name") == "left_joint1"
    assert "undeclared_arg" not in description
