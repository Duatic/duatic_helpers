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

from typing import Any
import os
import xacro
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def simulator_not_subcomponent_condition(simulator: str) -> IfCondition:
    """
    Create a condition that holds for a given simulator when not started as a subcomponent.

    Evaluates the launch arguments 'start_as_subcomponent' and 'simulator' at launch time.

    :param simulator: Simulator name to compare the 'simulator' launch argument against.

    :returns: Condition that is true if 'start_as_subcomponent' is not 'true' (case-insensitive) and
        the 'simulator' launch argument equals the given simulator.
    """
    return IfCondition(
        PythonExpression(
            [
                "('",
                LaunchConfiguration("start_as_subcomponent"),
                "'.lower() != 'true') and ",
                "('",
                LaunchConfiguration("simulator"),
                "' == '",
                simulator,
                "')",
            ]
        )
    )


def process_robot_description(
    urdf_file_path: str, srdf_file_path: str = "", *, mappings: dict[str, Any] | None = None
) -> str:
    """
    Parses the URDF and SRDF xacro files into a common robot description.

    As the consumers of the robot description usually ignore unknown tags,
    appending the SRDF presents no issue.

    :param urdf_file_path: Path to the URDF file
    :param srdf_file_path: Path to the SRDF file (optional)
    :param mappings: Mappings to pass to the xacro files

    :returns: The combined robot description string
    :raises: FileNotFoundError for non-existing input files
    """
    if not os.path.exists(urdf_file_path):
        raise FileNotFoundError(f"Could not find URDF file at {urdf_file_path}")
    if srdf_file_path and not os.path.exists(srdf_file_path):
        raise FileNotFoundError(f"Could not find SRDF file at {srdf_file_path}")

    # Process URDF
    urdf_doc = xacro.process_file(urdf_file_path, mappings=mappings)

    if srdf_file_path:
        # Process SRDF
        srdf_doc = xacro.process_file(srdf_file_path, mappings=mappings)

        # Append SRDF to URDF
        urdf_root = urdf_doc.documentElement
        for node in list(srdf_doc.documentElement.childNodes):
            urdf_root.appendChild(urdf_doc.importNode(node, deep=True))

    return urdf_doc.toxml()
