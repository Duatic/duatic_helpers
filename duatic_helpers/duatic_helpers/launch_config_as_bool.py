# Copyright (c) 2025 Nishalan Govender
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions


class LaunchConfigAsBool(launch.Substitution):
    """
    Converts a LaunchConfiguration value into a normalized boolean string: 'True' or 'False'.

    Values like 'true', '1', 'yes' and 'on' (case-insensitive) map to 'True'. Everything else,
    including 'false', '0', 'no' and the empty string, maps to 'False'. The result is meant for
    use in PythonExpression and IfCondition contexts.
    """

    def __init__(self, name: str) -> None:
        """
        Construct the substitution.

        Args:
            name: Name of the launch configuration to read.
        """
        super().__init__()
        self._config = LaunchConfiguration(name)

    def perform(self, context: LaunchContext) -> str:
        """
        Resolve the launch configuration and normalize it.

        Args:
            context: Launch context used to resolve the configuration.

        Returns:
            'True' if the value is truthy, otherwise 'False'.
        """
        value = perform_substitutions(context, [self._config])
        if value.strip().lower() in ["true", "1", "yes", "on"]:
            return "True"
        return "False"

    def describe(self) -> str:
        """Return a description of this substitution as a string."""
        return f"LaunchConfigAsBool({self._config.describe()})"
