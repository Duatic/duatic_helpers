# Copyright (c) 2019 Intel Corporation
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

from __future__ import annotations

import tempfile
from typing import TYPE_CHECKING

import launch

if TYPE_CHECKING:
    from typing import IO


class ReplaceString(launch.Substitution):
    """
    Substitution that replaces strings on a given file.

    Used in launch system
    """

    def __init__(
        self,
        source_file: launch.SomeSubstitutionsType,
        replacements: dict[str, launch.SomeSubstitutionsType],
        condition: launch.Condition | None = None,
    ) -> None:
        """
        Construct the substitution.

        Args:
            source_file: The file in which to replace strings.
            replacements: Mapping from the string to find to the string to replace it with.
            condition: If given and it evaluates to false, the source file is used unchanged.
        """
        super().__init__()

        from launch.utilities import normalize_to_list_of_substitutions

        # import here to avoid loop

        self.__source_file: list[launch.Substitution] = normalize_to_list_of_substitutions(
            source_file
        )
        self.__replacements: dict[str, list[launch.Substitution]] = {}
        for key in replacements:
            self.__replacements[key] = normalize_to_list_of_substitutions(replacements[key])
        self.__condition = condition

    @property
    def name(self) -> list[launch.Substitution]:
        """Getter for name."""
        return self.__source_file

    @property
    def condition(self) -> launch.Condition | None:
        """Getter for condition."""
        return self.__condition

    def describe(self) -> str:
        """Return a description of this substitution as a string."""
        return ""

    def perform(self, context: launch.LaunchContext) -> str:
        """
        Write a copy of the source file with the replacements applied.

        Args:
            context: Launch context used to resolve the file name and the replacements.

        Returns:
            Path of the temporary file with the replacements applied, or the path of the
            source file if the condition evaluates to false.
        """
        yaml_filename: str = launch.utilities.perform_substitutions(context, self.name)
        if self.__condition is None or self.__condition.evaluate(context):
            output_file = tempfile.NamedTemporaryFile(mode="w", delete=False)
            replacements = self.resolve_replacements(context)
            try:
                input_file = open(yaml_filename)
                self.replace(input_file, output_file, replacements)
            except Exception as err:  # noqa: B902
                print("ReplaceString substitution error: ", err)
            finally:
                input_file.close()
                output_file.close()
            return output_file.name
        else:
            return yaml_filename

    def resolve_replacements(self, context: launch.LaunchContext) -> dict[str, str]:
        """
        Resolve the replacement substitutions to plain strings.

        Args:
            context: Launch context used to resolve the substitutions.

        Returns:
            Mapping from the string to find to its resolved replacement.
        """
        resolved_replacements = {}
        for key in self.__replacements:
            resolved_replacements[key] = launch.utilities.perform_substitutions(
                context, self.__replacements[key]
            )
        return resolved_replacements

    def replace(
        self, input_file: IO[str], output_file: IO[str], replacements: dict[str, str]
    ) -> None:
        """
        Copy the input file line by line to the output file, applying the replacements.

        Args:
            input_file: File to read from.
            output_file: File to write the result to.
            replacements: Mapping from the string to find to the string to replace it with.

        Raises:
            TypeError: If a key or value of the replacements is not a string.
        """
        for line in input_file:
            for key, value in replacements.items():
                if isinstance(key, str) and isinstance(value, str):
                    if key in line:
                        line = line.replace(key, value)
                else:
                    raise TypeError(
                        "A provided replacement pair is not a string. Both key and value should be"
                        "strings."
                    )
            output_file.write(line)
