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

import tempfile
from typing import IO, Optional

import launch


class ReplaceString(launch.Substitution):
    """
    Substitution that replaces strings on a given file.

    Used in launch system
    """

    def __init__(
        self,
        source_file: launch.SomeSubstitutionsType,
        replacements: dict[str, launch.SomeSubstitutionsType],
        condition: Optional[launch.Condition] = None,
    ) -> None:
        super().__init__()

        from launch.utilities import normalize_to_list_of_substitutions

        # import here to avoid loop

        self.__source_file: list[launch.Substitution] = \
            normalize_to_list_of_substitutions(source_file)
        self.__replacements = {}
        for key in replacements:
            self.__replacements[key] = normalize_to_list_of_substitutions(
                replacements[key]
            )
        self.__condition = condition

    @property
    def name(self) -> list[launch.Substitution]:
        """Getter for name."""
        return self.__source_file

    @property
    def condition(self) -> Optional[launch.Condition]:
        """Getter for condition."""
        return self.__condition

    def describe(self) -> str:
        """Return a description of this substitution as a string."""
        return ''

    def perform(self, context: launch.LaunchContext) -> str:
        yaml_filename: str = launch.utilities.perform_substitutions(
            context, self.name
        )
        if self.__condition is None or self.__condition.evaluate(context):
            output_file = tempfile.NamedTemporaryFile(mode='w', delete=False)
            replacements = self.resolve_replacements(context)
            try:
                input_file = open(yaml_filename)
                self.replace(input_file, output_file, replacements)
            except Exception as err:  # noqa: B902
                print('ReplaceString substitution error: ', err)
            finally:
                input_file.close()
                output_file.close()
            return output_file.name
        else:
            return yaml_filename

    def resolve_replacements(self, context: launch.LaunchContext) -> dict[str, str]:
        resolved_replacements = {}
        for key in self.__replacements:
            resolved_replacements[key] = launch.utilities.perform_substitutions(
                context, self.__replacements[key]
            )
        return resolved_replacements

    def replace(self, input_file: IO[str], output_file: IO[str],
                replacements: dict[str, str]) -> None:
        for line in input_file:
            for key, value in replacements.items():
                if isinstance(key, str) and isinstance(value, str):
                    if key in line:
                        line = line.replace(key, value)
                else:
                    raise TypeError(
                        'A provided replacement pair is not a string. Both key and value should be'
                        'strings.'
                    )
            output_file.write(line)
