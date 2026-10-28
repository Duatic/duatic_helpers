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
import yaml

if TYPE_CHECKING:
    from collections.abc import Generator
    from typing import TypeAlias

YamlValue: TypeAlias = str | int | float | bool


class DictItemReference:
    """Reference to a single entry of a dictionary, allowing the entry's value to be replaced."""

    def __init__(self, dictionary: dict[str, YamlValue], key: str) -> None:
        """
        Construct the reference.

        Args:
            dictionary: The dictionary containing the entry.
            key: The key of the entry in the dictionary.
        """
        self.dictionary = dictionary
        self.dictKey = key

    def key(self) -> str:
        """Return the key of the referenced entry."""
        return self.dictKey

    def setValue(self, value: YamlValue) -> None:
        """
        Replace the value of the referenced entry in the dictionary.

        Args:
            value: The new value.
        """
        self.dictionary[self.dictKey] = value


class RewrittenYaml(launch.Substitution):
    """
    Substitution that modifies the given YAML file.

    Used in launch system
    """

    def __init__(
        self,
        source_file: launch.SomeSubstitutionsType,
        param_rewrites: dict[str, launch.SomeSubstitutionsType],
        root_key: launch.SomeSubstitutionsType | None = None,
        key_rewrites: dict[str, launch.SomeSubstitutionsType] | None = None,
        value_rewrites: dict[str, launch.SomeSubstitutionsType] | None = None,
        convert_types: bool = False,
    ) -> None:
        """
        Construct the substitution.

        Args:
            source_file: The original YAML file to modify.
            param_rewrites: Mappings from a parameter key (or dotted path) to its new value.
            root_key: If provided, the contents are placed under this key.
            key_rewrites: Mappings from a key to its replacement key.
            value_rewrites: Mappings from a value to its replacement value.
            convert_types: Whether to attempt converting the string to a number or boolean.
        """
        super().__init__()

        # import here to avoid loop
        from launch.utilities import normalize_to_list_of_substitutions

        self.__source_file: list[launch.Substitution] = normalize_to_list_of_substitutions(
            source_file
        )
        self.__param_rewrites: dict[str, list[launch.Substitution]] = {}
        self.__key_rewrites: dict[str, list[launch.Substitution]] = {}
        self.__value_rewrites: dict[str, list[launch.Substitution]] = {}
        self.__convert_types = convert_types
        self.__root_key: list[launch.Substitution] | None = None
        for key in param_rewrites:
            self.__param_rewrites[key] = normalize_to_list_of_substitutions(param_rewrites[key])
        if key_rewrites is not None:
            for key in key_rewrites:
                self.__key_rewrites[key] = normalize_to_list_of_substitutions(key_rewrites[key])
        if value_rewrites is not None:
            for value in value_rewrites:
                self.__value_rewrites[value] = normalize_to_list_of_substitutions(
                    value_rewrites[value]
                )
        if root_key is not None:
            self.__root_key = normalize_to_list_of_substitutions(root_key)

    @property
    def name(self) -> list[launch.Substitution]:
        """Getter for name."""
        return self.__source_file

    def describe(self) -> str:
        """Return a description of this substitution as a string."""
        return ""

    def perform(self, context: launch.LaunchContext) -> str:
        """
        Write a rewritten copy of the source YAML file.

        Args:
            context: Launch context used to resolve the file name and the rewrites.

        Returns:
            Path of the temporary file containing the rewritten YAML.
        """
        yaml_filename = launch.utilities.perform_substitutions(context, self.name)
        rewritten_yaml = tempfile.NamedTemporaryFile(mode="w", delete=False)
        param_rewrites, keys_rewrites, value_rewrites = self.resolve_rewrites(context)

        with open(yaml_filename) as yaml_file:
            data = yaml.safe_load(yaml_file)

        self.substitute_params(data, param_rewrites)
        self.add_params(data, param_rewrites)
        self.substitute_keys(data, keys_rewrites)
        self.substitute_values(data, value_rewrites)
        if self.__root_key is not None:
            root_key = launch.utilities.perform_substitutions(context, self.__root_key)
            if root_key:
                data = {root_key: data}
        yaml.dump(data, rewritten_yaml)
        rewritten_yaml.close()
        return rewritten_yaml.name

    def resolve_rewrites(
        self, context: launch.LaunchContext
    ) -> tuple[dict[str, str], dict[str, str], dict[str, str]]:
        """
        Resolve the parameter, key and value rewrites to plain strings.

        Args:
            context: Launch context used to resolve the substitutions.

        Returns:
            The resolved parameter rewrites, key rewrites and value rewrites, in this order.
        """
        resolved_params = {}
        for key in self.__param_rewrites:
            resolved_params[key] = launch.utilities.perform_substitutions(
                context, self.__param_rewrites[key]
            )
        resolved_keys = {}
        for key in self.__key_rewrites:
            resolved_keys[key] = launch.utilities.perform_substitutions(
                context, self.__key_rewrites[key]
            )
        resolved_values = {}
        for value in self.__value_rewrites:
            resolved_values[value] = launch.utilities.perform_substitutions(
                context, self.__value_rewrites[value]
            )
        return resolved_params, resolved_keys, resolved_values

    def substitute_params(self, yaml: dict[str, YamlValue], param_rewrites: dict[str, str]) -> None:
        """
        Replace existing parameters in place, matched by leaf key or by full dotted path.

        Args:
            yaml: The YAML data to modify.
            param_rewrites: Mappings from a leaf key or dotted path to its new (string) value.
        """
        # substitute leaf-only parameters
        for key in self.getYamlLeafKeys(yaml):
            if key.key() in param_rewrites:
                raw_value = param_rewrites[key.key()]
                key.setValue(self.convert(raw_value))

        # substitute total path parameters
        yaml_paths = self.pathify(yaml)
        for path in yaml_paths:
            if path in param_rewrites:
                # this is an absolute path (ex. 'key.keyA.keyB.val')
                rewrite_val = self.convert(param_rewrites[path])
                yaml_keys = path.split(".")
                yaml = self.updateYamlPathVals(yaml, yaml_keys, rewrite_val)

    def add_params(self, yaml: dict[str, YamlValue], param_rewrites: dict[str, str]) -> None:
        """
        Add parameters in place that are not yet present, if their path has 'ros__parameters'.

        Args:
            yaml: The YAML data to modify.
            param_rewrites: Mappings from a dotted path to its (string) value.
        """
        # add new total path parameters
        yaml_paths = self.pathify(yaml)
        for path in param_rewrites:
            if not path in yaml_paths:  # noqa: E713
                new_val = self.convert(param_rewrites[path])
                yaml_keys = path.split(".")
                if "ros__parameters" in yaml_keys:
                    yaml = self.updateYamlPathVals(yaml, yaml_keys, new_val)

    def substitute_values(self, yaml: dict[str, YamlValue], value_rewrites: dict[str, str]) -> None:
        """
        Replace values in place, recursing into nested dictionaries and lists.

        Args:
            yaml: The YAML data to modify.
            value_rewrites: Mappings from a value (compared as string) to its new (string) value.
        """

        def process_value(value: YamlValue) -> YamlValue:
            if isinstance(value, dict):
                for k, v in list(value.items()):
                    value[k] = process_value(v)
                return value
            elif isinstance(value, list):
                return [process_value(v) for v in value]
            elif str(value) in value_rewrites:
                return self.convert(value_rewrites[str(value)])
            return value

        for key in list(yaml.keys()):
            yaml[key] = process_value(yaml[key])

    def updateYamlPathVals(
        self, yaml: dict[str, YamlValue], yaml_key_list: list[str], rewrite_val: YamlValue
    ) -> dict[str, YamlValue]:
        """
        Set the value at a nested path, creating intermediate dictionaries as needed.

        Consumes the path: the visited keys are popped off the given list.

        Args:
            yaml: The YAML data (or the nested part of it) to modify.
            yaml_key_list: The path of keys leading to the value.
            rewrite_val: The value to set at the end of the path.

        Returns:
            The modified YAML data.
        """
        for key in yaml_key_list:
            if key == yaml_key_list[-1]:
                yaml[key] = rewrite_val
                break
            key = yaml_key_list.pop(0)
            if isinstance(yaml, list):
                yaml[int(key)] = self.updateYamlPathVals(yaml[int(key)], yaml_key_list, rewrite_val)
            else:
                yaml[key] = self.updateYamlPathVals(  # type: ignore[assignment]
                    yaml.get(key, {}), yaml_key_list, rewrite_val  # type: ignore[arg-type]
                )
        return yaml

    def substitute_keys(self, yaml: dict[str, YamlValue], key_rewrites: dict[str, str]) -> None:
        """
        Rename keys in place, recursing into nested dictionaries.

        Args:
            yaml: The YAML data to modify.
            key_rewrites: Mappings from a key to its replacement key.
        """
        if len(key_rewrites) != 0:
            for key in list(yaml.keys()):
                val = yaml[key]
                if key in key_rewrites:
                    new_key = key_rewrites[key]
                    yaml[new_key] = yaml[key]
                    del yaml[key]
                if isinstance(val, dict):
                    self.substitute_keys(val, key_rewrites)

    def getYamlLeafKeys(self, yamlData: dict[str, YamlValue]) -> Generator[DictItemReference]:
        """
        Iterate over all entries of a nested dictionary, including entries holding dictionaries.

        Args:
            yamlData: The YAML data to traverse.

        Yields:
            A reference to each entry, nested entries first.
        """
        if not isinstance(yamlData, dict):
            return

        for key in yamlData.keys():
            child = yamlData[key]

            if isinstance(child, dict):
                # Recursively process nested dictionaries
                yield from self.getYamlLeafKeys(child)

            yield DictItemReference(yamlData, key)

    def pathify(
        self,
        d: dict[str, YamlValue] | list[YamlValue] | YamlValue,
        p: str | None = None,
        paths: dict[str, YamlValue] | None = None,
        joinchar: str = ".",
    ) -> dict[str, YamlValue]:
        """
        Flatten nested YAML data into a mapping from dotted path to leaf value.

        List entries are addressed by their index, e.g. 'a.b.0.c'.

        Args:
            d: The YAML data to flatten.
            p: The path prefix of the data. Leave unset when calling from outside.
            paths: The mapping to fill. Leave unset when calling from outside.
            joinchar: The character joining the path elements.

        Returns:
            Mapping from the path of each leaf to its value.
        """
        if p is None:
            paths = {}
            self.pathify(d, "", paths, joinchar=joinchar)
            return paths

        assert paths is not None
        pn = p
        if p != "":
            pn += joinchar
        if isinstance(d, dict):
            for k in d:
                v = d[k]
                self.pathify(v, str(pn) + str(k), paths, joinchar=joinchar)
        elif isinstance(d, list):
            for idx, e in enumerate(d):
                self.pathify(e, pn + str(idx), paths, joinchar=joinchar)
        else:
            paths[p] = d
        return paths

    def convert(self, text_value: str) -> YamlValue:
        """
        Convert a string to a number or boolean where possible.

        Numbers are only converted if type conversion was enabled. 'true' and 'false'
        (case-insensitive) are always converted to booleans.

        Args:
            text_value: The string to convert.

        Returns:
            The int, float or bool the string represents, otherwise the unchanged string.
        """
        if self.__convert_types:
            # try converting to int or float
            try:
                return float(text_value) if "." in text_value else int(text_value)
            except ValueError:
                pass

        # try converting to bool
        if text_value.lower() == "true":
            return True
        if text_value.lower() == "false":
            return False

        # nothing else worked so fall through and return text
        return text_value
