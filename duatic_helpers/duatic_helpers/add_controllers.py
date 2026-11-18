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
"""Combine a base controllers file with a file that only adds new controllers."""

import tempfile

import yaml

ROOT = "/**"
CONTROLLER_MANAGER = "controller_manager"
PARAMS = "ros__parameters"


def add_controllers(base_file: str, extra_file: str) -> str:
    """
    Add the controllers declared in an extra file to a base controllers file.

    Both files use the layout read by ``duatic_control``'s ``control.launch.py``. The extra file
    may only hold ``/**`` with new controller declarations under
    ``controller_manager.ros__parameters`` (each a mapping with a ``type``) and the
    ``ros__parameters`` blocks of those new controllers. It cannot change or remove anything
    the base file sets, so the base keeps its controllers, their parameters and the controller
    manager settings.

    :param base_file: Path to the base controllers file.
    :param extra_file: Path to the file with the new controllers, or empty for none.
    :returns: ``base_file`` itself when ``extra_file`` is empty, otherwise the path of a new
        temporary file holding the combined config. The temporary file is never deleted.
    :raises ValueError: If the extra file holds anything other than new controllers, or reuses
        a name the base file already declares or configures.
    """
    if not extra_file:
        return base_file

    with open(base_file) as f:
        combined = yaml.safe_load(f)
    with open(extra_file) as f:
        extra = yaml.safe_load(f)

    if not isinstance(extra, dict) or list(extra) != [ROOT] or not isinstance(extra[ROOT], dict):
        raise ValueError(f"{extra_file}: must hold only a '{ROOT}' mapping")

    root = combined[ROOT]
    declared = root[CONTROLLER_MANAGER][PARAMS]
    extra_blocks = dict(extra[ROOT])
    extra_manager = extra_blocks.pop(CONTROLLER_MANAGER, None)
    if not isinstance(extra_manager, dict) or list(extra_manager) != [PARAMS]:
        raise ValueError(
            f"{extra_file}: '{ROOT}.{CONTROLLER_MANAGER}' must hold only '{PARAMS}' with the new"
            " controllers"
        )
    new_controllers = extra_manager[PARAMS] or {}

    for name, declaration in new_controllers.items():
        if not isinstance(declaration, dict) or "type" not in declaration:
            raise ValueError(
                f"{extra_file}: '{name}' is not a controller declaration with a 'type'; settings"
                " of the base controller manager cannot be changed"
            )
    if clashes := new_controllers.keys() & (declared.keys() | root.keys()):
        raise ValueError(f"{extra_file}: {sorted(clashes)} already exist in {base_file}")
    if undeclared := extra_blocks.keys() - new_controllers.keys():
        raise ValueError(
            f"{extra_file}: {sorted(undeclared)} are not controllers declared in this file"
        )

    declared.update(new_controllers)
    root.update(extra_blocks)
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as output_file:
        yaml.safe_dump(combined, output_file, sort_keys=False)
    return output_file.name
