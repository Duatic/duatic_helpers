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

from rcl_interfaces.msg import Log

from duatic_monitoring.failure_snapshot_logs import format_log, snapshot_files


def test_format_log():
    msg = Log(level=Log.ERROR, name="robot.driver", msg="drive fault\n")
    msg.stamp.sec = 0
    line = format_log(msg)
    assert line.endswith("] [ERROR] [robot.driver] drive fault")


def test_snapshot_files_skip_the_open_file(tmp_path):
    bag = tmp_path / "snapshot"
    bag.mkdir()
    for n in (10, 2, 1):
        (bag / f"snapshot_{n}.mcap").touch()
    names = [f.name for f in snapshot_files(tmp_path)]
    assert names == ["snapshot_1.mcap", "snapshot_2.mcap"]
    assert snapshot_files(bag / "snapshot_10.mcap") == [bag / "snapshot_10.mcap"]
