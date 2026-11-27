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

"""Prints the ROS log messages recorded in failure snapshots as text.

Usage: ros2 run duatic_monitoring failure_snapshot_logs <session dir or .mcap file>...
"""

import argparse
from datetime import datetime
from pathlib import Path

import rosbag2_py
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message

SEVERITY_NAMES = {
    Log.DEBUG: "DEBUG",
    Log.INFO: "INFO",
    Log.WARN: "WARN",
    Log.ERROR: "ERROR",
    Log.FATAL: "FATAL",
}


def format_log(msg: Log) -> str:
    stamp = datetime.fromtimestamp(msg.stamp.sec + msg.stamp.nanosec / 1e9)
    severity = SEVERITY_NAMES.get(msg.level, f"LVL{msg.level}")
    return f"[{stamp:%Y-%m-%d %H:%M:%S.%f}] [{severity}] [{msg.name}] {msg.msg.strip()}"


def snapshot_files(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    files = path.glob("snapshot/*.mcap") if (path / "snapshot").is_dir() else path.glob("*.mcap")
    # The newest file is the one the recorder holds open for the next snapshot.
    return sorted(files, key=lambda f: int(f.stem.rsplit("_", 1)[-1]))[:-1]


def read_logs(file: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(file), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/rosout"]))
    while reader.has_next():
        _, data, _ = reader.read_next()
        yield deserialize_message(data, Log)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("paths", nargs="+", type=Path, help="session directory or .mcap file")
    args = parser.parse_args()

    for path in args.paths:
        if (path / "events.txt").is_file():
            print((path / "events.txt").read_text().rstrip())
        for file in snapshot_files(path):
            print(f"=== {file}")
            for msg in read_logs(file):
                print(format_log(msg))


if __name__ == "__main__":
    main()
