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

"""Writes a rosbag2 snapshot to disk when asked to through its ~/trigger service.

A rosbag2 recorder in snapshot mode holds the most recent topic data in memory. Any node
that detects a failure calls ~/trigger with a reason, and this node records the reason in
the session's events.txt and has the recorder write its buffer.

Each launch records into its own session directory. The snapshots of all sessions are
kept within a disk budget, the oldest removed first.
"""

import fcntl
import re
import shutil
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rosbag2_interfaces.srv import Snapshot, SplitBagfile
from duatic_helper_msgs.srv import TriggerSnapshot

SESSION_NAME_FORMAT = "%Y-%m-%d_%H-%M-%S-%f"
SESSION_NAME_RE = re.compile(r"^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}-\d{6}$")
EVENTS_FILE = "events.txt"
LOCK_FILE = ".lock"
SNAPSHOT_BAG = "snapshot"
STATIC_BAG = "static"


def session_is_live(session: Path) -> bool:
    """Report whether a running launch holds the lock of a session directory."""
    try:
        with open(session / LOCK_FILE) as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return True
    except OSError:
        return False
    return False


def snapshot_files(session: Path) -> list[Path]:
    """List the files of a session's snapshot bag, oldest first."""
    files = (session / SNAPSHOT_BAG).glob("*.mcap")
    return sorted(files, key=lambda f: int(f.stem.rsplit("_", 1)[-1]))


def prune(root: Path, current: Path, max_bytes: int) -> tuple[list[Path], list[Path]]:
    """Delete old snapshots and sessions under root, and return the ones removed.

    Snapshots are removed oldest first until the rest fit in max_bytes. The newest file of
    each bag is the one the recorder holds open, or left empty when it stopped, so it is
    neither counted nor removed. A finished session is removed once it holds no snapshot,
    which includes one that never recorded an event. Only directories named like a session
    are touched, and never one a running launch is using.
    """
    sessions = sorted(d for d in root.iterdir() if d.is_dir() and SESSION_NAME_RE.match(d.name))
    snapshots = [f for d in sessions for f in snapshot_files(d)[:-1]]
    total = sum(f.stat().st_size for f in snapshots)
    removed_snapshots = []
    for snapshot in snapshots:
        if total <= max_bytes:
            break
        total -= snapshot.stat().st_size
        snapshot.unlink()
        removed_snapshots.append(snapshot)

    removed_sessions = [
        d
        for d in sessions
        if d.resolve() != current.resolve()
        and len(snapshot_files(d)) <= 1
        and not session_is_live(d)
    ]
    for session in removed_sessions:
        shutil.rmtree(session, ignore_errors=True)
    return removed_snapshots, removed_sessions


class FailureSnapshotNode(Node):
    """Writes the rosbag2 snapshot to disk when its trigger service is called."""

    def __init__(self):
        super().__init__("failure_snapshot")

        self.declare_parameter("session_dir", "")
        self.declare_parameter("snapshot_service", "/failure_snapshot_recorder/snapshot")
        self.declare_parameter("static_split_service", "/failure_static_recorder/split_bagfile")
        self.declare_parameter("max_disk_bytes", 20_000_000_000)

        session_dir = self.get_parameter("session_dir").value
        if not session_dir:
            raise ValueError("The session_dir parameter is required")
        self._session_dir = Path(session_dir).expanduser()
        self._max_disk_bytes = self.get_parameter("max_disk_bytes").value
        self._static_split_requested = False

        self._prune()

        # The trigger service awaits the recorder, so the recorder's answer must not wait
        # for the callback group the service occupies.
        recorder_group = ReentrantCallbackGroup()
        self._snapshot_client = self.create_client(
            Snapshot, self.get_parameter("snapshot_service").value, callback_group=recorder_group
        )
        # Splitting closes the file holding the latched topics, so it is readable even if the
        # recorder is killed later.
        self._static_split_client = self.create_client(
            SplitBagfile,
            self.get_parameter("static_split_service").value,
            callback_group=recorder_group,
        )
        self._trigger_service = self.create_service(TriggerSnapshot, "~/trigger", self._on_trigger)

    def _prune(self) -> None:
        """Keep the snapshots under the session root within the disk budget."""
        try:
            snapshots, sessions = prune(
                self._session_dir.parent, self._session_dir, self._max_disk_bytes
            )
        except OSError as e:
            self.get_logger().error(f"Cannot remove old snapshots: {e}")
            return
        if snapshots or sessions:
            self.get_logger().info(
                f"Removed {len(snapshots)} old snapshot(s) and {len(sessions)} session(s)"
            )

    def _record_event(self, reason: str) -> None:
        """Append a line to the session's event list."""
        stamp = datetime.now().isoformat(timespec="milliseconds")
        try:
            with open(self._session_dir / EVENTS_FILE, "a") as events:
                events.write(f"{stamp} {reason}\n")
        except OSError as e:
            self.get_logger().error(f"Could not record the event in {self._session_dir}: {e}")

    async def _on_trigger(self, request, response):
        """Record the reason and have the recorder write its buffer."""
        reason = request.reason or "no reason given"
        self.get_logger().warning(f"Writing a snapshot to {self._session_dir}: {reason}")
        self._record_event(reason)

        if not self._snapshot_client.service_is_ready():
            response.message = f"Snapshot service {self._snapshot_client.srv_name} is not available"
            self.get_logger().error(response.message)
            return response
        if not self._static_split_requested:
            if self._static_split_client.service_is_ready():
                self._static_split_client.call_async(SplitBagfile.Request())
                self._static_split_requested = True
            else:
                self.get_logger().warning(
                    f"{self._static_split_client.srv_name} is not available. "
                    "The latched topics are only saved on a clean shutdown."
                )

        result = await self._snapshot_client.call_async(Snapshot.Request())
        if not result.success:
            response.message = "The recorder failed to write the snapshot"
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = f"Snapshot written to {self._session_dir}"
        self.get_logger().info(response.message)
        self._prune()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FailureSnapshotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
