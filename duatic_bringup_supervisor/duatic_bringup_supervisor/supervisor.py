# Copyright 2026 Duatic AG
# SPDX-License-Identifier: BSD-3-Clause

"""Bring a robot up stage by stage, watch it while it runs, and stop it in reverse order.

Parameters:

- ``stage_files``: the base stage file, then the overlays in the order they apply
- ``timeout_scale``: factor applied to every stage timeout
- ``require.<stage>``: ``true`` or ``false`` overrides the stage's ``required``, empty keeps it
- ``vars.<name>``: value for ``${name}`` in the stage files
- ``watch_period``: seconds between the checks of stages that are up

The node publishes the state of every stage as diagnostics, and ``~/shutdown`` stops all stages
in reverse dependency order before the node exits. The node exits with code 1 when a required
stage fails, at startup or later.
"""

import sys
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from duatic_bringup_supervisor import graph
from duatic_bringup_supervisor.units import POLL_PERIOD, StageError, units_of

STOP_TIMEOUT = 30.0
# Consecutive failed watch cycles before a ready stage counts as failed, so that one slow
# service answer does not stop the robot
WATCH_FAILURES = 3
LEVELS = {
    "ready": DiagnosticStatus.OK,
    "pending": DiagnosticStatus.WARN,
    "starting": DiagnosticStatus.WARN,
    "stopping": DiagnosticStatus.WARN,
    "stopped": DiagnosticStatus.WARN,
}


class Supervisor(Node):
    """Starts the stages of a stage graph and keeps track of their state."""

    def __init__(self):
        """Load the stage graph from the parameters and prepare every stage's parts."""
        super().__init__("bringup_supervisor", automatically_declare_parameters_from_overrides=True)
        self.group = ReentrantCallbackGroup()
        files = self.get_parameter_or("stage_files", None)
        if files is None or not files.value:
            raise graph.StageGraphError("The 'stage_files' parameter names no stage file")
        self.stages = graph.parse(
            graph.load_files(files.value),
            variables=self._by_prefix("vars"),
            # Launch turns "true" into a boolean parameter, so compare in lower case
            required={k: v.lower() for k, v in self._by_prefix("require").items()},
            timeout_scale=float(self._parameter("timeout_scale", "1.0")),
        )
        self.watch_period = float(self._parameter("watch_period", 1.0))
        # Set when a stage is stopped, so that a start still waiting in its thread ends early
        self.cancel = {name: threading.Event() for name in self.stages}
        self.units = {
            name: units_of(self, stage, self.cancel[name]) for name, stage in self.stages.items()
        }
        self.status = {name: "pending" for name in self.stages}
        self.reason = {name: "" for name in self.stages}
        self.lock = threading.RLock()
        self.stop_lock = threading.Lock()
        self.threads: dict[str, threading.Thread] = {}
        self.stopped: set[str] = set()
        self.stop_requested = threading.Event()
        self.exit_code = 0
        self.diagnostics = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self.create_service(Trigger, "~/shutdown", self._on_shutdown, callback_group=self.group)
        self.get_logger().info(
            "Stages in start order: " + ", ".join(graph.topological_order(self.stages))
        )

    def _parameter(self, name: str, default):
        """The value of an optional parameter."""
        return self.get_parameter_or(name, Parameter(name, value=default)).value

    def _by_prefix(self, prefix: str) -> dict[str, str]:
        """Parameters ``<prefix>.<key>`` as a mapping from key to string value."""
        return {
            key: str(param.value) for key, param in self.get_parameters_by_prefix(prefix).items()
        }

    def _on_shutdown(self, _request, response):
        """Start an ordered stop; the node exits once every stage has stopped."""
        self.get_logger().info("Shutdown requested, stopping the stages in reverse order")
        self.stop_requested.set()
        response.success = True
        response.message = "Stopping the stages in reverse dependency order"
        return response

    def _set(self, name: str, status: str, reason: str | None = None) -> None:
        """Change a stage's state, and its reason when one is given, then publish."""
        with self.lock:
            self.status[name] = status
            if reason is not None:
                self.reason[name] = reason
        self.publish()

    def start_ready(self) -> None:
        """Start every pending stage whose dependencies are ready, each in its own thread."""
        with self.lock:
            if self.stop_requested.is_set():
                return
            names = graph.startable(self.stages, self.status)
            for name in names:
                self.status[name] = "starting"
        for name in names:
            self.get_logger().info(f"Starting stage '{name}'")
            self.threads[name] = threading.Thread(target=self._run_stage, args=(name,), daemon=True)
            self.threads[name].start()
        if names:
            self.publish()

    def _run_stage(self, name: str) -> None:
        """Start a stage's parts in order, then wait until all of them report up."""
        stage = self.stages[name]
        deadline = time.monotonic() + stage.timeout
        try:
            for unit in self.units[name]:
                if self.status[name] != "starting":
                    return
                unit.start(deadline)
            while True:
                if self.status[name] != "starting":
                    return
                problems = [p for unit in self.units[name] if (p := unit.problem())]
                if not problems:
                    break
                if time.monotonic() > deadline:
                    raise StageError("; ".join(problems))
                self.cancel[name].wait(POLL_PERIOD)
        except StageError as error:
            self.fail(name, f"did not come up within {stage.timeout:.0f} s: {error}")
            return
        except Exception as error:
            self.fail(name, f"could not be started: {error!r}")
            return
        with self.lock:
            # Checked and set together, so that a stop in between cannot be overwritten
            if self.status[name] != "starting":
                return
            self._set(name, "ready")
        self.get_logger().info(f"Stage '{name}' is ready")
        self.start_ready()

    def fail(self, name: str, reason: str) -> None:
        """Handle a failed stage.

        A required stage stops the whole robot. An optional one is stopped together with the
        stages that need it, and the rest keeps running.
        """
        with self.lock:
            # After an outside shutdown every node is going away, which is no stage failure
            if self.status[name] not in ("starting", "ready") or not self.context.ok():
                return
            self.status[name] = "failed"
            self.reason[name] = reason
            affected = graph.dependents(self.stages, name)
        self.get_logger().error(f"Stage '{name}' {reason}")
        if self.stages[name].required:
            self.get_logger().error(f"Stage '{name}' is required, stopping the robot")
            self.exit_code = 1
            self.stop_requested.set()
            return
        self.get_logger().warn(
            f"Stage '{name}' is optional, continuing without it"
            + (f" and without {affected}" if affected else "")
        )
        to_stop = [name]
        for other in affected:
            with self.lock:
                if self.status[other] == "pending":
                    self.status[other] = "skipped"
                    self.reason[other] = f"needs stage '{name}'"
                elif self.status[other] in ("starting", "ready"):
                    to_stop.append(other)
        self.publish()
        self._stop(to_stop)

    def _stop(self, names: list[str]) -> None:
        """Stop stages so that no stage stops before the stages that need it.

        A stage that is still starting is cancelled and stopped only once its start has ended,
        so that it cannot activate anything after its stop. Each stage is stopped once.
        """
        with self.stop_lock:
            for name in graph.stop_order(self.stages, names):
                if name not in self.stopped:
                    self._stop_stage(name)

    def _stop_stage(self, name: str) -> None:
        """Stop one stage's parts in reverse start order; a failed stage stays failed."""
        final_status = "failed" if self.status[name] == "failed" else "stopped"
        self._set(name, "stopping")
        self.cancel[name].set()
        thread = self.threads.get(name)
        if thread is not None and thread is not threading.current_thread():
            thread.join()
        for unit in reversed(self.units[name]):
            try:
                unit.stop(STOP_TIMEOUT)
            except StageError as error:
                self.get_logger().error(f"Stopping stage '{name}': {error}")
        self._set(name, final_status)
        self.stopped.add(name)
        self.get_logger().info(f"Stage '{name}' stopped")

    def watch(self) -> None:
        """Check the stages that are up until a stop is requested, then stop all of them."""
        failures = {name: 0 for name in self.stages}
        while not self.stop_requested.wait(self.watch_period):
            with self.lock:
                ready = [name for name, status in self.status.items() if status == "ready"]
            for name in ready:
                try:
                    problems = [p for unit in self.units[name] if (p := unit.problem())]
                except Exception as error:
                    problems = [repr(error)]
                failures[name] = failures[name] + 1 if problems else 0
                if failures[name] >= WATCH_FAILURES:
                    self.fail(name, "failed after startup: " + "; ".join(problems))
            self.publish()
        with self.lock:
            started = [
                name
                for name, status in self.status.items()
                if status in ("starting", "ready", "failed")
            ]
        if not self.context.ok():
            return
        self._stop(started)

    def publish(self) -> None:
        """Publish one diagnostic status per stage."""
        with self.lock:
            statuses = [
                DiagnosticStatus(
                    level=self._level(name, status),
                    name=f"{self.get_name()}: {name}",
                    message=status + (f", {self.reason[name]}" if self.reason[name] else ""),
                    values=[
                        KeyValue(key="state", value=status),
                        KeyValue(key="required", value=str(self.stages[name].required).lower()),
                        KeyValue(key="after", value=",".join(self.stages[name].after)),
                    ],
                )
                for name, status in self.status.items()
            ]
        header = Header(stamp=self.get_clock().now().to_msg())
        self.diagnostics.publish(DiagnosticArray(header=header, status=statuses))

    def _level(self, name: str, status: str) -> bytes:
        """Diagnostic level of a stage state: failures of required stages are errors."""
        if status in LEVELS:
            return LEVELS[status]
        return DiagnosticStatus.ERROR if self.stages[name].required else DiagnosticStatus.WARN


def main(args=None) -> None:
    """Run the supervisor until every stage has stopped, then exit with its code."""
    rclpy.init(args=args)
    try:
        node = Supervisor()
    except (graph.StageGraphError, OSError) as error:
        print(f"bringup_supervisor: {error}", file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(2)
    # Interrupted from outside (Ctrl-C): every node gets the signal, so there is nothing to stop
    node.context.on_shutdown(node.stop_requested.set)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        node.publish()
        node.start_ready()
        node.watch()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.exit_code = 0
    finally:
        # The process ends here, so the node is not destroyed one entity at a time
        executor.shutdown()
        rclpy.try_shutdown()
    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
