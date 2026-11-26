# Copyright 2026 Duatic AG
# SPDX-License-Identifier: BSD-3-Clause

"""The parts a stage starts, checks and stops: lifecycle nodes, managers it delegates to, topics.

Every call blocks the calling thread, so the supervisor runs each stage in its own thread while
an executor in another thread handles the responses.
"""

import threading
import time

import yaml
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    SetHardwareComponentState,
    SwitchController,
)
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from std_srvs.srv import Trigger

from duatic_bringup_supervisor.graph import Ros2Control, Stage, TopicCheck

CALL_TIMEOUT = 5.0
POLL_PERIOD = 0.2


class StageError(RuntimeError):
    """A part of a stage could not be started or stopped."""


def remaining(deadline: float) -> float:
    """Seconds left until a ``time.monotonic()`` deadline, never negative."""
    return max(0.0, deadline - time.monotonic())


def call(client, request, timeout: float, cancel: threading.Event | None = None):
    """Call a service and wait for the response, or until ``cancel`` is set.

    rclpy's ``Client.call`` cannot be interrupted, and a stage that is being stopped must not
    wait out a Nav2 startup of several minutes.

    :returns: The response, or ``None`` when the service is not available, does not answer in
        time, or the wait is cancelled.
    """
    if not client.service_is_ready():
        return None
    future = client.call_async(request)
    answered = threading.Event()
    future.add_done_callback(lambda _: answered.set())
    deadline = time.monotonic() + timeout
    while not answered.wait(min(POLL_PERIOD, remaining(deadline))):
        if time.monotonic() >= deadline or (cancel is not None and cancel.is_set()):
            client.remove_pending_request(future)
            return None
    return future.result()


def wait_for_service(client, deadline: float, cancel: threading.Event) -> None:
    """Block until a service is available.

    :raises StageError: When the deadline passes or the wait is cancelled first.
    """
    while not client.wait_for_service(timeout_sec=min(POLL_PERIOD, remaining(deadline))):
        if cancel.is_set():
            raise StageError("stopped while starting")
        if time.monotonic() >= deadline:
            raise StageError(f"'{client.srv_name}' is not available")


class LifecycleNodeUnit:
    """A lifecycle node that the stage configures and activates itself."""

    def __init__(self, node: Node, name: str, cancel: threading.Event):
        """Create the clients for the lifecycle services of node ``name``."""
        self.name = node.resolve_service_name(name)
        self.cancel = cancel
        self.get_state = node.create_client(GetState, f"{name}/get_state")
        self.change_state = node.create_client(ChangeState, f"{name}/change_state")

    def _state(self, timeout: float = CALL_TIMEOUT, cancel=None) -> State | None:
        """The node's current lifecycle state, or ``None`` when it does not answer."""
        response = call(self.get_state, GetState.Request(), timeout, cancel)
        return None if response is None else response.current_state

    def _transition(self, transition: int, label: str, timeout: float, cancel=None) -> None:
        """Trigger a lifecycle transition and raise if the node rejects or ignores it."""
        request = ChangeState.Request(transition=Transition(id=transition))
        response = call(self.change_state, request, timeout, cancel)
        if response is None or not response.success:
            raise StageError(f"'{self.name}' did not {label}")

    def start(self, deadline: float) -> None:
        """Configure and activate the node, skipping transitions it has already made."""
        wait_for_service(self.get_state, deadline, self.cancel)
        wait_for_service(self.change_state, deadline, self.cancel)
        state = None
        while state is None:
            if self.cancel.is_set() or time.monotonic() > deadline:
                raise StageError(f"'{self.name}' does not report its state")
            state = self._state(min(CALL_TIMEOUT, remaining(deadline)), self.cancel)
        configure, activate = Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE
        if state.id == State.PRIMARY_STATE_UNCONFIGURED:
            self._transition(configure, "configure", remaining(deadline), self.cancel)
            state = State(id=State.PRIMARY_STATE_INACTIVE)
        if state.id == State.PRIMARY_STATE_INACTIVE:
            self._transition(activate, "activate", remaining(deadline), self.cancel)
        elif state.id != State.PRIMARY_STATE_ACTIVE:
            raise StageError(f"'{self.name}' is {state.label}, cannot activate it")

    def problem(self) -> str | None:
        """What keeps the node from counting as up, or ``None`` when it is active."""
        state = self._state()
        if state is None:
            return f"'{self.name}' does not report its state"
        if state.id != State.PRIMARY_STATE_ACTIVE:
            return f"'{self.name}' is {state.label}, expected active"
        return None

    def stop(self, timeout: float) -> None:
        """Deactivate and clean up the node, so that a later start begins from unconfigured."""
        deadline = time.monotonic() + timeout
        state = self._state(min(CALL_TIMEOUT, timeout))
        if state is None:
            return
        if state.id == State.PRIMARY_STATE_ACTIVE:
            self._transition(Transition.TRANSITION_DEACTIVATE, "deactivate", remaining(deadline))
            state = State(id=State.PRIMARY_STATE_INACTIVE)
        if state.id == State.PRIMARY_STATE_INACTIVE:
            self._transition(Transition.TRANSITION_CLEANUP, "clean up", remaining(deadline))


class Nav2ManagerUnit:
    """A Nav2 lifecycle manager the stage hands its servers to, as one unit."""

    def __init__(self, node: Node, name: str, cancel: threading.Event):
        """Create the clients for the manager's ``manage_nodes`` and ``is_active`` services."""
        self.name = node.resolve_service_name(name)
        self.cancel = cancel
        self.manage = node.create_client(ManageLifecycleNodes, f"{name}/manage_nodes")
        self.is_active = node.create_client(Trigger, f"{name}/is_active")

    def _active(self, timeout: float = CALL_TIMEOUT) -> bool | None:
        """Whether the manager reports its nodes active, ``None`` when it does not answer."""
        response = call(self.is_active, Trigger.Request(), timeout)
        return None if response is None else response.success

    def _command(self, command: int, label: str, timeout: float, cancel=None) -> None:
        """Send a ``manage_nodes`` command and raise if the manager reports a failure."""
        request = ManageLifecycleNodes.Request(command=command)
        response = call(self.manage, request, timeout, cancel)
        if response is None or not response.success:
            raise StageError(f"'{self.name}' did not {label} its nodes")

    def start(self, deadline: float) -> None:
        """Ask the manager to start its nodes, unless they are already active."""
        wait_for_service(self.manage, deadline, self.cancel)
        wait_for_service(self.is_active, deadline, self.cancel)
        if not self._active(min(CALL_TIMEOUT, remaining(deadline))):
            startup = ManageLifecycleNodes.Request.STARTUP
            self._command(startup, "start", remaining(deadline), self.cancel)

    def problem(self) -> str | None:
        """What keeps the managed nodes from counting as up, or ``None`` when they are active."""
        active = self._active()
        if active is None:
            return f"'{self.name}' does not answer"
        return None if active else f"'{self.name}' reports its nodes inactive"

    def stop(self, timeout: float) -> None:
        """Ask the manager to deactivate and clean up its nodes."""
        if self._active(min(CALL_TIMEOUT, timeout)):
            self._command(ManageLifecycleNodes.Request.RESET, "reset", timeout)


def expected_controller_states(controllers_config: str) -> dict[str, str]:
    """Controller name to its expected state: ``inactive`` if the config says so, else ``active``."""
    if not controllers_config:
        return {}
    with open(controllers_config) as f:
        data = yaml.safe_load(f)
    parameters = data["/**"]["controller_manager"]["ros__parameters"]
    return {
        name: "inactive" if str(params.get("state", "active")).lower() == "inactive" else "active"
        for name, params in parameters.items()
        if isinstance(params, dict) and "type" in params
    }


class Ros2ControlUnit:
    """A controller manager the stage watches as one unit, and stops in order when it owns it."""

    def __init__(self, node: Node, config: Ros2Control):
        """Create the controller manager clients and read the expected controller states."""
        manager = config.controller_manager
        self.name = node.resolve_service_name(manager)
        self.manage = config.manage
        self.expected = expected_controller_states(config.controllers_config)
        self.list_controllers = node.create_client(ListControllers, f"{manager}/list_controllers")
        self.list_hardware = node.create_client(
            ListHardwareComponents, f"{manager}/list_hardware_components"
        )
        self.switch = node.create_client(SwitchController, f"{manager}/switch_controller")
        self.set_hardware = node.create_client(
            SetHardwareComponentState, f"{manager}/set_hardware_component_state"
        )

    def _controllers(self, timeout: float = CALL_TIMEOUT) -> dict[str, str] | None:
        """Loaded controllers and their states, ``None`` when the manager does not answer."""
        response = call(self.list_controllers, ListControllers.Request(), timeout)
        return None if response is None else {c.name: c.state for c in response.controller}

    def _hardware(self, timeout: float = CALL_TIMEOUT) -> dict[str, str] | None:
        """Hardware components and their state labels, ``None`` when the manager does not answer."""
        response = call(self.list_hardware, ListHardwareComponents.Request(), timeout)
        return None if response is None else {c.name: c.state.label for c in response.component}

    def _set_hardware(self, name: str, state_id: int, label: str, timeout: float) -> None:
        """Move one hardware component to a lifecycle state."""
        request = SetHardwareComponentState.Request(
            name=name, target_state=State(id=state_id, label=label)
        )
        response = call(self.set_hardware, request, timeout)
        if response is None or not response.ok:
            raise StageError(f"'{self.name}' did not set hardware '{name}' {label}")

    def _deactivate(self, controllers: list[str], timeout: float) -> None:
        """Deactivate controllers in one strict switch."""
        request = SwitchController.Request(
            deactivate_controllers=controllers,
            strictness=SwitchController.Request.STRICT,
            timeout=Duration(sec=int(CALL_TIMEOUT)),
        )
        response = call(self.switch, request, timeout)
        if response is None or not response.ok:
            raise StageError(f"'{self.name}' did not deactivate controllers {controllers}")

    def start(self, deadline: float) -> None:
        """Nothing to start: the controller manager activates the hardware and the spawners the
        controllers. A second party switching the same controllers makes both switches fail."""

    def problem(self) -> str | None:
        """What keeps controllers or hardware from their expected states, ``None`` if nothing."""
        hardware = self._hardware()
        if hardware is None:
            return f"'{self.name}' does not answer"
        if not hardware:
            return f"'{self.name}' reports no hardware components"
        wrong = [
            f"hardware '{name}' is {label}" for name, label in hardware.items() if label != "active"
        ]
        if self.expected:
            controllers = self._controllers() or {}
            wrong += [
                f"controller '{name}' is {controllers.get(name, 'not loaded')}, expected {state}"
                for name, state in self.expected.items()
                if controllers.get(name) != state
            ]
        return "; ".join(wrong) or None

    def stop(self, timeout: float) -> None:
        """Deactivate the controllers, then the hardware, so that nothing commands the drives."""
        if not self.manage:
            return
        deadline = time.monotonic() + timeout
        controllers = self._controllers(min(CALL_TIMEOUT, timeout)) or {}
        # Whatever is active, so that the order holds also when no controllers file is checked
        active = [name for name, state in controllers.items() if state == "active"]
        if active:
            self._deactivate(active, remaining(deadline))
        for name, label in (self._hardware(remaining(deadline)) or {}).items():
            if label == "active":
                self._set_hardware(
                    name, State.PRIMARY_STATE_INACTIVE, "inactive", remaining(deadline)
                )


class TopicUnit:
    """A topic the stage needs data on, for a part that is not a lifecycle node.

    It subscribes without deserializing, so that checking images or point clouds costs little.
    """

    def __init__(self, node: Node, check: TopicCheck):
        """Prepare the check; the subscription is made once the topic's type is known."""
        self.node = node
        self.name = node.resolve_topic_name(check.name)
        self.max_age = check.max_age
        self.subscription = None
        self.last_message: float | None = None

    def _subscribe(self) -> None:
        """Subscribe once the topic appears in the graph, with whatever type it has."""
        if not self.node.count_publishers(self.name):
            return
        types = dict(self.node.get_topic_names_and_types()).get(self.name)
        if types:
            self.subscription = self.node.create_subscription(
                get_message(types[0]),
                self.name,
                self._on_message,
                qos_profile_sensor_data,
                raw=True,
            )

    def _on_message(self, _message: bytes) -> None:
        """Record when the last message arrived."""
        self.last_message = time.monotonic()

    def start(self, deadline: float) -> None:
        """Nothing to start: the publisher is a plain node that runs from launch."""

    def _unsubscribe(self) -> None:
        """Drop the subscription, so that unused messages no longer wake the executor."""
        if self.subscription is not None:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None

    def problem(self) -> str | None:
        """Whether data arrives: a first message, and with ``max_age`` a recent one."""
        if self.last_message is None:
            if self.subscription is None:
                self._subscribe()
                if self.subscription is None:
                    return f"'{self.name}' is not advertised"
            return f"'{self.name}' sent no message"
        if self.max_age is None:
            # Only the first message counts, the subscription is no longer needed
            self._unsubscribe()
            return None
        age = time.monotonic() - self.last_message
        if age > self.max_age:
            return f"'{self.name}' sent nothing for {age:.1f} s"
        return None

    def stop(self, timeout: float) -> None:
        """Stop listening; launch owns the publishing process."""
        self._unsubscribe()
        self.last_message = None


def units_of(node: Node, stage: Stage, cancel: threading.Event) -> list:
    """The parts of a stage in the order they start: lifecycle nodes, delegates, then topics.

    :param cancel: Set when the stage is stopped, to end a start that is still waiting.
    """
    units: list = [LifecycleNodeUnit(node, name, cancel) for name in stage.lifecycle_nodes]
    units += [Nav2ManagerUnit(node, name, cancel) for name in stage.nav2_managers]
    if stage.ros2_control is not None:
        units.append(Ros2ControlUnit(node, stage.ros2_control))
    units += [TopicUnit(node, check) for check in stage.topics]
    return units
