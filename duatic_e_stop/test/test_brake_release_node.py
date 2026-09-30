"""Tests for the brake release gesture.

The controller manager is replaced by an in-process fake that answers list_controllers and
switch_controller immediately, so update() can be driven tick by tick without spinning.
"""

import time

import pytest
import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.duration import Duration
from rclpy.task import Future

from duatic_e_stop.brake_release_node import BrakeReleaseNode

DEADMAN = 10
BRAKE = 2
BRAKE_CONTROLLER = "brake_release_controller"


class FakeControllerManager:
    """Holds the true controller states and records every switch request."""

    def __init__(self):
        self.states = {}
        self.switch_requests = []

    def set(self, name, state, claimed_interfaces=()):
        self.states[name] = {"state": state, "claimed_interfaces": list(claimed_interfaces)}

    def list_controllers(self, _request):
        response = ListControllers.Response()
        for name, info in self.states.items():
            response.controller.append(
                ControllerState(
                    name=name,
                    state=info["state"],
                    claimed_interfaces=info["claimed_interfaces"],
                )
            )
        return response

    def switch_controller(self, request):
        self.switch_requests.append(request)
        for name in request.deactivate_controllers:
            self.states[name]["state"] = "inactive"
        for name in request.activate_controllers:
            self.states[name]["state"] = "active"
        return SwitchController.Response(ok=True)


class FakeClient:
    def __init__(self, handler):
        self.handler = handler

    def wait_for_service(self, timeout_sec=None):
        return True

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = Future()
        future.set_result(self.handler(request))
        return future


@pytest.fixture(scope="module", autouse=True)
def ros():
    # Keeps the nodes under test away from any controller manager or gamepad already running.
    rclpy.init(args=["--ros-args", "-r", "__ns:=/brake_release_node_test"])
    yield
    rclpy.shutdown()


@pytest.fixture
def cm():
    manager = FakeControllerManager()
    manager.set(BRAKE_CONTROLLER, "inactive")
    return manager


@pytest.fixture
def node(cm):
    brake_node = BrakeReleaseNode()
    brake_node.list_controllers_client = FakeClient(cm.list_controllers)
    brake_node.switch_controller_client = FakeClient(cm.switch_controller)
    brake_node.poll_controller_states()
    yield brake_node
    brake_node.destroy_node()


def hold(node, *buttons):
    """Feed one joystick message with the given buttons pressed and run one update tick."""
    pressed = [0] * 16
    for button in buttons:
        pressed[button] = 1
    node.latest_buttons = pressed
    node.last_joy_time = node.get_clock().now()
    node.update()


def activations(cm):
    return [r for r in cm.switch_requests if BRAKE_CONTROLLER in r.activate_controllers]


def deactivations(cm):
    return [
        r
        for r in cm.switch_requests
        if BRAKE_CONTROLLER in r.deactivate_controllers
        and BRAKE_CONTROLLER not in r.activate_controllers
    ]


def test_holding_the_combo_activates_once(node, cm):
    hold(node, DEADMAN, BRAKE)
    hold(node, DEADMAN, BRAKE)
    hold(node, DEADMAN, BRAKE)

    assert len(activations(cm)) == 1
    assert cm.states[BRAKE_CONTROLLER]["state"] == "active"


def test_a_single_button_does_nothing(node, cm):
    hold(node, DEADMAN)
    hold(node, BRAKE)

    assert cm.switch_requests == []


def test_button_order_does_not_matter(node, cm):
    hold(node, BRAKE)
    hold(node, BRAKE, DEADMAN)

    assert len(activations(cm)) == 1


def test_releasing_after_a_status_poll_deactivates(node, cm):
    hold(node, DEADMAN, BRAKE)
    node.poll_controller_states()
    hold(node, DEADMAN)

    assert len(deactivations(cm)) == 1
    assert cm.states[BRAKE_CONTROLLER]["state"] == "inactive"


def test_releasing_before_the_next_status_poll_deactivates(node, cm):
    hold(node, DEADMAN, BRAKE)
    hold(node, DEADMAN)

    assert len(deactivations(cm)) == 1
    assert cm.states[BRAKE_CONTROLLER]["state"] == "inactive"


def test_a_silent_gamepad_counts_as_released(node, cm):
    hold(node, DEADMAN, BRAKE)
    node.poll_controller_states()

    node.last_joy_time = node.get_clock().now() - Duration(seconds=1.0)
    node.update()

    assert len(deactivations(cm)) == 1
    assert cm.states[BRAKE_CONTROLLER]["state"] == "inactive"


def test_pressing_again_while_active_retriggers_the_controller(node, cm):
    cm.set(BRAKE_CONTROLLER, "active")
    node.poll_controller_states()

    hold(node, DEADMAN, BRAKE)

    request = activations(cm)[0]
    assert list(request.deactivate_controllers) == [BRAKE_CONTROLLER]
    assert list(request.activate_controllers) == [BRAKE_CONTROLLER]


def test_an_active_freeze_blocks_the_release(node, cm):
    cm.set("freeze_controller_arm", "active")
    node.poll_controller_states()

    hold(node, DEADMAN, BRAKE)

    assert activations(cm) == []


def test_a_freeze_since_the_last_status_poll_blocks_the_release(node, cm):
    cm.set("freeze_controller_arm", "active")

    hold(node, DEADMAN, BRAKE)

    assert activations(cm) == []


def test_an_unloaded_brake_controller_is_not_switched(node, cm):
    del cm.states[BRAKE_CONTROLLER]
    node.poll_controller_states()

    hold(node, DEADMAN, BRAKE)

    assert cm.switch_requests == []


def test_a_motion_controller_holding_position_blocks_the_release(node, cm):
    cm.set("joint_trajectory_controller", "active", ["arm/joint1/position"])
    node.poll_controller_states()

    hold(node, DEADMAN, BRAKE)

    assert activations(cm) == []


def test_a_controller_without_position_interfaces_does_not_block(node, cm):
    cm.set("joint_state_broadcaster", "active", ["arm/joint1/effort"])
    node.poll_controller_states()

    hold(node, DEADMAN, BRAKE)

    assert len(activations(cm)) == 1


def test_update_does_not_block_without_a_controller_manager():
    brake_node = BrakeReleaseNode()
    brake_node.controller_states = {BRAKE_CONTROLLER: {"state": "inactive"}}

    start = time.monotonic()
    hold(brake_node, DEADMAN, BRAKE)
    elapsed = time.monotonic() - start
    brake_node.destroy_node()

    assert elapsed < 0.1
