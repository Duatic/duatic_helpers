"""Tests for the brake release gesture and its completion feedback.

The controller manager is replaced by an in-process fake that answers list_controllers and
switch_controller immediately, so update() can be driven tick by tick without spinning.
"""

import time

import pytest
import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from sensor_msgs.msg import JoyFeedback
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
        self.accept_switches = True

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
        if not self.accept_switches:
            return SwitchController.Response(ok=False)
        for name in request.deactivate_controllers:
            self.states[name]["state"] = "inactive"
        for name in request.activate_controllers:
            self.states[name]["state"] = "active"
        return SwitchController.Response(ok=True)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


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
    brake_node.feedback_publisher = FakePublisher()
    yield brake_node
    brake_node.destroy_node()


def hold(node, *buttons):
    """Feed one joystick message with the given buttons pressed and run one update tick."""
    pressed = [0] * 16
    for button in buttons:
        pressed[button] = 1
    node.latest_buttons = pressed
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


def rumbles(node):
    return node.feedback_publisher.messages


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


def test_letting_go_keeps_the_controller_running(node, cm):
    hold(node, DEADMAN, BRAKE)
    hold(node)
    node.watch_release()

    assert len(cm.switch_requests) == 1
    assert cm.states[BRAKE_CONTROLLER]["state"] == "active"
    assert rumbles(node) == []


def test_the_gamepad_rumbles_once_the_controller_deactivates_itself(node, cm):
    hold(node, DEADMAN, BRAKE)
    cm.set(BRAKE_CONTROLLER, "inactive")
    node.watch_release()
    node.watch_release()

    assert len(rumbles(node)) == 1
    assert rumbles(node)[0].type == JoyFeedback.TYPE_RUMBLE


def test_a_rejected_activation_does_not_rumble(node, cm):
    cm.accept_switches = False
    hold(node, DEADMAN, BRAKE)
    node.watch_release()

    assert rumbles(node) == []


def test_pressing_again_while_active_retriggers_the_controller(node, cm):
    cm.set(BRAKE_CONTROLLER, "active")

    hold(node, DEADMAN, BRAKE)

    request = activations(cm)[0]
    assert list(request.deactivate_controllers) == [BRAKE_CONTROLLER]
    assert list(request.activate_controllers) == [BRAKE_CONTROLLER]


def test_restarting_a_running_release_does_not_rumble(node, cm):
    hold(node, DEADMAN, BRAKE)
    hold(node)
    hold(node, DEADMAN, BRAKE)
    node.watch_release()

    assert len(activations(cm)) == 2
    assert rumbles(node) == []


def test_a_freeze_aborts_a_running_release_without_rumble(node, cm):
    hold(node, DEADMAN, BRAKE)
    cm.set("freeze_controller_arm", "active")
    node.watch_release()
    node.watch_release()

    assert len(deactivations(cm)) == 1
    assert cm.states[BRAKE_CONTROLLER]["state"] == "inactive"
    assert rumbles(node) == []


def test_an_active_freeze_blocks_the_release(node, cm):
    cm.set("freeze_controller_arm", "active")

    hold(node, DEADMAN, BRAKE)

    assert activations(cm) == []


def test_an_unloaded_brake_controller_is_not_switched(node, cm):
    del cm.states[BRAKE_CONTROLLER]

    hold(node, DEADMAN, BRAKE)

    assert cm.switch_requests == []


def test_a_motion_controller_holding_position_blocks_the_release(node, cm):
    cm.set("joint_trajectory_controller", "active", ["arm/joint1/position"])

    hold(node, DEADMAN, BRAKE)

    assert activations(cm) == []


def test_a_controller_without_position_interfaces_does_not_block(node, cm):
    cm.set("joint_state_broadcaster", "active", ["arm/joint1/effort"])

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
