#!/usr/bin/env python3

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

"""Exit 0 once every readiness check of a stage passes, or 1 naming the failed checks after a timeout."""

import sys
import time

import rclpy
import yaml
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from lifecycle_msgs.srv import GetState
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

POLL_PERIOD = 0.5
CALL_TIMEOUT = 2.0


def expected_controller_states(controllers_config):
    """Controller name to expected state: inactive if the config says so, otherwise active."""
    with open(controllers_config) as f:
        data = yaml.safe_load(f)
    cm_params = data["/**"]["controller_manager"]["ros__parameters"]
    return {
        name: "inactive" if params.get("state", "active").lower() == "inactive" else "active"
        for name, params in cm_params.items()
        if isinstance(params, dict) and "type" in params
    }


class ServiceCheck:
    """Polls a service and evaluates each response."""

    def __init__(self, node, service_type, service_name):
        self.node = node
        self.service_name = node.resolve_service_name(service_name)
        self.client = node.create_client(service_type, service_name)
        self.service_type = service_type
        self.future = None
        self.sent_at = None
        self.ok = False
        self.problem = f"'{self.service_name}' is not available"

    def poll(self):
        if self.ok:
            return
        if self.future is not None:
            if not self.future.done():
                if time.monotonic() - self.sent_at > CALL_TIMEOUT:
                    self.client.remove_pending_request(self.future)
                    self.future = None
                    self.problem = f"'{self.service_name}' did not answer"
                return
            response = self.future.result()
            self.future = None
            if response is not None:
                self.ok, self.problem = self.evaluate(response)
                if self.ok:
                    return
        if not self.client.service_is_ready():
            self.problem = f"'{self.service_name}' is not available"
            return
        self.sent_at = time.monotonic()
        self.future = self.client.call_async(self.service_type.Request())

    def evaluate(self, response):
        raise NotImplementedError


class ControllersCheck(ServiceCheck):
    def __init__(self, node, controller_manager, expected):
        super().__init__(node, ListControllers, f"{controller_manager}/list_controllers")
        self.expected = expected

    def evaluate(self, response):
        states = {controller.name: controller.state for controller in response.controller}
        wrong = []
        for name, state in self.expected.items():
            if name not in states:
                wrong.append(f"'{name}' is not loaded, expected {state}")
            elif states[name] != state:
                wrong.append(f"'{name}' is {states[name]}, expected {state}")
        return not wrong, "controllers: " + "; ".join(wrong)


class HardwareComponentsCheck(ServiceCheck):
    def __init__(self, node, controller_manager):
        super().__init__(
            node, ListHardwareComponents, f"{controller_manager}/list_hardware_components"
        )

    def evaluate(self, response):
        if not response.component:
            return False, "the controller manager reports no hardware components"
        wrong = [
            f"'{component.name}' is {component.state.label}"
            for component in response.component
            if component.state.label != "active"
        ]
        return not wrong, "hardware components not active: " + "; ".join(wrong)


class LifecycleCheck(ServiceCheck):
    def __init__(self, node, lifecycle_node):
        super().__init__(node, GetState, f"{lifecycle_node}/get_state")
        self.lifecycle_node = lifecycle_node

    def evaluate(self, response):
        label = response.current_state.label
        return label == "active", f"'{self.lifecycle_node}' is {label}, expected active"


class TopicCheck:
    """Passes once a first message arrives, whatever its type."""

    def __init__(self, node, topic):
        self.node = node
        self.topic = node.resolve_topic_name(topic)
        self.subscription = None
        self.ok = False
        self.problem = f"'{self.topic}' was never advertised"

    def poll(self):
        if self.subscription is not None:
            return
        types = dict(self.node.get_topic_names_and_types()).get(self.topic)
        if not types:
            return
        self.subscription = self.node.create_subscription(
            get_message(types[0]), self.topic, self.on_message, qos_profile_sensor_data
        )
        self.problem = f"'{self.topic}' is advertised but sent no message"

    def on_message(self, _msg):
        self.ok = True


class StageGate(Node):
    def __init__(self):
        super().__init__("stage_gate")
        self.stage = self.declare_parameter("stage", "").value
        # The scale comes from a launch argument as a string, so that "2" and "2.0" both work
        timeout_scale = float(self.declare_parameter("timeout_scale", "1.0").value)
        if not timeout_scale > 0.0:
            raise ValueError(f"timeout_scale must be positive, got {timeout_scale}")
        self.timeout = self.declare_parameter("timeout", 60.0).value * timeout_scale
        topics = self.declare_parameter("topics", [""]).value
        controllers_config = self.declare_parameter("controllers_config", "").value
        hardware_components_active = self.declare_parameter(
            "hardware_components_active", False
        ).value
        lifecycle_nodes = self.declare_parameter("lifecycle_nodes", [""]).value
        controller_manager = self.declare_parameter(
            "controller_manager", "controller_manager"
        ).value

        self.checks = [TopicCheck(self, topic) for topic in topics if topic]
        if controllers_config:
            self.checks.append(
                ControllersCheck(
                    self, controller_manager, expected_controller_states(controllers_config)
                )
            )
        if hardware_components_active:
            self.checks.append(HardwareComponentsCheck(self, controller_manager))
        self.checks += [LifecycleCheck(self, name) for name in lifecycle_nodes if name]

        self.exit_code = None
        # Wall time, a paused simulation must not stop the timeout
        self.started_at = time.monotonic()
        self.timer = self.create_timer(
            POLL_PERIOD, self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def poll(self):
        for check in self.checks:
            check.poll()
        if all(check.ok for check in self.checks):
            self.get_logger().info(f"All checks of stage '{self.stage}' passed")
            self.exit_code = 0
        elif time.monotonic() - self.started_at > self.timeout:
            for check in self.checks:
                if not check.ok:
                    self.get_logger().error(check.problem)
            self.get_logger().error(
                f"Stage '{self.stage}' did not come up within {self.timeout:.0f} s"
            )
            self.exit_code = 1
        if self.exit_code is not None:
            self.timer.cancel()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = StageGate()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    node.destroy_node()
    # None means the gate was interrupted before it decided
    sys.exit(1 if node.exit_code is None else node.exit_code)


if __name__ == "__main__":
    main()
