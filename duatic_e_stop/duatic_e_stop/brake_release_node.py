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

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from controller_manager_msgs.srv import SwitchController, ListControllers
from sensor_msgs.msg import Joy


class BrakeReleaseNode(Node):
    """Releases the joint brakes via a deliberate two-button hold gesture.

    While the (deadman + brake button) combo is held, the brake_release_controller is active and
    nudges the joints to unload the mechanical brake pins; releasing either button deactivates it
    again and frees the position command interfaces. It is order-independent (both buttons just have
    to be held together).

    Note: the brake itself stays released after deactivation - that is the drive FSM (ControlOp),
    not this controller. Deactivating only frees the joints for the motion controllers.
    """

    # If no joystick message arrives for this long, treat the combo as released so the brake
    # release controller can never stay stuck active after a gamepad disconnect.
    JOY_TIMEOUT = Duration(seconds=0.5)

    def __init__(self):
        super().__init__("brake_release_node")

        self.declare_parameter("deadman_button", 10)  # R1 / Right Shoulder
        self.deadman_button = int(self.get_parameter("deadman_button").value)
        self.declare_parameter("brake_release_button", 2)  # Face Left / Square
        self.brake_release_button = int(self.get_parameter("brake_release_button").value)
        self.declare_parameter("brake_release_controller", "brake_release_controller")
        self.brake_release_controller = str(self.get_parameter("brake_release_controller").value)

        self.latest_buttons = []
        self.last_joy_time = self.get_clock().now()
        self.brake_combo_was_pressed = False
        # name -> {"state": str, "claimed_interfaces": [str]} from the periodic poll
        self.controller_states = {}

        self.list_controllers_client = self.create_client(
            ListControllers, "controller_manager/list_controllers"
        )
        self.switch_controller_client = self.create_client(
            SwitchController, "controller_manager/switch_controller"
        )

        self.create_subscription(Joy, "joy", self.joy_callback, 10)

        # Fast loop for button edge detection, slow loop for controller status.
        self.create_timer(0.05, self.update)
        self.create_timer(2.0, self.poll_controller_states)

    def joy_callback(self, msg: Joy):
        self.latest_buttons = list(msg.buttons)
        self.last_joy_time = self.get_clock().now()

    def update(self):
        """Edge-detect the hold combo and (de)activate the brake release controller accordingly."""
        btns = self.latest_buttons
        joy_alive = (self.get_clock().now() - self.last_joy_time) < self.JOY_TIMEOUT

        def pressed(idx):
            return idx is not None and 0 <= idx < len(btns) and bool(btns[idx])

        combo_pressed = (
            joy_alive and pressed(self.deadman_button) and pressed(self.brake_release_button)
        )

        if combo_pressed and not self.brake_combo_was_pressed:
            self._try_release_brake()  # rising edge: start releasing (guarded)
        elif not combo_pressed and self.brake_combo_was_pressed:
            self.deactivate_brake_release()  # falling edge: stop, free the joints

        self.brake_combo_was_pressed = combo_pressed

    def poll_controller_states(self):
        """Periodically cache all controller states so update() stays non-blocking."""
        if not self.list_controllers_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                "Controller Manager not available for status update.", throttle_duration_sec=10.0
            )
            return
        future = self.list_controllers_client.call_async(ListControllers.Request())
        future.add_done_callback(self._handle_status_response)

    def _handle_status_response(self, future):
        try:
            result = future.result()
            states = {}
            if result:
                for controller in result.controller:
                    states[controller.name] = {
                        "state": controller.state,
                        "claimed_interfaces": list(getattr(controller, "claimed_interfaces", [])),
                    }
            self.controller_states = states
        except Exception as e:
            self.get_logger().warn(f"Failed to update controller states: {e}")

    def _is_freeze_active(self):
        """True if any freeze controller is currently active (from the poll cache)."""
        return any(
            name.startswith("freeze_controller") and info.get("state") == "active"
            for name, info in self.controller_states.items()
        )

    def _is_blocking_controller_active(self):
        """True if an active controller (other than the brake controller) already claims a
        position command interface, i.e. the arm is being commanded and must not be nudged."""
        for name, info in self.controller_states.items():
            if name == self.brake_release_controller:
                continue
            if info.get("state") != "active":
                continue
            if any(iface.endswith("/position") for iface in info.get("claimed_interfaces", [])):
                return True
        return False

    def _try_release_brake(self):
        """Guarded entry point for the brake release combo."""
        # Guard 1: never release the brake while frozen (E-Stop must be cleared first).
        if self._is_freeze_active():
            self.get_logger().warn("Brake release ignored: freeze controller is active.")
            return

        # Guard 2: the brake controller must actually be loaded.
        if self.brake_release_controller not in self.controller_states:
            self.get_logger().warn(
                f"Brake release ignored: '{self.brake_release_controller}' is not loaded."
            )
            return

        # Guard 3: do nothing while a motion controller already holds the joints (normal operation).
        if self._is_blocking_controller_active():
            self.get_logger().warn("Brake release ignored: a motion controller is active.")
            return

        self.activate_brake_release()

    def activate_brake_release(self):
        """Activate the brake release controller. The brake-pin nudge happens on its activation;
        it stays active (holding the joints) until the combo is released."""
        if not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Controller Manager service not available for brake release.")
            return

        name = self.brake_release_controller
        already_active = self.controller_states.get(name, {}).get("state") == "active"

        request = SwitchController.Request()
        request.activate_controllers = [name]
        # SwitchController processes deactivations before activations, so this re-triggers atomically.
        request.deactivate_controllers = [name] if already_active else []
        request.strictness = SwitchController.Request.BEST_EFFORT

        self.get_logger().info("Brake release combo held - activating brake release controller.")
        future = self.switch_controller_client.call_async(request)
        future.add_done_callback(lambda f: self._log_switch_response(f, "activation"))

    def deactivate_brake_release(self):
        name = self.brake_release_controller
        # Nothing to do if it is not (or no longer) active.
        if self.controller_states.get(name, {}).get("state") != "active":
            return
        if not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                "Controller Manager service not available to deactivate brake release."
            )
            return

        request = SwitchController.Request()
        request.deactivate_controllers = [name]
        request.strictness = SwitchController.Request.BEST_EFFORT

        self.get_logger().info("Brake release combo released - deactivating brake release controller.")
        future = self.switch_controller_client.call_async(request)
        future.add_done_callback(lambda f: self._log_switch_response(f, "deactivation"))

    def _log_switch_response(self, future, action):
        try:
            response = future.result()
            if not response.ok:
                self.get_logger().warn(f"Brake release {action} rejected by controller-manager.")
        except Exception as e:
            self.get_logger().error(f"Brake release {action} service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BrakeReleaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
