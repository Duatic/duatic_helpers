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
from controller_manager_msgs.srv import SwitchController, ListControllers
from sensor_msgs.msg import Joy, JoyFeedback


class BrakeReleaseNode(Node):
    """Starts the brake release with a two-button gamepad gesture and rumbles once it is done.

    Pressing the (deadman + brake button) combo activates the brake_release_controller. Letting go
    does not stop it: the controller deactivates itself once every drive reports its brake as
    released, and the node then rumbles the gamepad. Pressing the combo again while the controller is
    still active restarts it. The buttons can be pressed in any order.
    """

    def __init__(self):
        super().__init__("brake_release_node")

        self.declare_parameter("deadman_button", 10)  # R1 / Right Shoulder
        self.deadman_button = int(self.get_parameter("deadman_button").value)
        self.declare_parameter("brake_release_button", 2)  # Face Left / Square
        self.brake_release_button = int(self.get_parameter("brake_release_button").value)
        self.declare_parameter("brake_release_controller", "brake_release_controller")
        self.brake_release_controller = str(self.get_parameter("brake_release_controller").value)

        self.latest_buttons = []
        self.brake_combo_was_pressed = False
        # Set while the controller runs after an activation from this node, until it stops.
        self.release_running = False
        # name -> {"state": str, "claimed_interfaces": [str]} from the last list_controllers call
        self.controller_states = {}

        self.list_controllers_client = self.create_client(
            ListControllers, "controller_manager/list_controllers"
        )
        self.switch_controller_client = self.create_client(
            SwitchController, "controller_manager/switch_controller"
        )

        self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.feedback_publisher = self.create_publisher(JoyFeedback, "joy/set_feedback", 10)

        # Fast loop for button edge detection, slower loop to notice the controller finishing.
        self.create_timer(0.05, self.update)
        self.create_timer(0.5, self.watch_release)

    def joy_callback(self, msg: Joy):
        """Keep the latest button states for the edge detection in update()."""
        self.latest_buttons = list(msg.buttons)

    def update(self):
        """Start the brake release on the rising edge of the button combo."""
        btns = self.latest_buttons

        def pressed(idx):
            return idx is not None and 0 <= idx < len(btns) and bool(btns[idx])

        combo_pressed = pressed(self.deadman_button) and pressed(self.brake_release_button)

        # Stored before acting, so a states response that returns at once sees the new value.
        was_pressed = self.brake_combo_was_pressed
        self.brake_combo_was_pressed = combo_pressed

        if combo_pressed and not was_pressed:
            self._try_release_brake()

    def watch_release(self):
        """Poll the controller states while a release runs, to notice when it has finished."""
        if not self.release_running:
            return
        if not self.list_controllers_client.service_is_ready():
            self.get_logger().warn(
                "Controller Manager not available for status update.", throttle_duration_sec=10.0
            )
            return
        future = self.list_controllers_client.call_async(ListControllers.Request())
        future.add_done_callback(self._handle_status_response)

    def _handle_status_response(self, future):
        try:
            self._store_controller_states(future.result())
        except Exception as e:
            self.get_logger().warn(f"Failed to update controller states: {e}")

    def _store_controller_states(self, result):
        """Cache the controller states and report a release the controller has finished."""
        states = {}
        if result:
            for controller in result.controller:
                states[controller.name] = {
                    "state": controller.state,
                    "claimed_interfaces": list(getattr(controller, "claimed_interfaces", [])),
                }
        self.controller_states = states

        # The controller deactivates itself only once every brake is released.
        brake_state = states.get(self.brake_release_controller, {}).get("state")
        if self.release_running and brake_state == "inactive":
            self.release_running = False
            self.get_logger().info("All brakes released.")
            self.feedback_publisher.publish(
                JoyFeedback(type=JoyFeedback.TYPE_RUMBLE, id=0, intensity=1.0)
            )
        elif self.release_running and self._is_freeze_active():
            self.abort_brake_release()

    def _is_freeze_active(self):
        """True if any freeze controller is currently active (from the poll cache)."""
        return any(
            name.startswith("freeze_controller") and info.get("state") == "active"
            for name, info in self.controller_states.items()
        )

    def _is_blocking_controller_active(self):
        """True if an active controller (other than the brake controller) already claims a
        position command interface, i.e. the arm is being commanded and must not be moved."""
        for name, info in self.controller_states.items():
            if name == self.brake_release_controller:
                continue
            if info.get("state") != "active":
                continue
            if any(iface.endswith("/position") for iface in info.get("claimed_interfaces", [])):
                return True
        return False

    def _try_release_brake(self):
        """Fetch fresh controller states, then run the guards on them."""
        if not self.list_controllers_client.service_is_ready():
            self.get_logger().error("Controller Manager service not available for brake release.")
            return
        future = self.list_controllers_client.call_async(ListControllers.Request())
        future.add_done_callback(self._release_with_fresh_states)

    def _release_with_fresh_states(self, future):
        try:
            self._store_controller_states(future.result())
        except Exception as e:
            self.get_logger().error(f"Brake release ignored: controller states unavailable: {e}")
            return

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
        """Activate the brake release controller, restarting it if it is still active."""
        if not self.switch_controller_client.service_is_ready():
            self.get_logger().error("Controller Manager service not available for brake release.")
            return

        name = self.brake_release_controller
        already_active = self.controller_states.get(name, {}).get("state") == "active"

        request = SwitchController.Request()
        request.activate_controllers = [name]
        # SwitchController processes deactivations before activations, so this restarts atomically.
        request.deactivate_controllers = [name] if already_active else []
        # Strict, so ok means the controller really is active and its stop can be watched for.
        request.strictness = SwitchController.Request.STRICT

        self.get_logger().info("Brake release combo pressed - activating brake release controller.")
        future = self.switch_controller_client.call_async(request)
        future.add_done_callback(self._handle_activation_response)

    def abort_brake_release(self):
        """Deactivate the running brake release controller, so a cleared freeze does not resume it."""
        self.release_running = False
        if not self.switch_controller_client.service_is_ready():
            self.get_logger().error(
                "Controller Manager service not available to abort brake release."
            )
            return

        request = SwitchController.Request()
        request.deactivate_controllers = [self.brake_release_controller]
        request.strictness = SwitchController.Request.BEST_EFFORT

        self.get_logger().warn("Freeze controller active - aborting brake release.")
        future = self.switch_controller_client.call_async(request)
        future.add_done_callback(self._handle_abort_response)

    def _handle_abort_response(self, future):
        """Log an abort the controller manager did not carry out."""
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Brake release abort service call failed: {e}")
            return
        if not response.ok:
            self.get_logger().error("Brake release abort rejected by controller-manager.")

    def _handle_activation_response(self, future):
        """Start watching for the end of the release once the controller manager confirms it."""
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Brake release activation service call failed: {e}")
            return
        if not response.ok:
            self.get_logger().warn("Brake release activation rejected by controller-manager.")
            return
        self.release_running = True


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
