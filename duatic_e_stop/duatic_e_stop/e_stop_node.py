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

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from controller_manager_msgs.srv import SwitchController, ListControllers
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

try:
    import serial

    _SERIAL_AVAILABLE = True
except ImportError:
    _SERIAL_AVAILABLE = False

try:
    from clearpath_platform_msgs.msg import StopStatus
except ImportError:
    StopStatus = None

# Platform topics consumed when extern_e_stop is enabled.
#
# Activation  — serial line (PC817/UART): immediate on press-edge detection.
#               Topics also activate if they confirm pressed (initial state / missed serial edge).
# Clearing    — platform topics ONLY, once fully seeded and both values show clear:
#   emergency_stop == False AND needs_reset == False  -> e-stop CLEAR
#
# Rationale: platform topics are too slow for reliable press detection at runtime,
# but they are the authoritative source for confirming that the e-stop has been released.


class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__("dynaarm_emergency_stop_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("emergency_stop_button", 9)
        self.emergency_stop_button = int(self.get_parameter("emergency_stop_button").value)

        # Set True to also monitor external platform e-stop (topics + serial cross-check).
        # If any source is active, freeze stays engaged regardless of gamepad state.
        self.declare_parameter("extern_e_stop", False)
        self._extern_e_stop_enabled: bool = self.get_parameter("extern_e_stop").value

        # Serial port parameters (only used when extern_e_stop is True).
        # /dev/duatic_estop is a stable udev symlink (see clearpath_ridgeback/udev/)
        # pinned to the Prolific PL2303 adapter's vendor/product ID, so it doesn't
        # depend on USB enumeration order like /dev/ttyUSB0 does.
        self.declare_parameter("serial_port", "/dev/duatic_estop")
        self.declare_parameter("serial_baud", 9600)

        self.declare_parameter("extern_emergency_stop_topic", "/r100_0208/platform/emergency_stop")
        self.declare_parameter("extern_mcu_stop_status_topic", "/r100_0208/platform/mcu/status/stop")

        # ── Gamepad / internal e-stop state ───────────────────────────────────
        self.gamepad_connected = False
        self.e_stop_button_release = True
        self.e_stop_pressed_state = False
        self.was_freeze_controller_active_before = False
        self.emergency_stop_pressed_time = None
        self.last_joy_received_time = self.get_clock().now()
        self.last_toggle_time = self.get_clock().now()
        self.freeze_controller_states = {}
        # Tracks the last freeze state we sent to the controller manager.
        # None = not yet requested. Prevents duplicate service calls while a
        # request is in-flight; reset to None on failure to allow a retry.
        self._freeze_target: bool | None = None

        # ── External e-stop state (platform topics) ───────────────────────────
        # Protected by self._lock because the serial reader thread also reads these.
        self._lock = threading.Lock()
        self._extern_emergency_stop: bool | None = None
        self._extern_needs_reset: bool | None = None
        self._extern_estop_active: bool = False
        self._extern_seeded: bool = False

        # ── Serial state (fast activation; clearing is topics-only) ─────────────
        self._serial_transition_count: int = 0
        self._last_serial_transition_time: float | None = None
        # Naive toggle guess from serial transitions. Seeded on first transition to the
        # opposite of the current platform state so it starts in phase. guess=PRESSED
        # immediately activates _extern_estop_active; guess=CLEAR is logged only.
        self._serial_guess_pressed: bool | None = None
        self._running: bool = False
        self._ser = None
        self._serial_port: str = ""
        self._serial_baud: int = 9600
        self._reader_thread: threading.Thread | None = None

        # ── Service clients ───────────────────────────────────────────────────
        self.list_controllers_client = self.create_client(
            ListControllers, "controller_manager/list_controllers"
        )
        self.switch_controller_client = self.create_client(
            SwitchController, "controller_manager/switch_controller"
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.joy_subscriber = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        if self._extern_e_stop_enabled:
            self.get_logger().info(
                "External e-stop monitoring ENABLED (platform topics + serial cross-check)."
            )

            # Platform topics use Best Effort — match that to avoid silent QoS mismatch.
            sub_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                Bool, self.get_parameter("extern_emergency_stop_topic").value, self._extern_estop_cb, sub_qos
            )
            if StopStatus is not None:
                self.create_subscription(
                    StopStatus,
                    self.get_parameter("extern_mcu_stop_status_topic").value,
                    self._extern_mcu_status_cb,
                    sub_qos,
                )
            else:
                self.get_logger().error(
                    f"Could not import StopStatus for {self.get_parameter('extern_mcu_stop_status_topic').value}. "
                    "needs_reset will not be tracked — update the import in e_stop_node.py."
                )

            # Serial port (fast-path activation; opened in a background thread so a
            # missing device at startup does not block the node or prevent retries).
            if not _SERIAL_AVAILABLE:
                self.get_logger().warn("pyserial not installed — serial fast-path disabled.")
            else:
                self._serial_port = self.get_parameter("serial_port").value
                self._serial_baud = self.get_parameter("serial_baud").value
                self._running = True
                self._reader_thread = threading.Thread(target=self._read_serial, daemon=True)
                self._reader_thread.start()
        else:
            self.get_logger().info("External e-stop monitoring DISABLED.")

        # ── Timers ────────────────────────────────────────────────────────────
        self.control_loop_timer = self.create_timer(0.001, self.control_loop)
        self.status_update_timer = self.create_timer(2.0, self.update_freeze_controllers_status)

    # ── External e-stop callbacks (ROS executor thread) ──────────────────────

    def _extern_estop_cb(self, msg: Bool):
        with self._lock:
            self._extern_emergency_stop = msg.data
            self._update_extern_state_locked()

    def _extern_mcu_status_cb(self, msg):
        with self._lock:
            self._extern_needs_reset = msg.needs_reset
            self._update_extern_state_locked()

    def _update_extern_state_locked(self):
        """Platform topics: set initial state and handle clearing. Must be called under self._lock.

        Activation is intentionally NOT done here for runtime transitions — the serial reader
        handles that with immediate effect. Topics are too slow for reliable press detection.
        This method is responsible for:
          1. Seeding the initial state on startup.
          2. Confirming active if a topic press arrives (missed serial edge safety net).
          3. Clearing _extern_estop_active — the ONLY path that can clear it.
        """
        if self._extern_emergency_stop is None:
            return  # emergency_stop topic not yet received

        # Consider seeded once we have both topic values (or StopStatus is unavailable).
        stop_status_available = StopStatus is not None
        fully_seeded = not stop_status_available or self._extern_needs_reset is not None

        if not self._extern_seeded and fully_seeded:
            self._extern_seeded = True
            platform_pressed = self._extern_emergency_stop or (self._extern_needs_reset is True)
            self.get_logger().warn(
                f"Initial external e-stop state (platform topics): "
                f"{'ACTIVE' if platform_pressed else 'CLEAR'} "
                f"(emergency_stop={self._extern_emergency_stop}, "
                f"needs_reset={self._extern_needs_reset})"
            )

        platform_pressed = self._extern_emergency_stop or (self._extern_needs_reset is True)
        was_active = self._extern_estop_active

        if platform_pressed:
            # Platform confirms pressed → activate (catches initial state and missed serial edges).
            self._extern_estop_active = True
        elif self._extern_seeded:
            # Platform confirms clear AND we are seeded → this is the only path that clears the flag.
            self._extern_estop_active = False
        # else: not yet seeded — do not clear; stay conservative.

        if self._extern_estop_active != was_active:
            if self._extern_estop_active:
                self.get_logger().error(
                    f"External e-stop ACTIVE (platform topics: "
                    f"emergency_stop={self._extern_emergency_stop}, "
                    f"needs_reset={self._extern_needs_reset})"
                )
            else:
                self.get_logger().info("External e-stop CLEARED (platform topics confirmed).")

    # ── Serial reader (separate thread — fast activation path) ──────────────

    def _read_serial(self):
        """Each received byte is a hardware transition (press or release edge).

        A naive toggle guess tracks which direction this edge is.

        Activation: if the guess says PRESSED, _extern_estop_active is set True immediately —
        this is the fast path, bypassing the slow platform topics.

        Clearing: serial NEVER clears _extern_estop_active. Only platform topics can do that,
        once they confirm emergency_stop=False and needs_reset=False.
        """
        while self._running:
            # Open port — retry until success or node shuts down.
            try:
                self._ser = serial.Serial(self._serial_port, self._serial_baud, timeout=0.1)
                self.get_logger().info(
                    f"Opened serial port {self._serial_port} at {self._serial_baud} baud."
                )
            except serial.SerialException as e:
                self.get_logger().warn(
                    f"Cannot open serial port {self._serial_port}: {e}. Retrying in 2s...",
                    throttle_duration_sec=10,
                )
                time.sleep(2.0)
                continue

            # Read loop — runs until serial error or node shutdown.
            try:
                while self._running:
                    data = self._ser.read(self._ser.in_waiting or 1)
                    if data:
                        with self._lock:
                            self._serial_transition_count += 1
                            self._last_serial_transition_time = time.monotonic()
                            # On the first transition, seed the guess to the CURRENT platform
                            # state so the toggle lands on the NEXT (opposite) state.
                            # Example: platform is CLEAR → seed=CLEAR → toggle → PRESSED ✓
                            # The original test.py used `not current` here (toggling to match
                            # the platform), which was correct for cross-check logging but is
                            # inverted for fast activation: it caused the first PRESS byte to
                            # be detected as CLEAR when the platform was idle.
                            if self._serial_guess_pressed is None:
                                self._serial_guess_pressed = (
                                    self._extern_estop_active  # seed = current state
                                    if self._extern_emergency_stop is not None
                                    else False  # unknown → seed CLEAR → first toggle = PRESSED (fail-safe)
                                )
                            self._serial_guess_pressed = not self._serial_guess_pressed
                            guess = self._serial_guess_pressed
                            count = self._serial_transition_count

                            # Fast-path activation: serial detected a PRESS → freeze immediately.
                            # A CLEAR guess does NOT clear the flag; only topics can do that.
                            was_active = self._extern_estop_active
                            if guess:
                                self._extern_estop_active = True

                            activated_now = self._extern_estop_active and not was_active
                            platform_active = self._extern_estop_active
                            platform_known = self._extern_emergency_stop is not None

                        guess_str = "PRESSED" if guess else "CLEAR"
                        if activated_now:
                            self.get_logger().error(
                                f"Serial transition #{count} → {guess_str} — "
                                "extern e-stop ACTIVATED! "
                                "(freeze engaged; clears only when platform topics confirm CLEAR)"
                            )
                        else:
                            if platform_known:
                                match_str = (
                                    "MATCHES platform"
                                    if guess == platform_active
                                    else f"DISAGREES with platform "
                                    f"({'ACTIVE' if platform_active else 'CLEAR'})"
                                )
                            else:
                                match_str = "platform state unknown (not yet seeded)"
                            self.get_logger().info(
                                f"Serial transition #{count} → guess: {guess_str} [{match_str}]"
                            )
            except serial.SerialException as e:
                self.get_logger().error(
                    f"Serial read error on {self._serial_port}: {e}. Reconnecting..."
                )
            finally:
                if self._ser is not None and self._ser.is_open:
                    self._ser.close()
                self._ser = None

    # ── Gamepad callback ─────────────────────────────────────────────────────

    def joy_callback(self, msg: Joy):
        self.last_joy_received_time = self.get_clock().now()

        if not self.gamepad_connected:
            self.get_logger().info("Gamepad connected.")
            self.gamepad_connected = True

        if self.emergency_stop_button is None or self.emergency_stop_button >= len(msg.buttons):
            return

        self.e_stop_pressed_state = bool(msg.buttons[self.emergency_stop_button])

    # ── Control loop ─────────────────────────────────────────────────────────

    def control_loop(self):
        now = self.get_clock().now()

        with self._lock:
            extern_active = self._extern_estop_active

        # External e-stop overrides everything: keep freeze engaged.
        if extern_active:
            self.get_logger().warning(
                "External e-stop active — freeze engaged.", throttle_duration_sec=10
            )
            self.set_freeze_controller(True)
            # Reset internal state so gamepad logic resumes cleanly once extern clears.
            self.e_stop_button_release = True
            self.emergency_stop_pressed_time = None
            return

        # No gamepad connected
        if not self.gamepad_connected:
            self.get_logger().warning("Waiting for gamepad to connect.", throttle_duration_sec=15)

        # Gamepad disconnected (1 s grace period before assuming disconnection)
        if self.gamepad_connected and (now - self.last_joy_received_time) > Duration(seconds=1.0):
            if (now - self.last_toggle_time).nanoseconds / 10**9 >= 2:
                self.get_logger().warn("Gamepad disconnected! Activating freeze_controller.")
                self.set_freeze_controller(True)
                self.gamepad_connected = False

        # Gamepad e-stop: button just pressed
        if self.e_stop_pressed_state and self.e_stop_button_release:
            self.get_logger().error("EMERGENCY STOP ACTIVATED!")
            self.set_freeze_controller(True)
            self.e_stop_button_release = False
            self.emergency_stop_pressed_time = now

        # Gamepad e-stop: button held 3 sec → deactivate (only when extern is also clear)
        elif self.e_stop_pressed_state and self.emergency_stop_pressed_time:
            hold_duration = (now - self.emergency_stop_pressed_time).nanoseconds / 1e9
            if hold_duration >= 3.0:
                if self.was_freeze_controller_active_before and not extern_active:
                    self.get_logger().info(
                        "Emergency stop button held for 3 sec — deactivating freeze_controller."
                    )
                    self.set_freeze_controller(False)
                self.emergency_stop_pressed_time = None

        # Button fully released
        elif not self.e_stop_pressed_state and not self.e_stop_button_release:
            self.e_stop_button_release = True
            self.emergency_stop_pressed_time = None

    # ── Controller management ────────────────────────────────────────────────

    def update_freeze_controllers_status(self):
        if not self.list_controllers_client.service_is_ready():
            self.get_logger().warn(
                "Controller Manager service not available for status update.",
                throttle_duration_sec=10,
            )
            return

        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)
        future.add_done_callback(self._handle_status_update_response)

    def _handle_status_update_response(self, future):
        try:
            status = {}
            result = future.result()
            if result:
                for controller in result.controller:
                    if controller.name.startswith("freeze_controller"):
                        status[controller.name] = controller.state
            self.freeze_controller_states = status
            # Keep was_freeze_controller_active_before in sync with the actual hardware state.
            # Without this, if the cache was empty when set_freeze_controller(True) first ran
            # (node startup race), the flag would stay False and the 3-second hold would
            # silently refuse to deactivate the freeze.
            self.was_freeze_controller_active_before = any(
                state == "active" for state in status.values()
            )
            self.get_logger().debug(f"Freeze controllers and states (periodic): {status}")
        except Exception as e:
            self.get_logger().warn(f"Failed to update freeze controller states: {e}")

    def get_freeze_controllers_status(self):
        return self.freeze_controller_states.copy()

    def set_freeze_controller(self, active: bool):
        # Skip if we already sent this exact request and are waiting for the response.
        # _freeze_target is reset to None on failure so that a retry is allowed.
        if self._freeze_target == active:
            return

        if not self.switch_controller_client.service_is_ready():
            self.get_logger().error(
                "Controller Manager service not available.", throttle_duration_sec=10
            )
            return  # Do not set _freeze_target — allows retry on next call.

        controller_names = list(self.freeze_controller_states.keys())
        if not controller_names:
            self.get_logger().warn(
                "No freeze_controller found to (de)activate — waiting for controller to load.",
                throttle_duration_sec=10,
            )
            return  # Do not set _freeze_target — allows retry once the status is populated.

        self._freeze_target = active  # Commit: suppress duplicates until response or failure.

        request = SwitchController.Request()
        if active:
            request.activate_controllers = controller_names
            request.deactivate_controllers = []
        else:
            request.activate_controllers = []
            request.deactivate_controllers = controller_names
        request.strictness = SwitchController.Request.BEST_EFFORT

        future = self.switch_controller_client.call_async(request)
        future.add_done_callback(
            lambda future: self.handle_switch_response(future, active, controller_names)
        )

    def handle_switch_response(self, future, activate, controller_names):
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info(
                    f"Freeze Controllers {'Activated' if activate else 'Deactivated'}: {controller_names}"
                )
            else:
                self.get_logger().warn("Freeze Controller switch failed. Checking actual state...")
                self._freeze_target = None  # Allow retry on next control_loop iteration.

            status = self.get_freeze_controllers_status()
            for name, state in status.items():
                self.get_logger().debug(f"Freeze controller '{name}' is in state: {state}")

            self.was_freeze_controller_active_before = any(
                state == "active" for state in status.values()
            )

        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}. Checking actual state...")
            self._freeze_target = None  # Allow retry on next control_loop iteration.
            status = self.get_freeze_controllers_status()
            for name, state in status.items():
                self.get_logger().debug(f"Freeze controller '{name}' is in state: {state}")
            self.was_freeze_controller_active_before = any(
                state == "active" for state in status.values()
            )

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
