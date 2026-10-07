"""Glove teleoperation panel: calibrate the glove, then an ON/OFF toggle drives the O7 hand.

Each time the panel starts, the glove must be calibrated (open hand, then fist)
before ON is allowed; it can be redone whenever the panel is OFF. The result is
written to the glove_5dt node's open_raw/closed_raw parameters.

While ON, this node holds the arbiter's deadman, keeps it in SIM_ONLY, and streams
glove-derived hand commands to /twin/joint_trajectory. The arbiter forwards them
to Gazebo; RViz shows the result. It also publishes the unclamped glove target on
/glove_teleop/hand_target. If "also move the real Ti5 hand" is ticked, it
publishes /glove_teleop/real_enable, which lets ti5_bridge drive the real hand.

Turning OFF, closing the window, or this process dying all stop the stream: the
arbiter and the bridge both time out within 0.3 s.
"""

import json
import threading
import time
import tkinter as tk

import rclpy
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arm_o7_glove.mapping import (
    GLOVE_FINGERS,
    HAND_JOINTS,
    calibration_from_samples,
    curls_to_hand_targets,
    step_toward,
)

ARBITER = "/arm_o7_twin_arbiter"
GLOVE_NODE = "/glove_5dt"
STALE_SEC = 0.3
RATE_HZ = 20.0
# Below the arbiter's 0.10 rad start-state tolerance.
MAX_STEP_RAD = 0.08
# 0.08 rad per 0.08 s is the model's 1 rad/s finger velocity limit (sim time).
POINT_TIME_SEC = 0.08

CALIBRATION_COUNTDOWN_SEC = 3
CALIBRATION_RECORD_SEC = 2


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("glove_teleop")
        self.on = False
        self.deadman_held = False
        self.real_wanted = False
        self.calibrated = False
        self.capture = None  # finger -> [raw counts] while recording a calibration pose
        self.curls = None
        self.curls_time = None
        self.sim = None
        self.sim_time = None
        self.arbiter = {}
        self.bridge_status = "not running"
        self.note = "OFF"
        self.sent = 0

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(JointState, "/glove/finger_curls", self._on_curls, 10)
        self.create_subscription(
            JointState, "/sim/joint_states", self._on_sim, qos_profile_sensor_data
        )
        self.create_subscription(String, "/twin/status", self._on_status, latched)
        self.create_subscription(String, "/ti5_bridge/status", self._on_bridge, 10)

        self._heartbeat = self.create_publisher(Empty, "/twin/heartbeat", 10)
        self._deadman = self.create_publisher(Bool, "/twin/deadman", 10)
        self._command = self.create_publisher(JointTrajectory, "/twin/joint_trajectory", 10)
        self._target = self.create_publisher(JointState, "/glove_teleop/hand_target", 10)
        self._real_enable = self.create_publisher(Bool, "/glove_teleop/real_enable", 10)

        self._triggers = {
            name: self.create_client(Trigger, f"{ARBITER}/{path}")
            for name, path in (
                ("sim_only", "mode/sim_only"),
                ("safe_idle", "mode/safe_idle"),
                ("clear_fault", "clear_fault"),
            )
        }
        self._glove_params = self.create_client(
            SetParametersAtomically, f"{GLOVE_NODE}/set_parameters_atomically"
        )
        self.create_timer(1.0 / RATE_HZ, self._tick)

    # -- inputs ---------------------------------------------------------------
    def _on_curls(self, message: JointState) -> None:
        self.curls = dict(zip(message.name, message.position))
        self.curls_time = time.monotonic()
        capture = self.capture
        if capture is not None:
            for finger, raw in zip(message.name, message.effort):
                capture.setdefault(finger, []).append(raw)

    def _on_sim(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in HAND_JOINTS):
            self.sim = positions
            self.sim_time = time.monotonic()

    def _on_status(self, message: String) -> None:
        try:
            self.arbiter = json.loads(message.data)
        except ValueError:
            pass

    def _on_bridge(self, message: String) -> None:
        self.bridge_status = message.data

    @staticmethod
    def _fresh(stamp) -> bool:
        return stamp is not None and time.monotonic() - stamp <= STALE_SEC

    # -- outputs --------------------------------------------------------------
    def _tick(self) -> None:
        self._heartbeat.publish(Empty())
        self._deadman.publish(Bool(data=self.deadman_held))
        self._real_enable.publish(Bool(data=self.on and self.real_wanted))
        if not self.on:
            return
        if self.arbiter.get("mode") != "SIM_ONLY":
            self.note = "waiting for arbiter SIM_ONLY"
            return
        if not self._fresh(self.curls_time) or not all(f in self.curls for f in GLOVE_FINGERS):
            self.note = "no glove data — holding"
            return
        if not self._fresh(self.sim_time):
            self.note = "no sim joint states — holding"
            return
        targets = curls_to_hand_targets(self.curls)

        # The real hand follows the glove directly: the sim fingers are capped
        # at ~0.3 rad/s of wall time on this PC and would hold it back.
        target = JointState()
        target.header.stamp = self.get_clock().now().to_msg()
        target.name = list(HAND_JOINTS)
        target.position = [float(targets[name]) for name in HAND_JOINTS]
        self._target.publish(target)

        command = step_toward(targets, self.sim, MAX_STEP_RAD)
        message = JointTrajectory()
        message.joint_names = list(HAND_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(command[name]) for name in HAND_JOINTS]
        point.time_from_start = Duration(sec=0, nanosec=int(POINT_TIME_SEC * 1e9))
        message.points = [point]
        self._command.publish(message)
        self.sent += 1
        self.note = "streaming"

    def call(self, name: str, done=None) -> None:
        client = self._triggers[name]
        if not client.service_is_ready():
            self.note = f"arbiter service {name} not available (is terminal 2 running?)"
            return
        future = client.call_async(Trigger.Request())

        def finished(result_future) -> None:
            result = result_future.result()
            if result is not None and not result.success:
                self.note = f"{name}: {result.message}"
            if done is not None:
                done(result)

        future.add_done_callback(finished)

    def apply_calibration(self, open_raw, closed_raw, done) -> None:
        """Write the calibration to the glove node; done(ok, message)."""
        if not self._glove_params.service_is_ready():
            done(False, "glove node not running")
            return

        def int_array(name, values):
            return Parameter(
                name=name,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER_ARRAY,
                    integer_array_value=[int(v) for v in values],
                ),
            )

        request = SetParametersAtomically.Request(
            parameters=[int_array("open_raw", open_raw), int_array("closed_raw", closed_raw)]
        )

        def finished(result_future) -> None:
            result = result_future.result()
            if result is None:
                done(False, "glove node did not answer")
                return
            if not result.result.successful:
                done(False, result.result.reason)
            else:
                self.calibrated = True
                done(True, "")

        self._glove_params.call_async(request).add_done_callback(finished)

    def turn_on(self) -> None:
        if not self.calibrated:
            self.note = "calibrate the glove first"
            return

        def enter_sim_only() -> None:
            self.deadman_held = True
            self.on = True
            # Let a few deadman samples reach the arbiter before asking for a mode.
            threading.Timer(0.3, lambda: self.call("sim_only")).start()

        if self.arbiter.get("fault_latched"):
            # clear_fault requires SAFE_IDLE with the deadman released, which is
            # exactly the OFF state we are leaving.
            self.call("clear_fault", lambda _result: enter_sim_only())
        else:
            enter_sim_only()

    def turn_off(self) -> None:
        # Commands and the real-hand enable stop now. The deadman is released only
        # once the arbiter is back in SAFE_IDLE, so the release is not logged as a
        # fault, and after 0.5 s regardless.
        self.on = False
        self.note = "OFF"

        def release(_result=None) -> None:
            if not self.on:  # a quick OFF -> ON must not drop the new deadman
                self.deadman_held = False

        self.call("safe_idle", release)
        threading.Timer(0.5, release).start()


class Panel:
    def __init__(self, node: TeleopNode) -> None:
        self.node = node
        self.calibrating = False
        self.calibration_text = "Not calibrated — click Calibrate glove"
        self.root = tk.Tk()
        self.root.title("Glove → O7 hand")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.calibrate_button = tk.Button(
            self.root, text="Calibrate glove", font=("Sans", 13), command=self.calibrate
        )
        self.calibrate_button.pack(padx=12, pady=(12, 2))
        self.calibration_label = tk.Label(self.root, font=("Sans", 12, "bold"), wraplength=420)
        self.calibration_label.pack(padx=12, pady=(2, 8))

        self.toggle = tk.Button(
            self.root, font=("Sans", 20, "bold"), width=16, height=2, command=self.flip
        )
        self.toggle.pack(padx=12, pady=(6, 6))

        self.real = tk.BooleanVar(value=False)
        tk.Checkbutton(
            self.root,
            text="Also move the REAL Ti5 hand",
            variable=self.real,
            command=self.on_real,
            font=("Sans", 12),
        ).pack(pady=4)

        self.text = tk.Label(self.root, font=("Monospace", 10), justify="left", anchor="w")
        self.text.pack(fill="x", padx=12, pady=(6, 12))
        self.render()

    # -- calibration ----------------------------------------------------------
    def calibrate(self) -> None:
        if self.node.on or self.calibrating:
            return
        self.calibrating = True
        self.node.calibrated = False
        self._countdown(
            "Open your hand flat, fingers straight",
            CALIBRATION_COUNTDOWN_SEC,
            lambda: self._record("open hand", self._after_open),
        )

    def _after_open(self, open_samples) -> None:
        self._countdown(
            "Make a tight fist, thumb bent too",
            CALIBRATION_COUNTDOWN_SEC,
            lambda: self._record("fist", lambda closed: self._finish(open_samples, closed)),
        )

    def _countdown(self, instruction, seconds, then) -> None:
        if seconds == 0:
            then()
            return
        self.calibration_text = f"{instruction} — recording in {seconds}…"
        self.root.after(1000, lambda: self._countdown(instruction, seconds - 1, then))

    def _record(self, pose, then) -> None:
        self.calibration_text = f"Recording {pose} — hold still…"
        self.node.capture = {}

        def stop() -> None:
            samples, self.node.capture = self.node.capture, None
            then(samples)

        self.root.after(CALIBRATION_RECORD_SEC * 1000, stop)

    def _finish(self, open_samples, closed_samples) -> None:
        try:
            open_raw, closed_raw = calibration_from_samples(
                open_samples, closed_samples, self._sensor_order()
            )
        except ValueError as exc:
            self._calibration_done(False, str(exc))
            return
        self.calibration_text = "Saving calibration…"
        self.node.apply_calibration(open_raw, closed_raw, self._calibration_done)

    def _sensor_order(self):
        # The glove node publishes fingers in sensor order, which is the order
        # its open_raw/closed_raw parameters use.
        curls = self.node.curls
        return list(curls) if curls else list(GLOVE_FINGERS)

    def _calibration_done(self, ok, message) -> None:
        # May run on the ROS thread, so only set state; render() draws it.
        if ok:
            self.calibration_text = "Calibrated ✓ — you can turn it ON"
        else:
            self.calibration_text = f"Calibration failed: {message}. Click Calibrate glove to retry."
        self.calibrating = False

    # -- controls ---------------------------------------------------------------
    def flip(self) -> None:
        if self.node.on:
            self.node.turn_off()
        else:
            self.node.turn_on()
        self.render()

    def on_real(self) -> None:
        self.node.real_wanted = bool(self.real.get())

    def render(self) -> None:
        node = self.node
        if not rclpy.ok():  # Ctrl+C / SIGTERM: rclpy's signal handler shut ROS down
            self.root.destroy()
            return
        if node.on:
            self.toggle.config(text="ON — click to stop", bg="#2e7d32", activebackground="#388e3c", fg="white", state="normal")
        elif node.calibrated and not self.calibrating:
            self.toggle.config(text="OFF — click to start", bg="#b71c1c", activebackground="#c62828", fg="white", state="normal")
        else:
            self.toggle.config(text="Calibrate first", bg="#757575", fg="white", state="disabled")
        self.calibrate_button.config(
            state="disabled" if node.on or self.calibrating else "normal"
        )
        self.calibration_label.config(text=self.calibration_text)

        curls = node.curls if node.curls and node._fresh(node.curls_time) else None
        glove = (
            "  ".join(f"{finger[:3]} {curls.get(finger, 0.0):4.2f}" for finger in GLOVE_FINGERS)
            if curls
            else "no data"
        )
        arbiter = node.arbiter
        lines = [
            f"panel:    {node.note}   (commands sent {node.sent})",
            f"glove:    {glove}",
            f"arbiter:  {arbiter.get('mode', 'no status — is terminal 2 running?')}",
        ]
        if arbiter.get("interlock_reason"):
            lines.append(f"          {arbiter['interlock_reason']}")
        lines.append(f"real hand: {node.bridge_status if node.real_wanted else 'not selected'}")
        self.text.config(text="\n".join(lines))
        self.root.after(100, self.render)

    def close(self) -> None:
        if self.node.on:
            self.node.turn_off()
        self.node.real_wanted = False
        time.sleep(0.2)  # let the OFF deadman and real_enable samples go out
        self.root.destroy()


def main() -> None:
    rclpy.init()
    node = TeleopNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        Panel(node).root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
