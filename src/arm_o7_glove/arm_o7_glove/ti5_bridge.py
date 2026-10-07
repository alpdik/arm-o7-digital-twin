"""Drive the real Ti5 hand over RS485 from the glove teleop.

source:=glove (default) follows the panel's glove target (/glove_teleop/hand_target)
directly. The simulated fingers are capped at ~0.3 rad/s of wall time on this PC,
so the real hand leads and the sim catches up.
source:=sim follows the *measured* simulated hand (/sim/joint_states) instead:
it never drifts from what Gazebo and RViz show, but moves at sim speed.

It moves only while all of these are fresh (<= 0.3 s) and true:
  - /glove_teleop/real_enable   (the panel is ON with "real hand" ticked)
  - /twin/mode == SIM_ONLY      (the arbiter is accepting glove commands)
  - the selected source         (a complete hand)
Otherwise it sends nothing and the hand holds its last commanded angle.
Every frame is rate-limited and clamped to 0..90, and the torque cap is applied
at startup.
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from arm_o7_glove.mapping import HAND_JOINTS, ease_angles, hand_to_ti5_angles
from arm_o7_glove.ti5_protocol import FINGERS, Ti5Hand

STALE_SEC = 0.3


class Ti5Bridge(Node):
    def __init__(self) -> None:
        super().__init__("ti5_bridge")
        self.declare_parameter("port", "/dev/ti5_hand")
        self.declare_parameter("torque", 150)  # 0..1000, same default as ti5.py
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("max_step", 6)  # angle units per frame
        self.declare_parameter("source", "glove")  # glove | sim

        port = self.get_parameter("port").value
        torque = int(self.get_parameter("torque").value)
        self._max_step = int(self.get_parameter("max_step").value)
        self._source = self.get_parameter("source").value
        if self._source not in ("glove", "sim"):
            raise ValueError(f"source must be glove or sim, not {self._source!r}")

        self._hand = Ti5Hand(port)
        current, _ = self._hand.all_angles()
        if not current or set(current) != set(FINGERS):
            self._hand.close()
            raise RuntimeError(f"no reply from the Ti5 hand on {port}; is it powered?")
        self._hand.set_torque(torque)
        self._angles = {servo: int(angle) for servo, angle in current.items()}
        self.get_logger().info(
            f"Ti5 hand on {port}, torque {torque}/1000, following {self._source}, "
            f"holding {self._fmt(self._angles)}"
        )

        self._enabled = False
        self._enabled_time = None
        self._mode = ""
        self._hand_target = None
        self._hand_target_time = None
        self._state = "HOLD"
        self._reason = "starting"

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, "/glove_teleop/real_enable", self._on_enable, 10)
        self.create_subscription(String, "/twin/mode", self._on_mode, latched)
        if self._source == "glove":
            self.create_subscription(JointState, "/glove_teleop/hand_target", self._on_hand, 10)
        else:
            self.create_subscription(
                JointState, "/sim/joint_states", self._on_hand, qos_profile_sensor_data
            )
        self._status = self.create_publisher(String, "/ti5_bridge/status", 10)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)
        self.create_timer(0.2, self._publish_status)

    @staticmethod
    def _fmt(angles) -> str:
        return " ".join(f"{FINGERS[servo]}={angle}" for servo, angle in sorted(angles.items()))

    def _on_enable(self, message: Bool) -> None:
        self._enabled = bool(message.data)
        self._enabled_time = time.monotonic()

    def _on_mode(self, message: String) -> None:
        self._mode = message.data

    def _on_hand(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in HAND_JOINTS):
            self._hand_target = positions
            self._hand_target_time = time.monotonic()

    @staticmethod
    def _fresh(stamp) -> bool:
        return stamp is not None and time.monotonic() - stamp <= STALE_SEC

    def _hold_reason(self):
        if not self._fresh(self._enabled_time):
            return "panel not running"
        if not self._enabled:
            return "real hand not enabled in panel"
        if self._mode != "SIM_ONLY":
            return f"arbiter is {self._mode or 'unknown'}"
        if not self._fresh(self._hand_target_time):
            return f"no {self._source} hand data"
        return None

    def _tick(self) -> None:
        reason = self._hold_reason()
        if reason is not None:
            self._state, self._reason = "HOLD", reason
            return
        goal = hand_to_ti5_angles(self._hand_target)
        self._angles = ease_angles(self._angles, goal, self._max_step)
        try:
            self._hand.set_angles(self._angles)
        except Exception as exc:  # serial unplugged etc.: stop and say so
            self._state, self._reason = "ERROR", str(exc)
            self.get_logger().error(f"Ti5 write failed: {exc}", throttle_duration_sec=2.0)
            return
        self._state, self._reason = "ACTIVE", ""

    def _publish_status(self) -> None:
        detail = f" ({self._reason})" if self._reason else ""
        self._status.publish(String(data=f"{self._state}{detail}  {self._fmt(self._angles)}"))

    def close(self) -> None:
        self._hand.close()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = Ti5Bridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()  # the hand holds its last commanded angle
            node.destroy_node()
        rclpy.try_shutdown()
