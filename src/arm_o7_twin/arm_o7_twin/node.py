"""ROS 2 node for the fail-closed ARM1.5/O7 command arbiter."""

from __future__ import annotations

import json
import math
import time
from typing import Dict, Optional, Tuple

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .core import (
    ARM_JOINTS,
    CANONICAL_JOINTS,
    HAND_JOINTS,
    GateDecision,
    JointLimit,
    JointStateSnapshot,
    Mode,
    SafetyConfig,
    TrajectoryData,
    TrajectoryPointData,
    ValidationError,
    default_joint_limits,
    evaluate_command,
    is_fresh,
    make_joint_state_snapshot,
    make_shadow_trajectory,
    routes_for_mode,
    shadow_publish_due,
    split_trajectory,
    tracking_error,
)


class TwinArbiter(Node):
    """Validate, interlock, split, and route canonical joint trajectories."""

    def __init__(self) -> None:
        super().__init__("arm_o7_twin_arbiter")
        self._declare_parameters()
        self._config = self._read_safety_config()
        self._enable_real_output = bool(self.get_parameter("enable_real_output").value)
        self._shadow_publish_rate_hz = float(
            self.get_parameter("shadow_publish_rate_hz").value
        )
        self._shadow_trajectory_duration_sec = float(
            self.get_parameter("shadow_trajectory_duration_sec").value
        )
        if (
            not math.isfinite(self._shadow_publish_rate_hz)
            or self._shadow_publish_rate_hz <= 0.0
        ):
            raise ValueError("shadow_publish_rate_hz must be finite and greater than zero")
        if (
            not math.isfinite(self._shadow_trajectory_duration_sec)
            or self._shadow_trajectory_duration_sec <= 0.0
        ):
            raise ValueError(
                "shadow_trajectory_duration_sec must be finite and greater than zero"
            )
        self._mode = Mode.SAFE_IDLE
        self._fault_latched = False
        self._fault_reason = ""
        self._last_rejection = ""
        self._last_command_monotonic: Optional[float] = None
        self._last_shadow_publish_monotonic: Optional[float] = None
        self._last_heartbeat_monotonic: Optional[float] = None
        self._last_deadman_monotonic: Optional[float] = None
        self._deadman_pressed = False
        self._sim_state: Optional[JointStateSnapshot] = None
        self._real_state: Optional[JointStateSnapshot] = None
        self._tracked_joints: Tuple[str, ...] = ()
        self._accepted_count = 0
        self._rejected_count = 0
        self._shadow_publish_count = 0

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        topic = self._topic
        self._sim_arm_pub = self.create_publisher(
            JointTrajectory, topic("sim_arm_output"), reliable
        )
        self._sim_hand_pub = self.create_publisher(
            JointTrajectory, topic("sim_hand_output"), reliable
        )
        self._real_arm_pub = self.create_publisher(
            JointTrajectory, topic("real_arm_output"), reliable
        )
        self._real_hand_pub = self.create_publisher(
            JointTrajectory, topic("real_hand_output"), reliable
        )
        self._status_pub = self.create_publisher(String, topic("status"), latched)
        self._mode_pub = self.create_publisher(String, topic("mode_state"), latched)
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", reliable
        )

        self.create_subscription(
            JointTrajectory, topic("command_input"), self._on_command, reliable
        )
        self.create_subscription(Bool, topic("deadman"), self._on_deadman, reliable)
        self.create_subscription(Empty, topic("heartbeat"), self._on_heartbeat, reliable)
        self.create_subscription(String, topic("mode_command"), self._on_mode_command, reliable)
        self.create_subscription(
            JointState, topic("sim_joint_states"), self._on_sim_state, state_qos
        )
        self.create_subscription(
            JointState, topic("real_joint_states"), self._on_real_state, state_qos
        )

        for mode in Mode:
            service_name = f"~/mode/{mode.value.lower()}"
            self.create_service(Trigger, service_name, self._mode_service_callback(mode))
        self.create_service(Trigger, "~/clear_fault", self._on_clear_fault)

        status_period = float(self.get_parameter("status_period_sec").value)
        if not math.isfinite(status_period) or status_period <= 0.0:
            raise ValueError("status_period_sec must be finite and greater than zero")
        self.create_timer(min(status_period, 0.1), self._safety_watchdog)
        self.create_timer(status_period, self._publish_status)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        real_state = "ENABLED" if self._enable_real_output else "disabled"
        self.get_logger().warning(
            f"Arbiter started in SAFE_IDLE; real output is {real_state}"
        )
        self._publish_status()

    def _declare_parameters(self) -> None:
        self.declare_parameter("enable_real_output", False)
        self.declare_parameter("heartbeat_timeout_sec", 0.30)
        self.declare_parameter("deadman_timeout_sec", 0.30)
        self.declare_parameter("joint_state_timeout_sec", 0.30)
        self.declare_parameter("start_state_tolerance_rad", 0.10)
        self.declare_parameter("tracking_error_tolerance_rad", 0.20)
        self.declare_parameter("require_all_command_joints", False)
        self.declare_parameter("require_complete_controller_groups", True)
        self.declare_parameter("status_period_sec", 0.5)
        self.declare_parameter("shadow_publish_rate_hz", 20.0)
        self.declare_parameter("shadow_trajectory_duration_sec", 0.10)

        defaults = {
            "command_input": "/twin/joint_trajectory",
            "deadman": "/twin/deadman",
            "heartbeat": "/twin/heartbeat",
            "mode_command": "/twin/mode_command",
            "mode_state": "/twin/mode",
            "status": "/twin/status",
            "sim_joint_states": "/sim/joint_states",
            "real_joint_states": "/real/joint_states",
            "sim_arm_output": "/sim/arm_controller/joint_trajectory",
            "sim_hand_output": "/sim/hand_controller/joint_trajectory",
            "real_arm_output": "/real/arm_controller/joint_trajectory",
            "real_hand_output": "/real/hand_controller/joint_trajectory",
        }
        for name, value in defaults.items():
            self.declare_parameter(f"topics.{name}", value)

        for name, limit in default_joint_limits().items():
            self.declare_parameter(f"joint_limits.{name}.min", limit.minimum)
            self.declare_parameter(f"joint_limits.{name}.max", limit.maximum)

    def _read_safety_config(self) -> SafetyConfig:
        limits: Dict[str, JointLimit] = {}
        for name in CANONICAL_JOINTS:
            minimum = float(self.get_parameter(f"joint_limits.{name}.min").value)
            maximum = float(self.get_parameter(f"joint_limits.{name}.max").value)
            limits[name] = JointLimit(minimum, maximum)
        return SafetyConfig(
            joint_limits=limits,
            heartbeat_timeout_sec=float(self.get_parameter("heartbeat_timeout_sec").value),
            deadman_timeout_sec=float(self.get_parameter("deadman_timeout_sec").value),
            joint_state_timeout_sec=float(
                self.get_parameter("joint_state_timeout_sec").value
            ),
            start_state_tolerance_rad=float(
                self.get_parameter("start_state_tolerance_rad").value
            ),
            tracking_error_tolerance_rad=float(
                self.get_parameter("tracking_error_tolerance_rad").value
            ),
            require_all_command_joints=bool(
                self.get_parameter("require_all_command_joints").value
            ),
            require_complete_controller_groups=bool(
                self.get_parameter("require_complete_controller_groups").value
            ),
        )

    def _topic(self, key: str) -> str:
        return str(self.get_parameter(f"topics.{key}").value)

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _on_set_parameters(self, parameters) -> SetParametersResult:
        protected = {
            "enable_real_output",
            "heartbeat_timeout_sec",
            "deadman_timeout_sec",
            "joint_state_timeout_sec",
            "start_state_tolerance_rad",
            "tracking_error_tolerance_rad",
            "require_all_command_joints",
            "require_complete_controller_groups",
            "status_period_sec",
            "shadow_publish_rate_hz",
            "shadow_trajectory_duration_sec",
        }
        protected.update(
            f"joint_limits.{name}.{bound}"
            for name in CANONICAL_JOINTS
            for bound in ("min", "max")
        )
        changed = {parameter.name for parameter in parameters}
        if changed.intersection(protected):
            return SetParametersResult(
                successful=False,
                reason="safety parameters are startup-only; restart in SAFE_IDLE",
            )
        if any(name.startswith("topics.") for name in changed):
            return SetParametersResult(
                successful=False,
                reason="topic parameters are startup-only",
            )
        return SetParametersResult(successful=True)

    def _on_heartbeat(self, _message: Empty) -> None:
        self._last_heartbeat_monotonic = self._now()

    def _on_deadman(self, message: Bool) -> None:
        was_pressed = self._deadman_pressed
        self._deadman_pressed = bool(message.data)
        self._last_deadman_monotonic = self._now()
        if was_pressed and not self._deadman_pressed and self._mode is not Mode.SAFE_IDLE:
            self._trip("deadman released")

    def _on_sim_state(self, message: JointState) -> None:
        self._sim_state = self._state_from_message("sim", message)

    def _on_real_state(self, message: JointState) -> None:
        self._real_state = self._state_from_message("real", message)
        if self._mode is Mode.SHADOW and self._real_state is not None:
            self._mirror_real_state_to_sim(self._real_state)

    def _state_from_message(
        self, side: str, message: JointState
    ) -> Optional[JointStateSnapshot]:
        try:
            return make_joint_state_snapshot(message.name, message.position, self._now())
        except ValidationError as exc:
            self._last_rejection = f"invalid {side} joint state: {exc}"
            self.get_logger().error(self._last_rejection)
            if self._mode is not Mode.SAFE_IDLE:
                self._trip(self._last_rejection)
            return None

    @staticmethod
    def _trajectory_from_message(message: JointTrajectory) -> TrajectoryData:
        points = tuple(
            TrajectoryPointData(
                positions=tuple(point.positions),
                velocities=tuple(point.velocities),
                accelerations=tuple(point.accelerations),
                effort=tuple(point.effort),
                time_from_start_ns=(
                    int(point.time_from_start.sec) * 1_000_000_000
                    + int(point.time_from_start.nanosec)
                ),
            )
            for point in message.points
        )
        return TrajectoryData(
            joint_names=tuple(message.joint_names),
            points=points,
            frame_id=message.header.frame_id,
            stamp_sec=int(message.header.stamp.sec),
            stamp_nanosec=int(message.header.stamp.nanosec),
        )

    @staticmethod
    def _trajectory_to_message(trajectory: TrajectoryData) -> JointTrajectory:
        message = JointTrajectory()
        message.header.frame_id = trajectory.frame_id
        message.header.stamp.sec = trajectory.stamp_sec
        message.header.stamp.nanosec = trajectory.stamp_nanosec
        message.joint_names = list(trajectory.joint_names)
        for source in trajectory.points:
            point = JointTrajectoryPoint()
            point.positions = list(source.positions)
            point.velocities = list(source.velocities)
            point.accelerations = list(source.accelerations)
            point.effort = list(source.effort)
            point.time_from_start.sec = source.time_from_start_ns // 1_000_000_000
            point.time_from_start.nanosec = source.time_from_start_ns % 1_000_000_000
            message.points.append(point)
        return message

    def _on_command(self, message: JointTrajectory) -> None:
        if self._mode is Mode.SHADOW:
            self._reject(
                GateDecision(
                    False,
                    "SHADOW is driven by /real/joint_states; canonical commands are ignored",
                )
            )
            return
        trajectory = self._trajectory_from_message(message)
        decision = evaluate_command(
            mode=self._mode,
            trajectory=trajectory,
            config=self._config,
            now_monotonic=self._now(),
            deadman_pressed=self._deadman_pressed,
            deadman_monotonic=self._last_deadman_monotonic,
            heartbeat_monotonic=self._last_heartbeat_monotonic,
            sim_state=self._sim_state,
            real_state=self._real_state,
            enable_real_output=self._enable_real_output,
        )
        if not decision.accepted:
            self._reject(decision)
            return

        arm = split_trajectory(trajectory, ARM_JOINTS)
        hand = split_trajectory(trajectory, HAND_JOINTS)
        if arm is None and hand is None:
            self._reject(GateDecision(False, "trajectory did not contain routed joints"))
            return

        if decision.route_sim:
            self._publish_split(arm, hand, self._sim_arm_pub, self._sim_hand_pub)
        if decision.route_real:
            # This branch cannot be reached unless enable_real_output was true
            # at startup and all real-side interlocks passed.
            self._publish_split(arm, hand, self._real_arm_pub, self._real_hand_pub)
        self._tracked_joints = trajectory.joint_names
        self._last_command_monotonic = self._now()
        self._accepted_count += 1
        self._last_rejection = ""

    def _mirror_real_state_to_sim(self, snapshot: JointStateSnapshot) -> None:
        """Rate-limit, validate, and mirror a real state only to simulation."""

        now = self._now()
        if not shadow_publish_due(
            self._last_shadow_publish_monotonic,
            now,
            self._shadow_publish_rate_hz,
        ):
            return
        try:
            trajectory = make_shadow_trajectory(
                snapshot, self._shadow_trajectory_duration_sec, self._config
            )
        except (ValidationError, ValueError) as exc:
            self._reject(GateDecision(False, f"shadow state rejected: {exc}"))
            if self._mode is not Mode.SAFE_IDLE:
                self._trip(f"shadow state rejected: {exc}")
            return

        decision = evaluate_command(
            mode=Mode.SHADOW,
            trajectory=trajectory,
            config=self._config,
            now_monotonic=now,
            deadman_pressed=self._deadman_pressed,
            deadman_monotonic=self._last_deadman_monotonic,
            heartbeat_monotonic=self._last_heartbeat_monotonic,
            sim_state=self._sim_state,
            real_state=self._real_state,
            enable_real_output=self._enable_real_output,
        )
        if not decision.accepted:
            self._reject(decision)
            return

        arm = split_trajectory(trajectory, ARM_JOINTS)
        hand = split_trajectory(trajectory, HAND_JOINTS)
        # SHADOW has a one-way invariant: only simulation publishers appear in
        # this method. Never add a real publisher here.
        self._publish_split(arm, hand, self._sim_arm_pub, self._sim_hand_pub)
        self._tracked_joints = trajectory.joint_names
        self._last_shadow_publish_monotonic = now
        self._last_command_monotonic = now
        self._shadow_publish_count += 1
        self._accepted_count += 1
        self._last_rejection = ""

    def _publish_split(self, arm, hand, arm_publisher, hand_publisher) -> None:
        if arm is not None:
            arm_publisher.publish(self._trajectory_to_message(arm))
        if hand is not None:
            hand_publisher.publish(self._trajectory_to_message(hand))

    def _reject(self, decision: GateDecision) -> None:
        self._rejected_count += 1
        self._last_rejection = decision.reason
        self.get_logger().warning(f"Command rejected: {decision.reason}")
        safety_markers = (
            "deadman",
            "heartbeat",
            "missing or stale",
            "start-state mismatch",
            "tracking error",
            "real output is disabled",
        )
        if self._mode is not Mode.SAFE_IDLE and any(
            marker in decision.reason for marker in safety_markers
        ):
            self._trip(decision.reason)
        else:
            self._publish_status()

    def _on_mode_command(self, message: String) -> None:
        try:
            target = Mode.parse(message.data)
        except ValueError as exc:
            self._last_rejection = str(exc)
            self.get_logger().warning(self._last_rejection)
            self._publish_status()
            return
        success, reason = self._request_mode(target)
        if not success:
            self.get_logger().warning(f"Mode request rejected: {reason}")

    def _mode_service_callback(self, target: Mode):
        def callback(_request: Trigger.Request, response: Trigger.Response):
            response.success, response.message = self._request_mode(target)
            return response

        return callback

    def _request_mode(self, target: Mode) -> Tuple[bool, str]:
        if target is Mode.SAFE_IDLE:
            self._mode = Mode.SAFE_IDLE
            self._tracked_joints = ()
            self._publish_status()
            return True, "mode is SAFE_IDLE"
        if target is self._mode:
            return True, f"mode is already {target.value}"
        if self._fault_latched:
            reason = f"fault is latched: {self._fault_reason}"
            self._last_rejection = reason
            self._publish_status()
            return False, reason
        if self._mode is not Mode.SAFE_IDLE:
            reason = "active-mode transitions must pass through SAFE_IDLE"
            self._last_rejection = reason
            self._publish_status()
            return False, reason
        route_sim, route_real = routes_for_mode(target)
        if route_real and not self._enable_real_output:
            reason = "real output is disabled by startup configuration"
            self._last_rejection = reason
            self._publish_status()
            return False, reason
        now = self._now()
        if not self._deadman_pressed:
            reason = "deadman must be pressed before entering an active mode"
            self._last_rejection = reason
            self._publish_status()
            return False, reason
        if self._last_deadman_monotonic is None or (
            now - self._last_deadman_monotonic > self._config.deadman_timeout_sec
        ):
            reason = "a fresh deadman sample is required before entering an active mode"
            self._last_rejection = reason
            self._publish_status()
            return False, reason
        if self._last_heartbeat_monotonic is None or (
            now - self._last_heartbeat_monotonic > self._config.heartbeat_timeout_sec
        ):
            reason = "a fresh heartbeat is required before entering an active mode"
            self._last_rejection = reason
            self._publish_status()
            return False, reason
        required_states = []
        if route_sim:
            required_states.append(("sim", self._sim_state))
        if route_real or target is Mode.SHADOW:
            required_states.append(("real", self._real_state))
        for side, snapshot in required_states:
            if not is_fresh(snapshot, now, self._config.joint_state_timeout_sec):
                reason = f"fresh {side} joint state required before entering {target.value}"
                self._last_rejection = reason
                self._publish_status()
                return False, reason
        if target in (Mode.SHADOW, Mode.TWIN_COMMAND):
            assert self._sim_state is not None and self._real_state is not None
            error, missing = tracking_error(
                self._sim_state, self._real_state, CANONICAL_JOINTS
            )
            if missing:
                reason = (
                    "complete sim and real joint states are required for tracking; "
                    f"missing: {', '.join(missing)}"
                )
                self._last_rejection = reason
                self._publish_status()
                return False, reason
            allowed_error = self._config.tracking_error_tolerance_rad
            if target is Mode.SHADOW:
                allowed_error = min(
                    allowed_error, self._config.start_state_tolerance_rad
                )
            if error > allowed_error:
                reason = (
                    f"sim-real tracking error {error:.4f} rad exceeds "
                    f"{allowed_error:.4f} rad"
                )
                self._last_rejection = reason
                self._publish_status()
                return False, reason
        self._mode = target
        if target is Mode.SHADOW:
            self._last_shadow_publish_monotonic = None
        self._last_rejection = ""
        self.get_logger().info(f"Mode changed to {target.value}")
        self._publish_status()
        return True, f"mode changed to {target.value}"

    def _on_clear_fault(self, _request: Trigger.Request, response: Trigger.Response):
        if self._mode is not Mode.SAFE_IDLE:
            response.success = False
            response.message = "enter SAFE_IDLE before clearing a fault"
        elif self._deadman_pressed:
            response.success = False
            response.message = "release the deadman before clearing a fault"
        else:
            self._fault_latched = False
            self._fault_reason = ""
            self._last_rejection = ""
            response.success = True
            response.message = "fault cleared; active mode still requires fresh safety inputs"
        self._publish_status()
        return response

    def _trip(self, reason: str) -> None:
        if self._fault_latched and self._fault_reason == reason:
            return
        previous_mode = self._mode
        self._fault_latched = True
        self._fault_reason = reason
        self._last_rejection = reason
        self._mode = Mode.SAFE_IDLE
        self._tracked_joints = ()
        self.get_logger().error(
            f"Safety interlock tripped from {previous_mode.value}: {reason}"
        )
        self._publish_status()

    def _safety_watchdog(self) -> None:
        if self._mode is Mode.SAFE_IDLE:
            return
        now = self._now()
        if not self._deadman_pressed:
            self._trip("deadman is not pressed")
            return
        if self._last_deadman_monotonic is None or (
            now - self._last_deadman_monotonic > self._config.deadman_timeout_sec
        ):
            self._trip("deadman timeout")
            return
        if self._last_heartbeat_monotonic is None or (
            now - self._last_heartbeat_monotonic > self._config.heartbeat_timeout_sec
        ):
            self._trip("heartbeat timeout")
            return
        route_sim, route_real = routes_for_mode(self._mode)
        if route_sim and not is_fresh(
            self._sim_state, now, self._config.joint_state_timeout_sec
        ):
            self._trip("sim joint-state timeout")
            return
        if (route_real or self._mode is Mode.SHADOW) and not is_fresh(
            self._real_state, now, self._config.joint_state_timeout_sec
        ):
            self._trip("real joint-state timeout")
            return
        if self._mode in (Mode.SHADOW, Mode.TWIN_COMMAND):
            assert self._sim_state is not None and self._real_state is not None
            error, missing = tracking_error(
                self._sim_state, self._real_state, CANONICAL_JOINTS
            )
            if missing:
                self._trip(f"tracking state lost joints: {', '.join(missing)}")
            elif error > self._config.tracking_error_tolerance_rad:
                self._trip(
                    f"sim-real tracking error {error:.4f} rad exceeds "
                    f"{self._config.tracking_error_tolerance_rad:.4f} rad"
                )

    def _status_payload(self) -> Dict[str, object]:
        now = self._now()

        def age(stamp: Optional[float]):
            return None if stamp is None else round(max(0.0, now - stamp), 4)

        tracking = None
        if self._sim_state is not None and self._real_state is not None:
            value, missing = tracking_error(
                self._sim_state, self._real_state, CANONICAL_JOINTS
            )
            tracking = None if missing else round(value, 6)
        return {
            "mode": self._mode.value,
            "fault_latched": self._fault_latched,
            "fault_reason": self._fault_reason,
            "interlock_reason": self._fault_reason or self._last_rejection,
            "deadman": self._deadman_pressed,
            "real_output_enabled": self._enable_real_output,
            "heartbeat_age_sec": age(self._last_heartbeat_monotonic),
            "deadman_age_sec": age(self._last_deadman_monotonic),
            "sim_state_age_sec": age(
                None if self._sim_state is None else self._sim_state.received_monotonic
            ),
            "real_state_age_sec": age(
                None if self._real_state is None else self._real_state.received_monotonic
            ),
            "tracking_error_rad": tracking,
            "last_command_joints": list(self._tracked_joints),
            "last_command_age_sec": age(self._last_command_monotonic),
            "last_shadow_publish_age_sec": age(
                self._last_shadow_publish_monotonic
            ),
            "last_rejection": self._last_rejection,
            "accepted_commands": self._accepted_count,
            "rejected_commands": self._rejected_count,
            "shadow_publications": self._shadow_publish_count,
        }

    def _publish_status(self) -> None:
        payload = self._status_payload()
        status_message = String()
        status_message.data = json.dumps(payload, sort_keys=True, separators=(",", ":"))
        self._status_pub.publish(status_message)
        mode_message = String()
        mode_message.data = self._mode.value
        self._mode_pub.publish(mode_message)

        diagnostic = DiagnosticStatus()
        diagnostic.name = f"{self.get_name()}: safety"
        diagnostic.hardware_id = "arm1.5_linker_o7"
        if self._fault_latched:
            diagnostic.level = DiagnosticStatus.ERROR
            diagnostic.message = self._fault_reason
        elif self._last_rejection:
            diagnostic.level = DiagnosticStatus.WARN
            diagnostic.message = self._last_rejection
        elif self._mode is Mode.SAFE_IDLE:
            diagnostic.level = DiagnosticStatus.WARN
            diagnostic.message = "SAFE_IDLE"
        else:
            diagnostic.level = DiagnosticStatus.OK
            diagnostic.message = self._mode.value
        diagnostic.values = [
            KeyValue(key=str(key), value=str(value)) for key, value in payload.items()
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [diagnostic]
        self._diagnostics_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwinArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
