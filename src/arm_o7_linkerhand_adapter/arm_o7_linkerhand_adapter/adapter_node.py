"""Fail-closed trajectory and state adapter for a right LinkerHand O7."""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty
from trajectory_msgs.msg import JointTrajectory

from .core import (
    AdapterError,
    HAND_JOINTS,
    JointMap,
    canonical_to_sdk,
    sample_trajectory,
    sdk_to_canonical,
    validate_maps,
    validate_trajectory,
)


DEFAULTS = {
    "thumb_cmc_roll": (6, 1.54 / 1.1339, 0.0, 0.0, 1.1339),
    "thumb_cmc_yaw": (1, -1.43 / 1.9189, 0.0, 0.0, 1.9189),
    "thumb_cmc_pitch": (0, 0.75 / 0.5146, 0.0, 0.0, 0.5146),
    "index_mcp_pitch": (2, 1.62 / 1.3607, 0.0, 0.0, 1.3607),
    "middle_mcp_pitch": (3, 1.62 / 1.3607, 0.0, 0.0, 1.3607),
    "ring_mcp_pitch": (4, 1.62 / 1.3607, 0.0, 0.0, 1.3607),
    "pinky_mcp_pitch": (5, 1.62 / 1.3607, 0.0, 0.0, 1.3607),
}


class O7Adapter(Node):
    def __init__(self) -> None:
        super().__init__("o7_linkerhand_adapter")
        self._declare_parameters()
        self._maps = self._read_maps()
        self._mapping_verified = bool(self.get_parameter("mapping_verified").value)
        self._command_enabled = bool(self.get_parameter("command_enabled").value)
        self._heartbeat_timeout = self._positive("heartbeat_timeout_sec")
        self._deadman_timeout = self._positive("deadman_timeout_sec")
        self._state_timeout = self._positive("sdk_state_timeout_sec")
        self._max_duration = self._positive("max_trajectory_duration_sec")
        rate = self._positive("output_rate_hz")
        if rate > 30.0:
            raise ValueError("output_rate_hz cannot exceed the SDK limit of 30 Hz")

        self._last_heartbeat: Optional[float] = None
        self._last_deadman: Optional[float] = None
        self._deadman = False
        self._last_state_time: Optional[float] = None
        self._last_positions: Optional[Tuple[float, ...]] = None
        self._trajectory = None
        self._trajectory_start: Optional[float] = None
        self._initial_positions: Optional[Tuple[float, ...]] = None

        self._sdk_pub = self.create_publisher(
            JointState, str(self.get_parameter("topics.sdk_command").value), 10
        )
        self._state_pub = self.create_publisher(
            JointState, str(self.get_parameter("topics.hand_joint_states").value), 10
        )
        self.create_subscription(
            JointTrajectory,
            str(self.get_parameter("topics.trajectory_input").value),
            self._on_trajectory,
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("topics.sdk_state").value),
            self._on_sdk_state,
            10,
        )
        self.create_subscription(
            Bool, str(self.get_parameter("topics.deadman").value), self._on_deadman, 10
        )
        self.create_subscription(
            Empty,
            str(self.get_parameter("topics.heartbeat").value),
            self._on_heartbeat,
            10,
        )
        self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().warning(
            "O7 adapter started fail-closed: command_enabled=%s, mapping_verified=%s"
            % (self._command_enabled, self._mapping_verified)
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("command_enabled", False)
        self.declare_parameter("mapping_verified", False)
        self.declare_parameter("output_rate_hz", 30.0)
        self.declare_parameter("heartbeat_timeout_sec", 0.30)
        self.declare_parameter("deadman_timeout_sec", 0.30)
        self.declare_parameter("sdk_state_timeout_sec", 0.30)
        self.declare_parameter("max_trajectory_duration_sec", 5.0)
        self.declare_parameter("topics.trajectory_input", "/real/hand_controller/joint_trajectory")
        self.declare_parameter("topics.sdk_command", "/cb_right_hand_control_cmd_arc")
        self.declare_parameter("topics.sdk_state", "/cb_right_hand_state_arc")
        self.declare_parameter("topics.hand_joint_states", "/real/hand_joint_states")
        self.declare_parameter("topics.deadman", "/twin/deadman")
        self.declare_parameter("topics.heartbeat", "/twin/heartbeat")
        for name, values in DEFAULTS.items():
            index, scale, offset, minimum, maximum = values
            self.declare_parameter(f"mapping.{name}.sdk_index", index)
            self.declare_parameter(f"mapping.{name}.scale", scale)
            self.declare_parameter(f"mapping.{name}.offset", offset)
            self.declare_parameter(f"mapping.{name}.min", minimum)
            self.declare_parameter(f"mapping.{name}.max", maximum)

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be positive and finite")
        return value

    def _read_maps(self):
        return validate_maps(
            [
                JointMap(
                    name,
                    int(self.get_parameter(f"mapping.{name}.sdk_index").value),
                    float(self.get_parameter(f"mapping.{name}.scale").value),
                    float(self.get_parameter(f"mapping.{name}.offset").value),
                    float(self.get_parameter(f"mapping.{name}.min").value),
                    float(self.get_parameter(f"mapping.{name}.max").value),
                )
                for name in HAND_JOINTS
            ]
        )

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _on_heartbeat(self, _message: Empty) -> None:
        self._last_heartbeat = self._now()

    def _on_deadman(self, message: Bool) -> None:
        self._deadman = bool(message.data)
        self._last_deadman = self._now()
        if not self._deadman:
            self._trajectory = None

    def _fresh(self, timestamp: Optional[float], timeout: float, now: float) -> bool:
        return timestamp is not None and 0.0 <= now - timestamp <= timeout

    def _authorized(self, now: float) -> bool:
        return (
            self._command_enabled
            and self._mapping_verified
            and self._deadman
            and self._fresh(self._last_deadman, self._deadman_timeout, now)
            and self._fresh(self._last_heartbeat, self._heartbeat_timeout, now)
            and self._fresh(self._last_state_time, self._state_timeout, now)
        )

    def _on_trajectory(self, message: JointTrajectory) -> None:
        now = self._now()
        if not self._authorized(now):
            self.get_logger().warning("O7 trajectory rejected: adapter interlock is not ready")
            return
        try:
            points = [
                (
                    point.positions,
                    float(point.time_from_start.sec)
                    + float(point.time_from_start.nanosec) / 1_000_000_000.0,
                )
                for point in message.points
            ]
            self._trajectory = validate_trajectory(
                message.joint_names, points, self._maps, self._max_duration
            )
        except AdapterError as exc:
            self.get_logger().error(f"O7 trajectory rejected: {exc}")
            return
        self._trajectory_start = now
        self._initial_positions = self._last_positions

    def _on_sdk_state(self, message: JointState) -> None:
        try:
            positions = sdk_to_canonical(message.position, self._maps)
        except AdapterError as exc:
            self.get_logger().error(f"invalid O7 SDK state: {exc}")
            return
        self._last_positions = positions
        self._last_state_time = self._now()
        output = JointState()
        output.header.stamp = self.get_clock().now().to_msg()
        output.name = list(HAND_JOINTS)
        output.position = list(positions)
        self._state_pub.publish(output)

    def _on_timer(self) -> None:
        if self._trajectory is None or self._trajectory_start is None:
            return
        now = self._now()
        if not self._authorized(now):
            self._trajectory = None
            return
        try:
            positions, finished = sample_trajectory(
                self._trajectory, now - self._trajectory_start, self._initial_positions
            )
            sdk_positions = canonical_to_sdk(HAND_JOINTS, positions, self._maps)
        except AdapterError as exc:
            self.get_logger().error(f"O7 command stopped: {exc}")
            self._trajectory = None
            return
        output = JointState()
        output.header.stamp = self.get_clock().now().to_msg()
        output.position = list(sdk_positions)
        self._sdk_pub.publish(output)
        if finished:
            self._trajectory = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = O7Adapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
