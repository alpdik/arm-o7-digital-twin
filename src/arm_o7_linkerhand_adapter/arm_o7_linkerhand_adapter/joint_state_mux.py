"""Merge separately owned physical arm and hand state topics."""

from __future__ import annotations

import math
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .core import ARM_JOINTS, HAND_JOINTS, AdapterError, merge_joint_states


class JointStateMux(Node):
    def __init__(self) -> None:
        super().__init__("arm_o7_real_joint_state_mux")
        self.declare_parameter("arm_topic", "/real/arm_joint_states")
        self.declare_parameter("hand_topic", "/real/hand_joint_states")
        self.declare_parameter("output_topic", "/real/joint_states")
        self.declare_parameter("state_timeout_sec", 0.30)
        self.declare_parameter("publish_rate_hz", 30.0)
        self._timeout = self._positive("state_timeout_sec")
        rate = self._positive("publish_rate_hz")
        self._arm: Optional[Dict[str, float]] = None
        self._hand: Optional[Dict[str, float]] = None
        self._arm_time: Optional[float] = None
        self._hand_time: Optional[float] = None
        self._publisher = self.create_publisher(
            JointState, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            JointState, str(self.get_parameter("arm_topic").value), self._on_arm, 10
        )
        self.create_subscription(
            JointState, str(self.get_parameter("hand_topic").value), self._on_hand, 10
        )
        self.create_timer(1.0 / rate, self._publish)

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be positive and finite")
        return value

    @staticmethod
    def _extract(message: JointState, required) -> Dict[str, float]:
        if len(message.name) != len(message.position):
            raise AdapterError("joint state name and position lengths differ")
        values = dict(zip(message.name, (float(v) for v in message.position)))
        missing = set(required).difference(values)
        if missing:
            raise AdapterError(f"joint state is missing {sorted(missing)}")
        selected = {name: values[name] for name in required}
        if any(not math.isfinite(value) for value in selected.values()):
            raise AdapterError("joint state contains a non-finite position")
        return selected

    def _on_arm(self, message: JointState) -> None:
        try:
            self._arm = self._extract(message, ARM_JOINTS)
            self._arm_time = time.monotonic()
        except AdapterError as exc:
            self.get_logger().error(f"invalid physical arm state: {exc}")

    def _on_hand(self, message: JointState) -> None:
        try:
            self._hand = self._extract(message, HAND_JOINTS)
            self._hand_time = time.monotonic()
        except AdapterError as exc:
            self.get_logger().error(f"invalid physical hand state: {exc}")

    def _publish(self) -> None:
        now = time.monotonic()
        if (
            self._arm is None
            or self._hand is None
            or self._arm_time is None
            or self._hand_time is None
            or now - self._arm_time > self._timeout
            or now - self._hand_time > self._timeout
        ):
            return
        names, positions = merge_joint_states(self._arm, self._hand)
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(names)
        message.position = list(positions)
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
