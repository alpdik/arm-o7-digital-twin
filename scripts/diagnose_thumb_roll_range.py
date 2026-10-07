#!/usr/bin/env python3
"""Measure commanded versus reported motion of the O7 thumb CMC roll joint."""

from __future__ import annotations

import sys
from dataclasses import dataclass

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ACTION_NAME = "/sim/hand_controller/follow_joint_trajectory"
JOINT_STATE_TOPIC = "/sim/joint_states"
JOINTS = [
    "thumb_cmc_roll",
    "thumb_cmc_yaw",
    "thumb_cmc_pitch",
    "index_mcp_pitch",
    "middle_mcp_pitch",
    "ring_mcp_pitch",
    "pinky_mcp_pitch",
]
OPEN = [0.001] * len(JOINTS)
ROLL_TARGETS = [0.020, 0.050, 0.100, 0.150, 0.200, 0.279]
STATUS_NAMES = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}


@dataclass
class GoalObservation:
    status: int
    result_code: int
    result_text: str
    actual: float


class ThumbRollDiagnostic(Node):
    def __init__(self) -> None:
        super().__init__("thumb_roll_range_diagnostic")
        self._latest_positions: dict[str, float] = {}
        self._action = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self._on_joint_state,
            qos_profile_sensor_data,
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_positions.update(zip(msg.name, msg.position, strict=False))

    def wait_until_ready(self) -> None:
        if not self._action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"action server unavailable: {ACTION_NAME}")

        deadline = self.get_clock().now().nanoseconds + int(10e9)
        while "thumb_cmc_roll" not in self._latest_positions:
            if self.get_clock().now().nanoseconds >= deadline:
                raise RuntimeError(f"joint state unavailable: {JOINT_STATE_TOPIC}")
            rclpy.spin_once(self, timeout_sec=0.1)

    def send(self, positions: list[float], duration_sec: int = 5) -> GoalObservation:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration_sec
        goal.trajectory.points = [point]

        send_future = self._action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("trajectory goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError("trajectory action returned no result")

        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

        result = wrapped_result.result
        return GoalObservation(
            status=wrapped_result.status,
            result_code=result.error_code,
            result_text=result.error_string,
            actual=self._latest_positions["thumb_cmc_roll"],
        )


def main() -> int:
    rclpy.init()
    node = ThumbRollDiagnostic()
    issue_seen = False

    try:
        node.wait_until_ready()
        initial = node.send(OPEN)
        print(
            "INITIAL_OPEN "
            f"status={STATUS_NAMES.get(initial.status, initial.status)} "
            f"actual={initial.actual:+.4f}"
        )

        for target in ROLL_TARGETS:
            positions = OPEN.copy()
            positions[0] = target
            observation = node.send(positions)
            error = target - observation.actual
            status_name = STATUS_NAMES.get(observation.status, str(observation.status))
            print(
                f"THUMB_ROLL target={target:+.4f} actual={observation.actual:+.4f} "
                f"error={error:+.4f} status={status_name} "
                f"result_code={observation.result_code} "
                f"result_text={observation.result_text!r}"
            )
            if observation.status != 4 or abs(error) > 0.08:
                issue_seen = True

            recovery = node.send(OPEN)
            if recovery.status != 4:
                print("RECOVERY=FAILED", file=sys.stderr)
                return 2

        print(
            "THUMB_ROLL_RANGE_DIAGNOSTIC="
            + ("ISSUE_CONFIRMED" if issue_seen else "PASS")
        )
        return 1 if issue_seen else 0
    except Exception as exc:  # noqa: BLE001 - diagnostic must print a clear cause
        print(f"DIAGNOSTIC_ERROR={exc}", file=sys.stderr)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
