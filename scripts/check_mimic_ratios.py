#!/usr/bin/env python3
"""Verify O7 mimic-joint state ratios from the running simulation."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


MIMIC_JOINTS = {
    "thumb_mcp": ("thumb_cmc_pitch", 1.3898),
    "thumb_ip": ("thumb_cmc_pitch", 1.5085),
    "index_pip": ("index_mcp_pitch", 1.3462),
    "index_dip": ("index_mcp_pitch", 0.4615),
    "middle_pip": ("middle_mcp_pitch", 1.3462),
    "middle_dip": ("middle_mcp_pitch", 0.4615),
    "ring_pip": ("ring_mcp_pitch", 1.3462),
    "ring_dip": ("ring_mcp_pitch", 0.4615),
    "pinky_pip": ("pinky_mcp_pitch", 1.3462),
    "pinky_dip": ("pinky_mcp_pitch", 0.4615),
}


class JointStateProbe(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("arm_o7_mimic_ratio_check")
        self.message = None
        self.create_subscription(
            JointState, topic, self._on_joint_state, qos_profile_sensor_data
        )

    def _on_joint_state(self, message: JointState) -> None:
        self.message = message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/sim/joint_states")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--tolerance", type=float, default=0.03)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = JointStateProbe(args.topic)
    deadline = time.monotonic() + args.timeout

    try:
        while node.message is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.message is None:
            print(f"MIMIC_CHECK=FAIL: no message received from {args.topic}")
            return 1

        positions = dict(zip(node.message.name, node.message.position))
        required = set(MIMIC_JOINTS)
        required.update(source for source, _ in MIMIC_JOINTS.values())
        missing = sorted(required.difference(positions))
        if missing:
            print("MIMIC_CHECK=FAIL: missing joints: " + ", ".join(missing))
            return 1

        sources = {source for source, _ in MIMIC_JOINTS.values()}
        if max(abs(positions[name]) for name in sources) < 0.1:
            print("MIMIC_CHECK=INCONCLUSIVE: hand is near zero; run the motion demo first")
            return 2

        failed = False
        max_error = 0.0
        for target, (source, multiplier) in MIMIC_JOINTS.items():
            expected = positions[source] * multiplier
            actual = positions[target]
            error = abs(actual - expected)
            max_error = max(max_error, error)
            status = "PASS" if error <= args.tolerance else "FAIL"
            failed |= status == "FAIL"
            print(
                f"{status:4} {target:11} <- {source:16} "
                f"actual={actual:+.4f} expected={expected:+.4f} error={error:.4f}"
            )

        if failed:
            print(
                f"MIMIC_CHECK=FAIL max_error={max_error:.4f} "
                f"tolerance={args.tolerance:.4f}"
            )
            return 1

        print(
            f"MIMIC_CHECK=PASS max_error={max_error:.4f} "
            f"tolerance={args.tolerance:.4f}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
