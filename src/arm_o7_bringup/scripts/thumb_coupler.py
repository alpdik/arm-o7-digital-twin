#!/usr/bin/env python3
"""Drive the two O7 thumb followers that DART cannot mimic natively."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


SOURCE_JOINT = "thumb_cmc_pitch"
FOLLOWERS = ((1.3898, 0.7152), (1.5085, 0.7763))


class ThumbCoupler(Node):
    def __init__(self) -> None:
        super().__init__("thumb_coupler")
        self._publisher = self.create_publisher(
            Float64MultiArray, "thumb_coupling_controller/commands", 10
        )
        self.create_subscription(
            JointState,
            "joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self._last_command = None
        self.get_logger().info(
            "O7 thumb coupling active: thumb_cmc_pitch -> thumb_mcp, thumb_ip"
        )

    def _on_joint_state(self, message: JointState) -> None:
        try:
            index = message.name.index(SOURCE_JOINT)
            source_position = message.position[index]
        except (ValueError, IndexError):
            return

        if not math.isfinite(source_position):
            return

        command = tuple(
            min(max(source_position * multiplier, 0.0), upper)
            for multiplier, upper in FOLLOWERS
        )
        if command == self._last_command:
            return

        output = Float64MultiArray()
        output.data = list(command)
        self._publisher.publish(output)
        self._last_command = command


def main() -> None:
    rclpy.init()
    node = ThumbCoupler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
