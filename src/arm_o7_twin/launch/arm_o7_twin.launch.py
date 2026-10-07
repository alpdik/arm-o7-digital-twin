"""Launch the ARM1.5/O7 digital-twin command arbiter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("arm_o7_twin"), "config", "arm_o7_twin.yaml"
    )
    config = LaunchConfiguration("config")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Absolute path to the arbiter parameter YAML",
            ),
            Node(
                package="arm_o7_twin",
                executable="arbiter",
                name="arm_o7_twin_arbiter",
                output="screen",
                parameters=[config],
                emulate_tty=True,
            ),
        ]
    )
