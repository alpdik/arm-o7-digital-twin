"""Start the guarded O7 SDK adapter and the physical joint-state mux."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = str(
        Path(get_package_share_directory("arm_o7_linkerhand_adapter"))
        / "config"
        / "o7_right.yaml"
    )
    config = LaunchConfiguration("config")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="arm_o7_linkerhand_adapter",
                executable="o7_adapter",
                name="o7_linkerhand_adapter",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="arm_o7_linkerhand_adapter",
                executable="joint_state_mux",
                name="arm_o7_real_joint_state_mux",
                output="screen",
                parameters=[config],
            ),
        ]
    )
