"""Glove teleop: 5DT glove reader, ON/OFF panel and (optionally) the real Ti5 hand bridge.

Start terminal 1 (Gazebo) and terminal 2 (robot + arbiter) first.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution(
        [FindPackageShare("arm_o7_glove"), "config", "glove_teleop.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "real_hand",
                default_value="true",
                description="Start the Ti5 bridge (it still waits for the panel's checkbox)",
            ),
            Node(
                package="arm_o7_glove",
                executable="glove_5dt",
                name="glove_5dt",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="arm_o7_glove",
                executable="teleop_panel",
                name="glove_teleop",
                output="screen",
            ),
            Node(
                package="arm_o7_glove",
                executable="ti5_bridge",
                name="ti5_bridge",
                parameters=[config],
                output="screen",
                condition=IfCondition(LaunchConfiguration("real_hand")),
            ),
        ]
    )
