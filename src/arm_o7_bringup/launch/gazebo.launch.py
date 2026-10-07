"""Terminal 1: start the Gazebo Harmonic world and its one-way clock bridge."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    verbosity = LaunchConfiguration("verbosity")
    description_share_parent = str(
        Path(get_package_share_directory("arm_o7_description")).parent
    )
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gazebo_resource_path = os.pathsep.join(
        value
        for value in (description_share_parent, existing_resource_path)
        if value
    )
    world_file = PathJoinSubstitution(
        [FindPackageShare("arm_o7_bringup"), "worlds", "arm_lab.sdf"]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": ["-r -v ", verbosity, " ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # '[' means Gazebo Transport -> ROS only. Never bridge /clock in both directions.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("verbosity", default_value="2"),
            # sdformat converts package://arm_o7_description/... into
            # model://arm_o7_description/.... Gazebo therefore needs the parent
            # of the installed package share directory on its resource path.
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH", value=gazebo_resource_path
            ),
            gazebo,
            clock_bridge,
        ]
    )
