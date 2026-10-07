"""Developer convenience launch; the documented two-terminal flow remains preferred."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    launch_dir = PathJoinSubstitution([FindPackageShare("arm_o7_bringup"), "launch"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, "gazebo.launch.py"]))
    )
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, "sim.launch.py"]))
    )
    return LaunchDescription([gazebo, TimerAction(period=3.0, actions=[robot])])
