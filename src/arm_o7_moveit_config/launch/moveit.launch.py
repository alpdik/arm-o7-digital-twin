"""Start MoveIt move_group and RViz against an already running controller backend."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _launch_setup(context):
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    use_sim_time = LaunchConfiguration("use_sim_time")
    hand_mount_xyz = LaunchConfiguration("hand_mount_xyz").perform(context)
    hand_mount_rpy = LaunchConfiguration("hand_mount_rpy").perform(context)

    description_file = str(
        Path(get_package_share_directory("arm_o7_description"))
        / "urdf"
        / "arm_o7.urdf.xacro"
    )
    moveit_config = (
        MoveItConfigsBuilder("arm_o7", package_name="arm_o7_moveit_config")
        .robot_description(
            file_path=description_file,
            mappings={
                "hardware_type": "none",
                "hand_mount_xyz": hand_mount_xyz,
                "hand_mount_rpy": hand_mount_rpy,
            },
        )
        .robot_description_semantic(file_path="config/arm_o7.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        name="moveit_rviz",
        output="screen",
        remappings=[
            ("/joint_states", f"/{namespace}/joint_states"),
            ("/monitored_planning_scene", f"/{namespace}/monitored_planning_scene"),
        ],
        arguments=[
            "-d",
            str(Path(get_package_share_directory("arm_o7_moveit_config")) / "config" / "moveit.rviz"),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )
    return [move_group, rviz]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="sim"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("hand_mount_xyz", default_value="0 0 0"),
            DeclareLaunchArgument("hand_mount_rpy", default_value="0 0 0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
