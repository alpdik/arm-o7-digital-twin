"""Terminal 2: publish, spawn, control, and optionally plan for the combined robot."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_name = LaunchConfiguration("world_name")
    entity_name = LaunchConfiguration("entity_name")
    start_moveit = LaunchConfiguration("start_moveit")
    start_twin = LaunchConfiguration("start_twin")
    hand_mount_xyz = LaunchConfiguration("hand_mount_xyz")
    hand_mount_rpy = LaunchConfiguration("hand_mount_rpy")

    description_file = PathJoinSubstitution(
        [FindPackageShare("arm_o7_description"), "urdf", "arm_o7.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("arm_o7_description"), "config", "sim_controllers.yaml"]
    )
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                description_file,
                " hardware_type:=sim",
                " controllers_file:=",
                controllers_file,
                " ros_namespace:=/sim",
                " hand_mount_xyz:=\'",
                hand_mount_xyz,
                "\' hand_mount_rpy:=\'",
                hand_mount_rpy,
                "\'",
            ]
        ),
        value_type=str,
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="sim",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_arm_o7",
        output="screen",
        arguments=[
            "-world",
            world_name,
            "-topic",
            "/sim/robot_description",
            "-name",
            entity_name,
            "-allow_renaming",
            "false",
        ],
    )

    manager = "/sim/controller_manager"
    common_spawner_args = [
        "--controller-manager",
        manager,
        "--controller-manager-timeout",
        "60",
    ]
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_joint_state_broadcaster",
        output="screen",
        arguments=["joint_state_broadcaster", *common_spawner_args],
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_arm_controller",
        output="screen",
        arguments=["arm_controller", *common_spawner_args],
    )
    hand_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_hand_controller",
        output="screen",
        arguments=["hand_controller", *common_spawner_args],
    )
    thumb_coupling_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_thumb_coupling_controller",
        output="screen",
        arguments=["thumb_coupling_controller", *common_spawner_args],
    )
    thumb_coupler = Node(
        package="arm_o7_bringup",
        executable="thumb_coupler.py",
        namespace="sim",
        name="thumb_coupler",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arm_o7_moveit_config"), "launch", "moveit.launch.py"]
            )
        ),
        condition=IfCondition(start_moveit),
        launch_arguments={
            "namespace": "sim",
            "use_sim_time": use_sim_time,
            "hand_mount_xyz": hand_mount_xyz,
            "hand_mount_rpy": hand_mount_rpy,
        }.items(),
    )

    twin_arbiter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_o7_twin"), "launch", "arm_o7_twin.launch.py"])
        ),
        condition=IfCondition(start_twin),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world_name", default_value="arm_lab"),
            DeclareLaunchArgument("entity_name", default_value="arm_o7"),
            DeclareLaunchArgument("start_moveit", default_value="true"),
            DeclareLaunchArgument("start_twin", default_value="true"),
            DeclareLaunchArgument(
                "hand_mount_xyz",
                default_value="0 0 0",
                description="Measured F-flange to O7 base translation in metres.",
            ),
            DeclareLaunchArgument(
                "hand_mount_rpy",
                default_value="0 0 0",
                description="Measured F-flange to O7 base roll/pitch/yaw in radians.",
            ),
            state_publisher,
            twin_arbiter,
            spawn_robot,
            RegisterEventHandler(
                OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_spawner])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=joint_state_spawner, on_exit=[arm_spawner])
            ),
            RegisterEventHandler(OnProcessExit(target_action=arm_spawner, on_exit=[hand_spawner])),
            RegisterEventHandler(
                OnProcessExit(target_action=hand_spawner, on_exit=[thumb_coupling_spawner])
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=thumb_coupling_spawner,
                    on_exit=[thumb_coupler, moveit],
                )
            ),
        ]
    )
