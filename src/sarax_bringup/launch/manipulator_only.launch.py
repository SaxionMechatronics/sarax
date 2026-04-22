"""Manipulator-only bringup.

Starts Gazebo with a minimal world, spawns sarax_plus, starts robot_state_publisher
(so gz_ros2_control inside the Gazebo plugin can read the URDF), and spawns the
joint_state_broadcaster + an effort controller of your choice.

Usage:
    ros2 launch sarax_bringup manipulator_only.launch.py
    ros2 launch sarax_bringup manipulator_only.launch.py controller:=joint_impedance_controller

Default is forward_effort_controller so you can sanity-check the pipeline with a
Float64MultiArray publish before loading the impedance law.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="forward_effort_controller",
        description="Which controller to spawn besides joint_state_broadcaster. "
                    "Use 'joint_impedance_controller' to load the impedance controller.",
    )
    controller = LaunchConfiguration("controller")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("sarax_description"), "urdf", "sarax_plus.urdf.xacro"]
    )
    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", xacro_file, " hardware:=gazebo"]
    )
    robot_description = {"robot_description": robot_description_content}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    world_file = PathJoinSubstitution(
        [FindPackageShare("sarax_gz_sim"), "worlds", "manipulator_only.sdf"]
    )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": ["-r ", world_file]}.items(),
    )

    # Spawn the robot from the topic that robot_state_publisher publishes.
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "sarax_plus",
            "-topic", "robot_description",
            "-z", "0.3",
        ],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller, "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawn jsb after gz is up and the model is alive; spawn the controller after jsb.
    after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn,
            on_exit=[jsb_spawner],
        )
    )
    after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[ctrl_spawner],
        )
    )

    return LaunchDescription([
        controller_arg,
        rsp,
        gz_sim,
        spawn,
        after_spawn,
        after_jsb,
    ])
