"""SITL coexistence bringup.

Assumes the user is already running PX4-SITL + Gazebo via:

    cd ~/wspaces/PX4-Autopilot
    make px4_sitl gz_sarax_plus

That command loads the SDF with the gz_ros2_control-system plugin, but the
plugin needs the URDF before it can configure -- so this launch starts
robot_state_publisher and the controller spawners on the ROS 2 side.

Nothing here talks to PX4; the two stacks meet only through Gazebo physics.

Usage:
    ros2 launch sarax_bringup sitl.launch.py
    ros2 launch sarax_bringup sitl.launch.py controller:=joint_impedance_controller
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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
        description="Which controller to spawn besides joint_state_broadcaster.",
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

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
        output="screen",
    )
    ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller,
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
        output="screen",
    )
    after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[ctrl_spawner],
        )
    )

    return LaunchDescription([controller_arg, rsp, jsb_spawner, after_jsb])
