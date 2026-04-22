"""Visualise the SARAX+ URDF in RViz with a jointstate slider GUI.

Usage:
    ros2 launch sarax_description view_robot.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_arg = DeclareLaunchArgument("hardware", default_value="mock")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("sarax_description"), "urdf", "sarax_plus.urdf.xacro"]
    )
    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", xacro_file, " hardware:=", LaunchConfiguration("hardware")]
    )
    robot_description = {"robot_description": robot_description_content}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    jsp_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("sarax_description"), "rviz", "view_robot.rviz"]
    )
    rviz = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", rviz_config],
        output="log",
    )
    return LaunchDescription([hardware_arg, rsp, jsp_gui, rviz])
