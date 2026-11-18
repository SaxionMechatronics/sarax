"""Publish robot frames and an optional stationary world for camera work."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare("sarax_description"), "urdf", "sarax_plus.urdf.xacro",
    ])
    description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file, " hardware:=mock"]),
        value_type=str,
    )
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("publish_robot_description", default_value="true"),
        DeclareLaunchArgument("publish_test_world", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description, "use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("publish_robot_description")),
            output="screen",
        ),
        # This world has the base's axes; it is not a PX4 NED reference.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="stationary_world_to_base",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "world", "--child-frame-id", "base_link",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("publish_test_world")),
            output="screen",
        ),
    ])
