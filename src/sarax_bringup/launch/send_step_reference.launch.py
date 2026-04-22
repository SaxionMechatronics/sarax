"""One-shot helper: publish a single JointTrajectoryPoint to the impedance
controller's reference topic. Use for quick manual step-response checks.

Usage (controller must already be loaded+activated):
    ros2 launch sarax_bringup send_step_reference.launch.py q1:=0.3 q2:=-0.4
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    q1_arg = DeclareLaunchArgument("q1", default_value="0.3")
    q2_arg = DeclareLaunchArgument("q2", default_value="-0.4")

    pub = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "-1",
            "/joint_impedance_controller/reference",
            "trajectory_msgs/msg/JointTrajectoryPoint",
            [
                "{positions: [", LaunchConfiguration("q1"),
                ", ", LaunchConfiguration("q2"),
                "], velocities: [0.0, 0.0]}"
            ],
        ],
        output="screen",
    )
    return LaunchDescription([q1_arg, q2_arg, pub])
