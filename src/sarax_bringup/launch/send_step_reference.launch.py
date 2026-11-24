"""Manual step-response helper: stream one JointTrajectoryPoint to the impedance
controller's reference topic until Ctrl-C.

The controller's `reference_timeout` watchdog drops a reference that stops
refreshing, so the step is streamed (default 20 Hz), not sent once. After Ctrl-C
the arm holds the pose it is in. For a reference that never expires, set the
q_ref_N parameters instead (ros2 param set /joint_impedance_controller q_ref_0 0.3).

Usage (controller must already be loaded+activated):
    ros2 launch sarax_bringup send_step_reference.launch.py q1:=0.3 q2:=-0.4
    ros2 launch sarax_bringup send_step_reference.launch.py q1:=0.3 q2:=-0.4 rate:=50
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    q1_arg = DeclareLaunchArgument("q1", default_value="0.3")
    q2_arg = DeclareLaunchArgument("q2", default_value="-0.4")
    rate_arg = DeclareLaunchArgument(
        "rate", default_value="20",
        description="Publish rate (Hz); must be faster than 1/reference_timeout.")

    pub = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "-r", LaunchConfiguration("rate"),
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
    return LaunchDescription([q1_arg, q2_arg, rate_arg, pub])
