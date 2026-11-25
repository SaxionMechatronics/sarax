"""state_bridge: PX4 /fmu/out/vehicle_odometry -> /sarax/odom + TF world -> base_link.

use_sim_time defaults to the active platform profile (SARAX_TARGET: sitl true, hw false);
the launch argument wins.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sarax_bringup import profile as sarax_profile


def generate_launch_description():
    default_sim = "true" if sarax_profile.get(sarax_profile.load(), "use_sim_time") else "false"
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value=default_sim),
        Node(
            package="sarax_px4_bridge",
            executable="state_bridge",
            name="state_bridge",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            output="screen",
        ),
    ])
