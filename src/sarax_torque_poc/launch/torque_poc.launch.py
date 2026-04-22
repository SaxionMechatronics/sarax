from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_name_arg = DeclareLaunchArgument(
        "model_name",
        default_value="sarax_plus_0",
        description="Gazebo model name whose manipulator joints we command.",
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="default",
        description="Gazebo world name (PX4 gz_sarax_plus defaults to 'default').",
    )
    model_name = LaunchConfiguration("model_name")
    world_name = LaunchConfiguration("world_name")

    # ROS <-> Gazebo bridge.
    #   ]  == ROS -> Gazebo (cmd_force: we send torques in)
    #   [  == Gazebo -> ROS (joint_state: we read joint feedback out)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="sarax_torque_bridge",
        arguments=[
            ["/model/", model_name, "/joint/mani_joint_1/cmd_force@std_msgs/msg/Float64]gz.msgs.Double"],
            ["/model/", model_name, "/joint/mani_joint_2/cmd_force@std_msgs/msg/Float64]gz.msgs.Double"],
            ["/world/", world_name, "/model/", model_name, "/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"],
        ],
        # Remap the long gz topic to a conventional /joint_states name on the ROS side.
        remappings=[
            (["/world/", world_name, "/model/", model_name, "/joint_state"], "/joint_states"),
        ],
        output="screen",
    )

    # Numeric params (efforts, sinusoid) use the node's built-in defaults.
    # Override at runtime with:
    #   ros2 launch sarax_torque_poc torque_poc.launch.py
    #   ros2 param set /sarax_torque_publisher efforts "[2.0, -1.0]"
    publisher = Node(
        package="sarax_torque_poc",
        executable="torque_publisher",
        name="sarax_torque_publisher",
        parameters=[{"model_name": model_name}],
        output="screen",
    )

    return LaunchDescription([model_name_arg, world_name_arg, bridge, publisher])
