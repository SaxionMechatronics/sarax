from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Preserve the original launch defaults; arguments override ROS parameters.
    parameters = {
        "use_sim_time": ("true", bool),
        "k_neighbors": ("500", int),
        "input_topic": ("/camera/points_filtered", str),
        "output_topic": ("/pca_normals", str),
        "arrow_scale": ("0.3", float),
        "arrow_step": ("2", int),
        "voxel_size": ("0.01", float),
        "max_range": ("5.0", float),
        "csv_output_path": ("", str),
    }

    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=default)
            for name, (default, _) in parameters.items()
        ],
        Node(
            package="sarax_perception",
            executable="pca_normal_node",
            name="pca_normal_estimation",
            parameters=[{
                name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
                for name, (_, value_type) in parameters.items()
            }],
            output="screen",
        ),
    ])
