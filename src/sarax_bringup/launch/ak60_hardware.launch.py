"""Connect the SARAX robot and existing impedance controller to physical AK60 motors."""

import atexit
import math
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml


def launch_hardware(context):
    names = (
        "can_interface", "hardware_verified", "use_second_motor", "motor_1_id", "motor_2_id",
        "motor_1_effort_limit", "motor_2_effort_limit", "feedback_timeout", "startup_timeout",
        "impedance_law", "max_position_error",
        "motor_1_stiffness", "motor_1_damping", "motor_2_stiffness", "motor_2_damping",
    )
    args = {name: LaunchConfiguration(name).perform(context) for name in names}
    for name in ("hardware_verified", "use_second_motor"):
        if args[name] not in ("true", "false"):
            raise ValueError(name + " must be true or false")
    max_position_error = float(args["max_position_error"])
    if not math.isfinite(max_position_error) or max_position_error <= 0.0:
        raise ValueError("max_position_error must be finite and greater than zero")
    joints = ["mani_joint_1", "mani_joint_2"] if args["use_second_motor"] == "true" else ["mani_joint_1"]
    mappings = {
        "hardware": "real",
        "ak60_can_interface": args["can_interface"],
        "ak60_hardware_verified": args["hardware_verified"],
        "ak60_use_second_motor": args["use_second_motor"],
        "ak60_feedback_timeout": args["feedback_timeout"],
        "ak60_startup_timeout": args["startup_timeout"],
        **{f"mani_joint_{i}_motor_id": args[f"motor_{i}_id"] for i in (1, 2)},
        **{f"mani_joint_{i}_effort_limit": args[f"motor_{i}_effort_limit"] for i in (1, 2)},
    }
    description = Path(get_package_share_directory("sarax_description")) / "urdf/sarax_plus.urdf.xacro"
    robot_description = xacro.process_file(str(description), mappings=mappings).toxml()

    config_path = Path(get_package_share_directory("sarax_bringup")) / "config/controllers.yaml"
    config = yaml.safe_load(config_path.read_text())
    manager = config["controller_manager"]["ros__parameters"]
    manager.update(robot_description=robot_description, use_sim_time=False,
                   hardware_components_initial_state={"unconfigured": ["SaraxManipulator"]})
    manager.pop("forward_effort_controller", None)
    config.pop("forward_effort_controller", None)
    config["joint_state_broadcaster"] = {"ros__parameters": {
        "joints": joints, "interfaces": ["position", "velocity", "effort"],
    }}
    controller = config["joint_impedance_controller"]["ros__parameters"]
    for name in list(controller):
        if name.startswith(("stiffness_", "damping_")):
            del controller[name]
    controller.update(joints=list(joints), robot_description=robot_description,
                      impedance_law=args["impedance_law"],
                      max_position_error=float(args["max_position_error"]),
                      effort_limits=[float(args[f"motor_{i}_effort_limit"])
                                     for i in range(1, len(joints) + 1)])
    for i in range(len(joints)):
        controller[f"stiffness_{i}"] = float(args[f"motor_{i + 1}_stiffness"])
        controller[f"damping_{i}"] = float(args[f"motor_{i + 1}_damping"])
    # Reuse the system controller configuration with physical-joint overrides.
    with tempfile.NamedTemporaryFile(mode="w", prefix="launch_params_sarax_", suffix=".yaml",
                                     delete=False) as stream:
        yaml.safe_dump(config, stream)
        parameters = stream.name
    atexit.register(Path(parameters).unlink, missing_ok=True)

    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": ParameterValue(robot_description, value_type=str),
                          "use_sim_time": False}]),
        Node(package="controller_manager", executable="ros2_control_node", output="screen",
             parameters=[parameters]),
        # Activation follows fresh real feedback; loading does not enable motors.
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=["joint_state_broadcaster", "joint_impedance_controller", "--inactive",
                        "-c", "/controller_manager", "--controller-manager-timeout", "20"]),
    ]


def generate_launch_description():
    defaults = {
        "can_interface": "can0", "hardware_verified": "false", "use_second_motor": "false",
        "motor_1_id": "2", "motor_2_id": "1",
        "motor_1_effort_limit": "5.0", "motor_2_effort_limit": "5.0",
        "feedback_timeout": "0.1", "startup_timeout": "1.0",
        "max_position_error": "0.2",
        "impedance_law": "sarax_impedance_controller::PDLaw",
        "motor_1_stiffness": "10.0", "motor_1_damping": "1.0",
        "motor_2_stiffness": "10.0", "motor_2_damping": "1.0",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=launch_hardware),
    ])
