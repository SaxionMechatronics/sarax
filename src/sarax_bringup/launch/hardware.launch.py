"""One hardware bringup: arm, XRCE agent, state_bridge and the flight recording.

    pixi run hardware                       # real AK60 motors, CAN set up first
    pixi run hardware hardware:=mock        # mock arm (no CAN adapter, controllers active)
    pixi run hardware extra_topics:='/inspection_target /plan_status'

What it starts:
  - the arm through ``SaraxArmSystem`` (belt transmission) with the AK60 driver, or with
    ``mock_components/GenericSystem`` for ``hardware:=mock`` (``ak60_hardware.launch.py``);
  - the Micro XRCE-DDS Agent (``scripts/agent.sh``), PX4's link to ROS 2;
  - ``state_bridge`` (/fmu/out/vehicle_odometry -> /sarax/odom + TF);
  - ``ros2 bag record`` of the platform topics plus ``extra_topics``: one bag per flight.
    The PX4 ULog stays on the FMU.

CAN setup needs ``sudo`` and a terminal, so it runs in ``scripts/hardware.sh`` (the pixi task)
before this file is launched; the mock arm skips it. Everything else, for example
``motor_1_stiffness:=8.0``, goes to ``ak60_hardware.launch.py`` unchanged. Platform values
come from the profile of ``SARAX_TARGET`` (the pixi task defaults it to ``hw``).
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import shlex

from sarax_bringup import profile as sarax_profile
from sarax_bringup import recording


def _flag(context, name):
    value = LaunchConfiguration(name).perform(context)
    if value not in ("true", "false"):
        raise ValueError(f"{name} must be true or false")
    return value == "true"


def _agent_script(context):
    explicit = LaunchConfiguration("agent_script").perform(context)
    candidates = [explicit] if explicit else [
        os.environ.get("SARAX_AGENT_SCRIPT", ""),
        # A symlink install keeps this file in the source tree: <repo>/src/sarax_bringup/launch.
        str(Path(__file__).resolve().parents[3] / "scripts" / "agent.sh"),
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return candidate
    raise ValueError("agent script not found: pass agent_script:=<path to scripts/agent.sh> "
                     "or set SARAX_AGENT_SCRIPT, or agent:=false to run the agent yourself")


def launch_platform(context):
    actions = []
    prof = sarax_profile.load(default="hw")
    use_sim_time = "true" if sarax_profile.get(prof, "use_sim_time") else "false"

    if _flag(context, "agent"):
        agent = ["bash", _agent_script(context),
                 *shlex.split(LaunchConfiguration("agent_args").perform(context))]
        actions.append(ExecuteProcess(cmd=agent, name="xrce_agent", output="screen"))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("sarax_px4_bridge"), "launch", "state_bridge.launch.py"])),
        launch_arguments={"use_sim_time": use_sim_time}.items()))

    if _flag(context, "record"):
        extra = recording.parse_extra_topics(LaunchConfiguration("extra_topics").perform(context))
        bag = LaunchConfiguration("bag_dir").perform(context)
        output = Path(bag) if bag else recording.default_bag_dir()
        if output.exists():
            raise ValueError(f"bag directory {output} already exists; rosbag2 does not overwrite")
        command = recording.record_command(output, extra)
        actions += [LogInfo(msg="Recording: " + recording.describe(command)),
                    ExecuteProcess(cmd=command, name="bag_record", output="screen")]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware", default_value="real",
                              description="real (AK60 motors over CAN) or mock (no motors)"),
        DeclareLaunchArgument("can_interface", default_value="",
                              description="CAN adapter of the motors (set by scripts/hardware.sh)"),
        DeclareLaunchArgument("hardware_verified", default_value="false",
                              description="operator acknowledgement for the real motors"),
        DeclareLaunchArgument("agent", default_value="true",
                              description="start the Micro XRCE-DDS Agent"),
        DeclareLaunchArgument("agent_args", default_value="",
                              description="MicroXRCEAgent arguments; empty = SITL udp4 -p 8888, "
                                          "vehicle e.g. 'serial --dev /dev/ttyUSB0 -b 921600'"),
        DeclareLaunchArgument("agent_script", default_value="",
                              description="path of scripts/agent.sh (default: $SARAX_AGENT_SCRIPT)"),
        DeclareLaunchArgument("record", default_value="true",
                              description="record the flight with ros2 bag"),
        DeclareLaunchArgument("extra_topics", default_value="",
                              description="topics recorded besides the platform list "
                                          "(whitespace or comma separated, absolute names)"),
        DeclareLaunchArgument("bag_dir", default_value="",
                              description="bag directory (must not exist); default "
                                          "$SARAX_BAG_DIR or ./bags, plus /flight_<timestamp>"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("sarax_bringup"), "launch", "ak60_hardware.launch.py"])),
            launch_arguments={"hardware": LaunchConfiguration("hardware"),
                              "can_interface": LaunchConfiguration("can_interface"),
                              "hardware_verified": LaunchConfiguration("hardware_verified"),
                              }.items()),
        OpaqueFunction(function=launch_platform),
    ])
