#!/usr/bin/env bash
# Launch the ROS 2 side of the SITL session:
#   - robot_state_publisher  (provides /robot_description for gz_ros2_control)
#   - joint_state_broadcaster
#   - the chosen manipulator controller (default: joint_impedance_controller)
#
# Run AFTER "Gazebo world is ready" appears in the px4 terminal.
#
# Invoke via:  pixi run ros
# Override the controller:  SARAX_CONTROLLER=forward_effort_controller pixi run ros
# Or pass as a launch arg:  pixi run ros -- controller:=forward_effort_controller
#
# The pixi environment already has ROS 2 activated.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source "$REPO/install/setup.bash"
set -u

exec ros2 launch sarax_bringup sitl.launch.py \
    controller:="${SARAX_CONTROLLER:-joint_impedance_controller}" \
    "$@"
