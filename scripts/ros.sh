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

PIXI_LIB="$REPO/.pixi/envs/default/lib"
BRIDGE_BIN="$REPO/install/ros_gz_bridge/lib/ros_gz_bridge/parameter_bridge"

LD_LIBRARY_PATH="$PIXI_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    "$BRIDGE_BIN" \
    "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
    "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \
    "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image" \
    "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
    --ros-args --remap __node:=camera_bridge &
BRIDGE_PID=$!

cleanup() {
    kill "$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 launch sarax_bringup sitl.launch.py \
    controller:="${SARAX_CONTROLLER:-joint_impedance_controller}" \
    "$@"
