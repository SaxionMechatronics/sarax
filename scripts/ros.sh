#!/usr/bin/env bash
# Launch the ROS 2 side of the SITL session:
#   - robot_state_publisher  (provides /robot_description for gz_ros2_control)
#   - joint_state_broadcaster
#   - the chosen manipulator controller (default: joint_impedance_controller)
#   - camera point-cloud filter
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
FILTER_BIN="$REPO/install/sarax_perception/lib/sarax_perception/cloud_filter_node"
CLOUD_MAX_RANGE="${SARAX_CLOUD_MAX_RANGE:-6.0}"
CLOUD_MIN_X="${SARAX_CLOUD_MIN_X:-0.30}"
CLOUD_MAX_X="${SARAX_CLOUD_MAX_X:-4.20}"
CLOUD_MAX_ABS_Y="${SARAX_CLOUD_MAX_ABS_Y:-1.35}"
CLOUD_MIN_Z="${SARAX_CLOUD_MIN_Z:--0.25}"
CLOUD_MAX_Z="${SARAX_CLOUD_MAX_Z:-1.35}"

LD_LIBRARY_PATH="$PIXI_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    "$BRIDGE_BIN" \
    "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
    "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \
    "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image" \
    "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
    --ros-args --remap __node:=camera_bridge &
BRIDGE_PID=$!

LD_LIBRARY_PATH="$PIXI_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    "$FILTER_BIN" \
    --ros-args \
    -p input_topic:=/camera/points \
    -p output_topic:=/camera/points_filtered \
    -p max_range:="$CLOUD_MAX_RANGE" \
    -p min_x:="$CLOUD_MIN_X" \
    -p max_x:="$CLOUD_MAX_X" \
    -p max_abs_y:="$CLOUD_MAX_ABS_Y" \
    -p min_z:="$CLOUD_MIN_Z" \
    -p max_z:="$CLOUD_MAX_Z" &
FILTER_PID=$!

cleanup() {
    kill "$BRIDGE_PID" "$FILTER_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" "$FILTER_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 launch sarax_bringup sitl.launch.py \
    controller:="${SARAX_CONTROLLER:-joint_impedance_controller}" \
    "$@"
