#!/usr/bin/env bash
# RealSense depth cloud and the existing ROI filter.
# Run: pixi run camera_hardware min_z:=0.50 max_range:=5.0
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

# Defaults for pixi run camera_hardware. Command-line name:=value arguments
# override the environment variables, which override the fallback values here.
POINTS_TOPIC="${SARAX_CAMERA_POINTS_TOPIC:-/camera/points}"
FILTERED_TOPIC="${SARAX_FILTERED_POINTS_TOPIC:-/camera/points_filtered}"
CLOUD_MIN_X="${SARAX_CLOUD_MIN_X:--1.35}"
CLOUD_MAX_X="${SARAX_CLOUD_MAX_X:-1.35}"
CLOUD_MAX_ABS_Y="${SARAX_CLOUD_MAX_ABS_Y:-1.35}"
CLOUD_MIN_Z="${SARAX_CLOUD_MIN_Z:-0.30}"
CLOUD_MAX_Z="${SARAX_CLOUD_MAX_Z:-4.20}"
CLOUD_MAX_RANGE="${SARAX_CLOUD_MAX_RANGE:-6.0}"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
MIDDLEWARE="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
FILTER_BIN="$REPO/install/sarax_perception/lib/sarax_perception/cloud_filter_node"

if [[ "${1:-}" == -- ]]; then
    shift
fi
for arg in "$@"; do
    if [[ "$arg" != *:=?* ]]; then
        echo "Expected name:=value, for example min_z:=0.50. Received: $arg" >&2
        exit 1
    fi
    value="${arg#*:=}"
    case "${arg%%:=*}" in
        points_topic) POINTS_TOPIC="$value" ;;
        filtered_points_topic) FILTERED_TOPIC="$value" ;;
        min_x) CLOUD_MIN_X="$value" ;;
        max_x) CLOUD_MAX_X="$value" ;;
        max_abs_y) CLOUD_MAX_ABS_Y="$value" ;;
        min_z) CLOUD_MIN_Z="$value" ;;
        max_z) CLOUD_MAX_Z="$value" ;;
        max_range) CLOUD_MAX_RANGE="$value" ;;
        ros_domain_id) DOMAIN_ID="$value" ;;
        rmw_implementation) MIDDLEWARE="$value" ;;
        ros_localhost_only) LOCALHOST_ONLY="$value" ;;
        *)
            echo "Unknown camera argument: $arg. See the camera_hardware arguments in README.md." >&2
            exit 1
            ;;
    esac
done

if [[ "$POINTS_TOPIC" != /* || "$FILTERED_TOPIC" != /* ]]; then
    echo "Use absolute topic names starting with /." >&2
    exit 1
fi
if [[ "$POINTS_TOPIC" == "$FILTERED_TOPIC" ]]; then
    echo "Camera and filtered topics must differ." >&2
    exit 1
fi
if [[ ! -x "$FILTER_BIN" ]]; then
    echo "Build the filter first: pixi run build --packages-select sarax_perception" >&2
    exit 1
fi

set +u
# shellcheck source=/dev/null
source "$REPO/install/setup.bash"
set -u
export ROS_DOMAIN_ID="$DOMAIN_ID"
export RMW_IMPLEMENTATION="$MIDDLEWARE"
export ROS_LOCALHOST_ONLY="$LOCALHOST_ONLY"

pids=()
cleanup() {
    for pid in "${pids[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    for pid in "${pids[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "RealSense -> $POINTS_TOPIC -> ROI filter -> $FILTERED_TOPIC"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

# Keep the camera process free of the default environment's Python and libraries.
setsid pixi run --clean-env -e camera /usr/bin/env \
    ROS_DOMAIN_ID="$ROS_DOMAIN_ID" RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
    ROS_LOCALHOST_ONLY="$ROS_LOCALHOST_ONLY" \
    ros2 run realsense2_camera realsense2_camera_node \
    --ros-args \
    -r "/camera/camera/depth/color/points:=$POINTS_TOPIC" \
    -p use_sim_time:=false \
    -p enable_depth:=true \
    -p enable_color:=false \
    -p pointcloud.enable:=true \
    -p pointcloud.stream_filter:=0 \
    -p pointcloud.pointcloud_qos:=SYSTEM_DEFAULT &
pids+=("$!")

# Optical coordinates: x right, y down, z forward; keep the original frame ID.
setsid env LD_LIBRARY_PATH="$REPO/.pixi/envs/default/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    "$FILTER_BIN" --ros-args \
    -p use_sim_time:=false \
    -p "input_topic:=$POINTS_TOPIC" \
    -p "output_topic:=$FILTERED_TOPIC" \
    -p max_range:="$CLOUD_MAX_RANGE" \
    -p min_x:="$CLOUD_MIN_X" \
    -p max_x:="$CLOUD_MAX_X" \
    -p max_abs_y:="$CLOUD_MAX_ABS_Y" \
    -p min_z:="$CLOUD_MIN_Z" \
    -p max_z:="$CLOUD_MAX_Z" &
pids+=("$!")

# Stop both owned process groups if either process exits or Ctrl+C is pressed.
wait -n "${pids[@]}"
