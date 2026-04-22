#!/usr/bin/env bash
# Launch PX4 SITL with the sarax_plus model in Gazebo Harmonic.
#
# Builds PX4 on the first run (takes a few minutes); subsequent launches
# reuse the cached build.
#
# After "Gazebo world is ready" appears, open a second terminal and run:
#   pixi run ros
#
# Invoke via:  pixi run px4
# The pixi environment already has ROS 2 and Gazebo activated.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="$REPO/PX4-Autopilot"

if [ ! -d "$PX4_DIR" ]; then
    echo "ERROR: PX4-Autopilot not found at $PX4_DIR"
    echo "       Run 'pixi run install' first."
    exit 1
fi

# Isolate from the system ROS installation — same issue as in build.sh:
# /opt/ros/humble headers and libraries conflict with the conda GCC 14 ABI.
# cmake derives search prefixes from PATH, so strip /opt/ros first.
_strip_ros() { echo "${1:-}" | tr ':' '\n' | grep -v '^/opt/ros' | paste -sd: -; }
export PATH="$(_strip_ros "$PATH")"
export CMAKE_PREFIX_PATH="${CONDA_PREFIX}:$(_strip_ros "${CMAKE_PREFIX_PATH:-}")"
export PKG_CONFIG_PATH="$(_strip_ros "${PKG_CONFIG_PATH:-}")"
export LD_LIBRARY_PATH="$(_strip_ros "${LD_LIBRARY_PATH:-}")"

# If the cmake cache has the wrong protobuf include dir (system /usr/include
# instead of the conda env), delete it so cmake re-detects with the correct
# CMAKE_PREFIX_PATH set above.
_px4_cache="$PX4_DIR/build/px4_sitl_default/CMakeCache.txt"
if grep -q 'Protobuf_INCLUDE_DIR:PATH=/usr/include' "$_px4_cache" 2>/dev/null; then
    echo ">>> Clearing stale PX4 cmake cache (wrong protobuf include dir)..."
    rm -rf "$PX4_DIR/build/px4_sitl_default"
fi

# Source the workspace overlay so the sarax model SDF and the pixi-built
# gz_ros2_control plugin are on the ament index before Gazebo starts.
# colcon's setup.bash references COLCON_TRACE without a default, which trips
# our -u flag; disable it around the source call.
set +u
source "$REPO/install/setup.bash"
set -u

cd "$PX4_DIR"
exec make px4_sitl gz_sarax_plus
