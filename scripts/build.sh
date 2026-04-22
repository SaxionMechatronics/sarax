#!/usr/bin/env bash
# Build the ROS 2 workspace with colcon.
# Skips the Ignition/Fortress and demo packages not needed at runtime.
#
# Invoke via:  pixi run build
# The pixi environment already has ROS 2 activated — no manual sourcing needed.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$REPO"

# Strip /opt/ros paths so the system ROS installation does not leak into the
# conda-based build.  robostack-humble (GCC 14 ABI) and system apt packages
# (GCC 11 ABI) cannot be mixed — the linker will hit CXXABI_1.3.15 undefined
# references if both are in scope simultaneously.
_strip_ros() {
  echo "${1:-}" | tr ':' '\n' | grep -v '^/opt/ros' | paste -sd: -
}
export PATH="$(_strip_ros "$PATH")"
export AMENT_PREFIX_PATH="$(_strip_ros "${AMENT_PREFIX_PATH:-}")"
export CMAKE_PREFIX_PATH="$(_strip_ros "${CMAKE_PREFIX_PATH:-}")"
export PKG_CONFIG_PATH="$(_strip_ros "${PKG_CONFIG_PATH:-}")"
export LD_LIBRARY_PATH="$(_strip_ros "${LD_LIBRARY_PATH:-}")"

# Ensure the conda env's libstdc++ (GCC 14, provides CXXABI_1.3.15) is found
# before the system GCC 11 version, which lacks that symbol.
# LDFLAGS is not read by CMake; must go in CMAKE_{EXE,SHARED}_LINKER_FLAGS.
_CONDA_LDFLAG="-L${CONDA_PREFIX}/lib"

colcon build \
    --base-paths src \
    --symlink-install \
    --packages-skip \
        ign_ros2_control \
        ign_ros2_control_demos \
        gz_ros2_control_demos \
        gz_ros2_control_tests \
        ros_gz_sim_demos \
        ros_gz_point_cloud \
        ros_gz \
        ros_ign \
        gpsd_client \
        gps_tools \
        gps_umd \
        ros_ign_gazebo_demos \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DBUILD_TESTING=OFF \
        "-DCMAKE_EXE_LINKER_FLAGS=${_CONDA_LDFLAG}" \
        "-DCMAKE_SHARED_LINKER_FLAGS=${_CONDA_LDFLAG}" \
    "$@"

echo
echo "Build complete. Workspace overlay: $REPO/install/setup.bash"
