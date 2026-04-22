#!/usr/bin/env bash
# Build the ROS 2 workspace with colcon.
# Skips the Ignition/Fortress and demo packages not needed at runtime.
#
# Invoke via:  pixi run build
# The pixi environment already has ROS 2 activated — no manual sourcing needed.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$REPO"

colcon build \
    --symlink-install \
    --packages-skip \
        ign_ros2_control \
        ign_ros2_control_demos \
        gz_ros2_control_demos \
        gz_ros2_control_tests \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DBUILD_TESTING=OFF \
    "$@"

echo
echo "Build complete. Workspace overlay: $REPO/install/setup.bash"
