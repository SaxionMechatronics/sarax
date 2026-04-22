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

# Source the workspace overlay so the sarax model SDF and the pixi-built
# gz_ros2_control plugin are on the ament index before Gazebo starts.
# (The ROS 2 base env is already active via pixi.)
source "$REPO/install/setup.bash"

cd "$PX4_DIR"
exec make px4_sitl gz_sarax_plus
