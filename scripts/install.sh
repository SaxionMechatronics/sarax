#!/usr/bin/env bash
# One-time setup for the SARAX+ workspace.
# Safe to re-run; each step is idempotent.
#
# All dependencies come from the pixi conda environment (robostack-humble +
# conda-forge).  No system-wide apt packages are installed.
#
# Prerequisites: pixi is installed (https://pixi.sh) and 'pixi install' has
# been run (or this script is invoked via 'pixi run install', which does it
# automatically).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

step() { echo; echo "──────────────────────────────────────────────"; echo ">>> $*"; echo "──────────────────────────────────────────────"; }

# ── 1. Remove Fortress/Ignition ros2_control packages if present ─────────────
# These may have been installed previously system-wide and register duplicate
# pluginlib descriptors that cause ClassLoaderException at runtime.
step "Removing any conflicting system-wide gz_ros2_control apt packages"
for pkg in \
    ros-humble-ign-ros2-control \
    ros-humble-ign-ros2-control-demos \
    ros-humble-gz-ros2-control \
    ros-humble-gz-ros2-control-demos; do
    if dpkg -l "$pkg" &>/dev/null 2>&1; then
        sudo apt-get remove -y "$pkg"
        echo "  removed $pkg"
    fi
done

# ── 2. Clone external sources ─────────────────────────────────────────────────
step "Cloning external sources via vcstool"
cd "$REPO"

# PX4 submodules are required for a full SITL build.
vcs import --recursive < deps.repos

# ── 3. Drop Ignition/Fortress sub-packages from gz_ros2_control ──────────────
# ign_ros2_control targets Gazebo Fortress (libignition-gazebo6).  It registers
# duplicate pluginlib base-class descriptors that clash with the Harmonic build
# at runtime even when the ign packages are never loaded.
step "Removing ign_ros2_control (Fortress) from source tree"
rm -rf \
    "$REPO/src/gz_ros2_control/ign_ros2_control" \
    "$REPO/src/gz_ros2_control/ign_ros2_control_demos"

# ── 4. Build the ROS 2 workspace ──────────────────────────────────────────────
bash "$REPO/scripts/build.sh"

echo
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  Setup complete.                                             ║"
echo "║                                                              ║"
echo "║  Terminal 1:  pixi run px4   (PX4 SITL + Gazebo)            ║"
echo "║  Terminal 2:  pixi run ros   (ROS 2 side, after world ready) ║"
echo "╚══════════════════════════════════════════════════════════════╝"
