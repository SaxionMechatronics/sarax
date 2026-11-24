#!/usr/bin/env bash
# Launch PX4 SITL with the sarax_plus model in Gazebo Harmonic.
#
# Builds PX4 on the first run (takes a few minutes); subsequent launches
# reuse the cached build.
#
# After ">>> Spawning sarax_plus_0" appears, open a second terminal and run:
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

# PX4 and Gazebo load their shared libraries from the Pixi environment.
export LD_LIBRARY_PATH="$REPO/.pixi/envs/default/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

cd "$PX4_DIR"

# Wipe stored parameters on every SITL launch so the airframe's param set-default
# values always apply from scratch. A persisted parameters.bson carries sensor
# calibration IDs across restarts that no longer match and fail the preflight checks.
_rootfs="$PX4_DIR/build/px4_sitl_default/rootfs"
rm -f "$_rootfs/parameters.bson" "$_rootfs/parameters_backup.bson" "$_rootfs/dataman"

# ── SITL-only parameter overrides (applied by rcS from PX4_PARAM_<name>) ─────
# They come from the `px4_params` section of the SARAX_TARGET profile
# (sarax_bringup/config/profiles; default sitl, e.g. NAV_DLL_ACT=0: no data-link-loss
# action, so arming does not require a ground station). An already exported
# PX4_PARAM_<name> wins. This script starts the simulator, so a target other than
# sitl is refused. On the vehicle keep a GCS link and its failsafes.
_target="$(python3 -m sarax_bringup.profile target)" || exit 1
if [ "$_target" != "sitl" ]; then
    echo "ERROR: scripts/px4.sh starts PX4 SITL; SARAX_TARGET=$_target is not a simulator target."
    exit 1
fi
_px4_env="$(python3 -m sarax_bringup.profile px4-env)" || exit 1
eval "$_px4_env"

# ── World and model come from this repo, not from the PX4 tree ───────────────
# The PX4 fork's Tools/simulation/gz submodule is upstream PX4-gazebo-models,
# which has no sarax_plus. Start Gazebo with a world (PX4's empty default world
# unless SARAX_GZ_WORLD names one in sarax_gz_sim/worlds), spawn sarax_plus_0
# from src/sarax_gz_sim, then let PX4 attach to it (standalone mode).
make px4_sitl   # builds on the first run; creates rootfs/gz_env.sh

# PX4's Gazebo environment (server.config with the sensor systems, PX4 plugins).
# It appends the PX4 model path after the overlay's, so model://sarax_plus and
# its meshes resolve to the sarax_gz_sim copy.
# shellcheck disable=SC1091
source "$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh"

if [ -n "${SARAX_GZ_WORLD:-}" ]; then
    WORLD_SDF="$REPO/install/sarax_gz_sim/share/sarax_gz_sim/worlds/$SARAX_GZ_WORLD.sdf"
else
    WORLD_SDF="$PX4_DIR/Tools/simulation/gz/worlds/default.sdf"
fi
if [ ! -f "$WORLD_SDF" ]; then
    echo "ERROR: world not found: $WORLD_SDF"
    exit 1
fi

echo ">>> Starting Gazebo with $WORLD_SDF"
gz sim --verbose=1 -r -s "$WORLD_SDF" &
GZ_SERVER_PID=$!
GZ_GUI_PID=""
if [ -z "${HEADLESS:-}" ]; then
    gz sim -g > /dev/null 2>&1 &
    GZ_GUI_PID=$!
fi
trap 'kill $GZ_SERVER_PID $GZ_GUI_PID 2>/dev/null || true' EXIT INT TERM

for _ in $(seq 1 60); do
    gz service -i --service /world/default/scene/info 2>&1 | grep -q "Service providers" && break
    sleep 1
done

# gz_ros2_control needs the absolute controllers.yaml path: fill in the model's
# placeholder for this checkout and spawn that copy.
MODEL_SRC="$REPO/install/sarax_gz_sim/share/sarax_gz_sim/models/sarax_plus/model.sdf"
CONTROLLERS_YAML="$REPO/install/sarax_bringup/share/sarax_bringup/config/controllers.yaml"
MODEL_SDF="$REPO/build/sarax_plus_spawn.sdf"
mkdir -p "$REPO/build"
sed "s#@SARAX_CONTROLLERS_YAML@#$CONTROLLERS_YAML#" "$MODEL_SRC" > "$MODEL_SDF"
echo ">>> Spawning sarax_plus_0 from $MODEL_SDF"
gz service -s /world/default/create --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$MODEL_SDF\", name: \"sarax_plus_0\", allow_renaming: false" > /dev/null
sleep 1

# Standalone: PX4 does not start its own world; it binds to sarax_plus_0.
# Start 'pixi run ros' now: the model's gz_ros2_control plugin waits for
# robot_state_publisher.
PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_NAME=sarax_plus_0 make px4_sitl gz_sarax_plus
