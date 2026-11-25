#!/usr/bin/env bash
# Terminal 1: pixi run motor_hardware
# Select an adapter: pixi run motor_hardware can_interface:=can1
# Configure/zero/enable hardware and activate controllers manually in terminal 2.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

# Platform values (CAN interface and bitrate, motor IDs, limits, gains, impedance law)
# come from the hw profile, sarax_bringup/config/profiles/hw.yaml. This script drives
# the real motors, so SARAX_TARGET defaults to hw here; another target is refused.
export SARAX_TARGET="${SARAX_TARGET:-hw}"

LAUNCH_ARGS=()
if [[ "${1:-}" == -- ]]; then
    shift
fi
for arg in "$@"; do
    case "$arg" in
        can_interface:=*)
            CAN_INTERFACE="${arg#can_interface:=}"
            if [[ -z "$CAN_INTERFACE" ]]; then
                echo "can_interface must name a CAN adapter." >&2
                exit 1
            fi
            ;;
        *) LAUNCH_ARGS+=("$arg") ;;
    esac
done

if [[ ! -f "$REPO/install/setup.bash" ]]; then
    echo "Workspace overlay not found. Run 'pixi run build' first." >&2
    exit 1
fi

set +u
# shellcheck source=/dev/null
source "$REPO/install/setup.bash"
set -u
export ROS_DOMAIN_ID=0
export LD_LIBRARY_PATH="$REPO/.pixi/envs/default/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

for tool in ip python ros2 sudo; do
    if ! command -v "$tool" >/dev/null; then
        echo "Required command '$tool' is missing. Run this script through 'pixi run motor_hardware'." >&2
        exit 1
    fi
done
IP="$(command -v ip)"
profile_get() { python3 -m sarax_bringup.profile --default hw get "$1"; }
if [[ "$(python3 -m sarax_bringup.profile --default hw target)" != hw ]]; then
    echo "motor_hardware needs SARAX_TARGET=hw (got '$SARAX_TARGET')." >&2
    exit 1
fi
# Precedence: can_interface:= argument > SARAX_CAN_INTERFACE > hw profile > auto-detect.
CAN_INTERFACE="${CAN_INTERFACE:-${SARAX_CAN_INTERFACE:-$(profile_get can_interface)}}"
CAN_BITRATE="$(profile_get can_bitrate)"
# Check the ROS installation before changing the adapter state.
ros2 pkg prefix sarax_bringup >/dev/null
ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null

# shellcheck source=scripts/can_setup.sh
source "$REPO/scripts/can_setup.sh"
sarax_can_setup "$CAN_INTERFACE" "$CAN_BITRATE"

echo "Launching with CAN interface $CAN_INTERFACE, ROS_DOMAIN_ID=$ROS_DOMAIN_ID and RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION."
echo "Continue the manual hardware/zero/enable/controller sequence in terminal 2."
# Motor settings come from the hw profile (the launch file reads it too), so editing
# hw.yaml needs only a task restart, not a rebuild. Command-line launch arguments
# below override them. hardware_verified is a per-run acknowledgement, not profile data.
exec ros2 launch sarax_bringup ak60_hardware.launch.py \
    can_interface:="$CAN_INTERFACE" \
    hardware_verified:=true \
    "${LAUNCH_ARGS[@]}"
