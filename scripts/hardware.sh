#!/usr/bin/env bash
# One hardware bringup: CAN, arm, XRCE agent, state_bridge and the flight recording.
#
#   pixi run hardware                                  # real AK60 motors
#   pixi run hardware can_interface:=can1              # explicit adapter
#   pixi run hardware can_setup:=skip                  # adapter already configured
#   pixi run hardware hardware:=mock                   # mock arm: no CAN adapter, no motors
#   pixi run hardware extra_topics:='/inspection_target /operator/status'
#   pixi run hardware agent_args:='serial --dev /dev/ttyUSB0 -b 921600'   # vehicle
#
# Real arm: the CAN adapter is configured first (sudo may ask for a password); no adapter,
# an ambiguous one or a failed ip command stops before anything starts. Mock arm: CAN setup
# is skipped. Other arguments go to scripts/../sarax_bringup/launch/hardware.launch.py.
# Platform values come from the hw profile; SARAX_TARGET defaults to hw and nothing else is
# accepted. SITL PX4 can stand in for the vehicle: the default agent connects to udp4 8888.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

export SARAX_TARGET="${SARAX_TARGET:-hw}"

LAUNCH_ARGS=()
HARDWARE=real
CAN_SETUP=auto
CAN_REQUEST=""
if [[ "${1:-}" == -- ]]; then
    shift
fi
for arg in "$@"; do
    case "$arg" in
        can_interface:=*) CAN_REQUEST="${arg#can_interface:=}"
            if [[ -z "$CAN_REQUEST" ]]; then
                echo "can_interface must name a CAN adapter." >&2
                exit 1
            fi ;;
        can_setup:=*) CAN_SETUP="${arg#can_setup:=}" ;;
        hardware:=*) HARDWARE="${arg#hardware:=}"; LAUNCH_ARGS+=("$arg") ;;
        *) LAUNCH_ARGS+=("$arg") ;;
    esac
done
case "$HARDWARE" in real|mock) ;; *) echo "hardware must be real or mock (got '$HARDWARE')." >&2; exit 1 ;; esac
case "$CAN_SETUP" in auto|skip) ;; *) echo "can_setup must be auto or skip (got '$CAN_SETUP')." >&2; exit 1 ;; esac

if [[ ! -f "$REPO/install/setup.bash" ]]; then
    echo "Workspace overlay not found. Run 'pixi run build' first." >&2
    exit 1
fi

set +u
# shellcheck source=/dev/null
source "$REPO/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export LD_LIBRARY_PATH="$REPO/.pixi/envs/default/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
# The XRCE agent speaks Fast DDS, so the ROS side must too (motor_hardware.sh uses Cyclone
# DDS, which cannot see the agent). Terminal 2 for manual commands needs the same RMW.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export SARAX_AGENT_SCRIPT="$REPO/scripts/agent.sh"
export SARAX_BAG_DIR="${SARAX_BAG_DIR:-$REPO/bags}"

for tool in python ros2; do
    if ! command -v "$tool" >/dev/null; then
        echo "Required command '$tool' is missing. Run this script through 'pixi run hardware'." >&2
        exit 1
    fi
done
if [[ "$(python3 -m sarax_bringup.profile --default hw target)" != hw ]]; then
    echo "hardware needs SARAX_TARGET=hw (got '$SARAX_TARGET')." >&2
    exit 1
fi
ros2 pkg prefix sarax_bringup >/dev/null
ros2 pkg prefix "$RMW_IMPLEMENTATION" >/dev/null

CAN_ARGS=()
if [[ "$HARDWARE" == mock ]]; then
    echo "Mock arm: CAN setup skipped."
elif [[ "$CAN_SETUP" == skip ]]; then
    # Adapter configured elsewhere: only the interface name is needed.
    CAN_INTERFACE="${CAN_REQUEST:-${SARAX_CAN_INTERFACE:-$(python3 -m sarax_bringup.profile --default hw get can_interface)}}"
    if [[ -z "$CAN_INTERFACE" ]]; then
        echo "can_setup:=skip needs the adapter name: can_interface:=<name> or SARAX_CAN_INTERFACE." >&2
        exit 1
    fi
    echo "CAN setup skipped; using $CAN_INTERFACE as it is."
    CAN_ARGS=(can_interface:="$CAN_INTERFACE" hardware_verified:=true)
else
    IP="$(command -v ip || true)"
    if [[ -z "$IP" ]]; then
        echo "Required command 'ip' is missing; cannot set up CAN (use hardware:=mock or can_setup:=skip)." >&2
        exit 1
    fi
    # Precedence: can_interface:= argument > SARAX_CAN_INTERFACE > hw profile > auto-detect.
    CAN_REQUEST="${CAN_REQUEST:-${SARAX_CAN_INTERFACE:-$(python3 -m sarax_bringup.profile --default hw get can_interface)}}"
    CAN_BITRATE="$(python3 -m sarax_bringup.profile --default hw get can_bitrate)"
    # shellcheck source=scripts/can_setup.sh
    source "$REPO/scripts/can_setup.sh"
    sarax_can_setup "$CAN_REQUEST" "$CAN_BITRATE" || {
        echo "CAN setup failed; nothing was started. Use hardware:=mock to run without the motors." >&2
        exit 1
    }
    CAN_ARGS=(can_interface:="$CAN_INTERFACE" hardware_verified:=true)
fi

echo "Launching hardware bringup (arm: $HARDWARE, ROS_DOMAIN_ID=$ROS_DOMAIN_ID, bags in $SARAX_BAG_DIR)."
exec ros2 launch sarax_bringup hardware.launch.py "${CAN_ARGS[@]}" "${LAUNCH_ARGS[@]}"
