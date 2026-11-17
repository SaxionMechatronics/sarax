#!/usr/bin/env bash
# Terminal 1: pixi run motor_hardware
# Select an adapter: pixi run motor_hardware can_interface:=can1
# Configure/zero/enable hardware and activate controllers manually in terminal 2.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

CAN_INTERFACE="${SARAX_CAN_INTERFACE:-}"
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
# Check the ROS installation before changing the adapter state.
ros2 pkg prefix sarax_bringup >/dev/null
ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null

# Use the kernel-reported CAN kind, not an interface-name pattern. This also
# excludes virtual CAN devices, which cannot be configured with a bitrate.
CAN_INTERFACE="$("$IP" -details -json link show type can | python -c '
import json
import sys

interfaces = sorted({
    link["ifname"] for link in json.load(sys.stdin)
    if link.get("linkinfo", {}).get("info_kind") == "can"
})
requested = sys.argv[1]
if requested:
    if requested not in interfaces:
        sys.exit("CAN interface %r was not found. Available CAN interfaces: %s"
                 % (requested, ", ".join(interfaces) or "(none)"))
    selected = requested
elif len(interfaces) == 1:
    selected = interfaces[0]
elif not interfaces:
    sys.exit("No CAN adapter found. Connect it and check: ip -details link show type can")
else:
    sys.exit("Multiple CAN interfaces found: %s. Select the motor adapter with "
             "pixi run motor_hardware can_interface:=<name>" % ", ".join(interfaces))
print(selected)
' "$CAN_INTERFACE")"

echo "Configuring $CAN_INTERFACE: classic CAN, 1000000 bit/s (sudo may ask for your password)."
sudo "$IP" link set dev "$CAN_INTERFACE" down
sudo "$IP" link set dev "$CAN_INTERFACE" type can bitrate 1000000 fd off
sudo "$IP" link set dev "$CAN_INTERFACE" up

echo "Launching with CAN interface $CAN_INTERFACE, ROS_DOMAIN_ID=$ROS_DOMAIN_ID and RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION."
echo "Continue the manual hardware/zero/enable/controller sequence in terminal 2."
# Default motor settings. Command-line launch arguments below override these.
# Editing this script only requires restarting the task, without a rebuild.
exec ros2 launch sarax_bringup ak60_hardware.launch.py \
    can_interface:="$CAN_INTERFACE" \
    hardware_verified:=true \
    use_second_motor:=true \
    motor_1_id:=2 \
    motor_2_id:=3 \
    impedance_law:=sarax_impedance_controller::PDGravityLaw \
    motor_1_effort_limit:=7 \
    motor_2_effort_limit:=7 \
    motor_1_stiffness:=9.0 \
    motor_1_damping:=1.0 \
    motor_2_stiffness:=5.0 \
    motor_2_damping:=1.0 \
    max_position_error:=1.0 \
    "${LAUNCH_ARGS[@]}"
