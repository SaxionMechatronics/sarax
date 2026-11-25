#!/usr/bin/env bash
# Shared CAN adapter selection and setup; sourced by motor_hardware.sh and hardware.sh.
#   sarax_can_setup <requested-interface-or-empty> <bitrate>
# Sets CAN_INTERFACE. Exits non-zero with a message when no adapter is found, the requested
# one does not exist, several are present without a selection, or an `ip` command fails.
# Needs IP (path of `ip`) and python in the environment; sudo may ask for a password.

sarax_can_setup() {
    local requested="$1" bitrate="$2"
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
             "can_interface:=<name>" % ", ".join(interfaces))
print(selected)
' "$requested")" || return 1

    echo "Configuring $CAN_INTERFACE: classic CAN, $bitrate bit/s (sudo may ask for your password)."
    sudo "$IP" link set dev "$CAN_INTERFACE" down
    sudo "$IP" link set dev "$CAN_INTERFACE" type can bitrate "$bitrate" fd off
    sudo "$IP" link set dev "$CAN_INTERFACE" up
}
