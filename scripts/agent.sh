#!/usr/bin/env bash
# Micro XRCE-DDS Agent: bridges PX4's uxrce_dds_client (SITL: UDP 8888, FC: serial or
# ethernet) to ROS 2 topics /fmu/in/* and /fmu/out/*.
#
# The agent is not packaged for conda-forge/robostack, so it is taken from the system
# (or from $SARAX_XRCE_AGENT).  Use v2.4.x: v3 targets Fast-DDS 3, which this
# Humble environment does not have.  Build and install it once, per PX4's docs:
#   git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
#   cd Micro-XRCE-DDS-Agent && mkdir build && cd build && cmake .. && make -j && sudo make install && sudo ldconfig
#
# Invoke via:  pixi run agent        (SITL: udp4 -p 8888; port from $SARAX_XRCE_PORT)
#              pixi run agent serial --dev /dev/ttyUSB0 -b 921600
#                  (vehicle: companion computer on the FC's serial link; PX4 needs
#                   UXRCE_DDS_CFG = that port and SER_<port>_BAUD = the same baud rate)
# Any arguments are passed to MicroXRCEAgent unchanged (see `MicroXRCEAgent --help`).
set -euo pipefail

AGENT="${SARAX_XRCE_AGENT:-$(command -v MicroXRCEAgent || true)}"
if [ -z "$AGENT" ] || [ ! -x "$AGENT" ]; then
    echo "ERROR: MicroXRCEAgent not found. Install v2.4.x (see the header of scripts/agent.sh)" >&2
    echo "       or set SARAX_XRCE_AGENT=/path/to/MicroXRCEAgent." >&2
    exit 1
fi

# The pixi env ships a different libfastcdr with the same soname; run the agent
# against its own libraries.
unset LD_LIBRARY_PATH
if [ "$#" -eq 0 ]; then
    set -- udp4 -p "${SARAX_XRCE_PORT:-8888}"
fi
# Default verbosity (info) logs every created entity and grows quickly; keep it at errors.
case " $* " in *" -v "*) ;; *) set -- "$@" -v "${SARAX_XRCE_VERBOSITY:-2}" ;; esac
# Replace a previous agent with the same arguments (same port or device); agents on other
# ports, e.g. a second SITL instance, stay up.
pkill -f "[M]icroXRCEAgent $*" 2>/dev/null || true
echo ">>> $AGENT $*"
exec "$AGENT" "$@"
