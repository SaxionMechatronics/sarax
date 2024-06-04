#!/bin/bash

cd $SARAX_WS/src/sarax/scripts/container

# Check if both lines are uncommented
grep -q '^# export LD_LIBRARY_PATH=/usr/lib/wsl/lib' ~/.bashrc && sed -i 's/^# export LD_LIBRARY_PATH=\/usr\/lib\/wsl\/lib$/export LD_LIBRARY_PATH=\/usr\/lib\/wsl\/lib/' ~/.bashrc || echo "Initialised running for WSL2"
grep -q '^# export LIBVA_DRIVER_NAME=d3d12' ~/.bashrc && sed -i 's/^# export LIBVA_DRIVER_NAME=d3d12$/export LIBVA_DRIVER_NAME=d3d12/' ~/.bashrc && echo "Please re-source the bash script using the following command: source ~/.bashrc" || python3 container_menu.py

# echo "Please re-source the bash script using the following command: source ~/.bashrc"