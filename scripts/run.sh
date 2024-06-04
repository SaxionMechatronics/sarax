#!/bin/sh

PACKAGE_NAME="figlet"

if apt -qq list $PACKAGE_NAME 2>/dev/null | grep -q "installed"; then
    figlet SARAX
else
    echo "$PACKAGE_NAME is not installed."
    sudo apt-get install figlet -y
    sudo apt-get install gnome-terminal -y
    figlet SARAX
fi

# Modify permissions for all scripts
cd scripts
sudo chmod +x check_existence.sh
sudo chmod +x docker_container.sh
sudo chmod +x install_docker_prerequisites.sh
sudo chmod +x install_docker.sh
sudo chmod +x install_sarax_linux.sh
sudo chmod +x install_sarax_wsl2.sh
sudo chmod +x container/docker_entrypoint.sh
sudo chmod +x container/check_and_source.sh

# Run the menu application
python3 menu.py