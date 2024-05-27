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

# Run the menu application
cd scripts && python3 menu.py