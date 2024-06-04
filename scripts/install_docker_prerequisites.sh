#!/bin/sh

# Creating a dependency worskpace
mkdir -p ~/dependencies/NVIDIA
cd ~/dependencies/NVIDIA

# Prepare the repos
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update the packages
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit

# Configure Docker
nvidia-ctk runtime configure --runtime=docker --config=$HOME/.config/docker/daemon.json
sudo nvidia-ctk config --set nvidia-container-cli.no-cgroups --in-place