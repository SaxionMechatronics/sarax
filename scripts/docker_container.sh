#!/bin/bash

# Function to open a new terminal and run commands
open_new_terminal() {
  if command -v gnome-terminal &> /dev/null; then
    gnome-terminal -- bash -c "$1; exec bash"
  elif command -v xterm &> /dev/null; then
    xterm -e "bash -c '$1; exec bash'"
  elif command -v konsole &> /dev/null; then
    konsole -e "bash -c '$1; exec bash'"
  elif command -v xfce4-terminal &> /dev/null; then
    xfce4-terminal -e "bash -c '$1; exec bash'"
  else
    echo "No compatible terminal found"
    exit 1
  fi
}

IMAGE_REPO=$1
IMAGE_NAME=$2
IMAGE_TAG=$3
SYSTEM=$4
CONTAINER_NAME=$5
OPTION=$6

echo "SYSTEM: $SYSTEM"
echo "IMAGE URL: $IMAGE_REPO"
echo "IMAGE NAME: $IMAGE_NAME"
echo "IMAGE TAG: $IMAGE_TAG"
echo "CONTAINER NAME: $CONTAINER_NAME"
echo "OPTION: $OPTION"

if [ $OPTION = 'Images' ]; then
    echo -e "\nListing Docker image"
    docker image ls
fi

if [ $OPTION = 'Download' ]; then
    echo -e "\nDownloading Docker image"
    docker pull $IMAGE_REPO
fi

if [ $OPTION = 'Run' ]; then
    echo "Finding the Docker image"
    IMAGE_ID=$(docker images | grep "^$IMAGE_NAME\s*$IMAGE_TAG" | awk '{print $3}')
    echo "$IMAGE_ID"

    echo "Allow xhost to access container and setup authorisation"
    xhost local:root && XAUTH=/tmp/.docker.xauth

    echo "Attempting to build Docker image"
    if [ $SYSTEM = 'Linux' ]; then
        SIMULATOR_COMMANDS="docker run --name $CONTAINER_NAME --runtime nvidia --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -it  -v /tmp/.X11-unix:/tmp/.X11-unix -e "DISPLAY=$DISPLAY" -e "QT_X11_NO_MITSHM=1" -e "XAUTHORITY=$XAUTH" --device /dev/dri/ -e "WAYLAND_DISPLAY=$WAYLAND_DISPLAY" -e "XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" -e "PULSE_SERVER=$PULSE_SERVER" -p 18570:18570/udp --privileged $IMAGE_ID bash"
        SARAX_FRAMEWORK_COMMANDS="docker exec -it sarax_container bash && roslaunch m4e_mani_base sarax_plus_sitl.launch"
    fi

    if [ $SYSTEM = 'WSL2' ]; then
        SIMULATOR_COMMANDS="sudo docker run --name sarax_container -it -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg -v /usr/lib/wsl:/usr/lib/wsl --device=/dev/dxg -e DISPLAY=$DISPLAY --device /dev/dri/card0 --device /dev/dri/renderD128 -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR -e PULSE_SERVER=$PULSE_SERVER --gpus all $IMAGE_ID bash"
        SARAX_FRAMEWORK_COMMANDS="docker exec -it sarax_container bash && roslaunch m4e_mani_base sarax_plus_sitl.launch"
    fi

    echo "Running container in a new terminal on $SYSTEM"
    open_new_terminal "$SIMULATOR_COMMANDS"
    open_new_terminal "$SARAX_FRAMEWORK_COMMANDS"
fi

if [ $OPTION = 'Start' ]; then
    echo -e "\nStarting Docker container"
    docker start $CONTAINER_NAME
    echo -e "\nEntering Docker container"
    COMMAND="docker exec -it sarax_container bash"
    open_new_terminal "$COMMAND"
fi 

if [ $OPTION = 'Stop' ]; then
    echo -e "\nStopping Docker container"
    docker stop $CONTAINER_NAME
fi 

if [ $OPTION = 'Remove' ]; then
    echo -e "\nRemoving Docker container"
    docker container rm $CONTAINER_NAME
fi 