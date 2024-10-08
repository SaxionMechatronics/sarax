# Sarax on Docker container

> \[!NOTE]
>
> ### Prerequisites
>
> This guide assumes the following:
>
> 1. [Docker](https://docs.docker.com/engine/install/) is installed and configured
> 2. [Docker prerequisites](docker-prerequisites.md) are setup and installed

## Installation

### Option 1: Using the CLI Application

Users can use the CLI application to handle all Docker functionalities

Run it with the following command

```shell
sudo chmod +x scripts/run.sh && ./scripts/run.sh
```

With the application, users are provided with the following options. ![Docker Config](<../.gitbook/assets/Sarax Docker Config.png>)

### Option 2: Manual Installation

#### Pulling the container from GHCR

This repository creates and publishes a Docker container onto the GitHub Container Registry as a [package](https://github.com/SaxionMechatronics/sarax/pkgs/container/sarax-framework). The container can be pulled with the command below.

```shell
docker pull ghcr.io/SaxionMechatronics/sarax-framework:latest
```

#### Running the container

To ensure full functionality of the Docker container, we will (create) and run the container with some privileges.

1. On the host machine, allow access for `xhost` from the container and setup the authorisation.

```shell
xhost local:root && XAUTH=/tmp/.docker.xauth
```

2. Find the correct image

```shell
docker image ls
```

The output should be similar to this

```shell
REPOSITORY                         TAG       IMAGE ID       CREATED       SIZE
ghcr.io/SaxionMechatronics/sarax-framework   latest    cb4ae397281c   4 days ago    7.41GB
```

3. Create the container with the name `sarax_container` and run it with the necessary privileges for running GUI applications.

```shell
docker run --name sarax_container --runtime nvidia --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -it  -v /tmp/.X11-unix:/tmp/.X11-unix \
           -e "DISPLAY=$DISPLAY" \
           -e "QT_X11_NO_MITSHM=1" \
           -e "XAUTHORITY=$XAUTH" \
           --device /dev/dri/ \
           -e "WAYLAND_DISPLAY=$WAYLAND_DISPLAY" -e "XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" \
           -e "PULSE_SERVER=$PULSE_SERVER" \
           -p 18570:18570/udp \
           --privileged cb4 bash
```

> \[!NOTE]
>
> ### Running on WSL2
>
> To run the Docker container with GPU acceleration on a `Windows` machine with `WSL2` enabled. Proceed with the following procedure.
>
> #### Docker Desktop
>
> 1. Find the correct image
>
> ```
> docker image ls
> ```
>
> The output should be similar to this
>
> ```shell
> REPOSITORY                         TAG       IMAGE ID       CREATED       SIZE
> ghcr.io/SaxionMechatronics/sarax-framework   latest    cb4ae397281c   4 days ago    7.41GB
> ```
>
> 2. Run the container from image `cb4` and name it `sarax_container`
>
> ```shell
> sudo docker run --name sarax_container -it -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg -v /usr/lib/wsl:/usr/lib/wsl --device=/dev/dxg -e DISPLAY=$DISPLAY --device /dev/dri/card0 --device /dev/dri/renderD128 -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR -e PULSE_SERVER=$PULSE_SERVER --gpus all cb4 bash
> ```

## Running Sarax on Docker Container

1. Head to the workspace and run the following bash file

```shell
cd $SARAX_WS/PX4-Autopilot && ./sarax_plus_sitl.bash
```

2. In another terminal, run another instance of the docker container

```shell
docker exec -it sarax_container bash
```

3. Run the `sarax` framework

```shell
roslaunch m4e_mani_base sarax_plus_sitl.launch
```

Below is a screenshot of the framework without `QGroundControl` in the docker container ![Sarax Preview](<../.gitbook/assets/Sarax Preview.png>)

4. On the host machine, open `QGroundControl` and ensure that the `Virtual Joystick` setting is enabled in the `Application Settings` ![Joystick](<../.gitbook/assets/Virtual Joystick Setting.png>)
5. Create a new `Comm Link` in `Application Settings -> Comm Link` ![New Comm Link](<../.gitbook/assets/New Comm Link.png>)
6. Connect to the Comm Link by double-clicking and head back to the map view. QGroundControl should be connected to the container. ![QGround Connected](<../.gitbook/assets/QGroundControl Connected.png>)

## Getting started with Sarax GUI

To perform a simple take-off procedure, proceed with the following commands.

1. Arm the drone
2. Switch to Offboard
3. Set value for Takeoff to `1.5m` and press Takeoff

The Sarax GUI and simulator should look like this ![Takeoff Preview](<../.gitbook/assets/Takeoff preview.png>)

Congrats, Sarax is successfully installed on the docker container.
