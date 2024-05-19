# Sarax on Docker container
>[!NOTE]
>## Prerequisites
>This guide assumes the following:
>
>1. [Docker](https://docs.docker.com/engine/install/) is installed and configured
>2. [Docker prerequisites](Docker%20prerequisites.md) are setup and installed

## Using the container

### Pulling the container from GHCR
This repository creates and publishes a Docker container onto the GitHub Container Registry as a [package](https://github.com/Arief-AK/sarax/pkgs/container/sarax-framework). The container can be pulled with the command below.
```shell
docker pull ghcr.io/arief-ak/sarax-framework:latest
```
<!-- ### Building the container
Optionally, the container can be built using the [Dockerfile](../Dockerfile) in the source. This can be done after cloning the repository and building the [Dockerfile](../Dockerfile) with the command below.
```shell
docker build .
``` -->

### Running the container
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
arief@ARIEF-ROG-G531:~$ docker image list
REPOSITORY                         TAG       IMAGE ID       CREATED         SIZE
ghcr.io/arief-ak/sarax-framework   latest    020e6e5b7106   8 hours ago     7.41GB
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
           --privileged 020 bash
```

4. Head to the workspace and run the following bash file
```shell
cd $SARAX_WS/PX4-Autopilot && ./sarax_plus_sitl.bash
```

5. In another terminal, run another instance of the docker container
```shell
docker exec -it sarax_container bash
```

6. Run the `sarax` framework
```shell
roslaunch m4e_mani_base sarax_plus_sitl.launch
```

Below is a screenshot of the framework without `QGroundControl` in the docker container
![Sarax Preview](images/Sarax%20Preview.png)

7. On the host machine, open `QGroundControl` and ensure that the `Virtual Joystick` setting is enabled in the `Application Settings`
![Joystick](images/Virtual%20Joystick%20Setting.png)

8. Create a new `Comm Link` in `Application Settings -> Comm Link`
![New Comm Link](images/New%20Comm%20Link.png)

9. Connect to the Comm Link by double-clicking and head back to the map view. QGroundControl should be connected to the container.
![QGround Connected](images/QGroundControl%20Connected.png)

### Getting started with Sarax GUI
To perform a simple take-off procedure, proceed with the following commands.

1. Arm the drone
2. Switch to Offboard
3. Set value for Takeoff to `1.5m` and press Takeoff

The Sarax GUI and simulator should look like this
![Takeoff Preview](images/Takeoff%20preview.png)

Congrats, Sarax is successfully installed on the docker container.