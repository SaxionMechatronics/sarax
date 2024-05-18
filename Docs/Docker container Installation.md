# Dry-run Docker installation

## Pulling the Docker image
To download the Docker image, pull the image from `px4io`
```shell
docker pull px4io/px4-dev-ros-noetic
```

# Building and running the Docker container
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
REPOSITORY                         TAG          IMAGE ID       CREATED        SIZE
px4io/px4-dev-ros-noetic           latest       52e98aa51240   6 weeks ago    4.79GB
```

3. Create the container and run it with the necessary privileges for running GUI applications.
```shell
docker run --name sarax_container --runtime nvidia --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -it  -v /tmp/.X11-unix:/tmp/.X11-unix \
           -e "DISPLAY=$DISPLAY" \
           -e "QT_X11_NO_MITSHM=1" \
           -e "XAUTHORITY=$XAUTH" \
           --device /dev/dri/ \
           -e "WAYLAND_DISPLAY=$WAYLAND_DISPLAY" -e "XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" \
           -e "PULSE_SERVER=$PULSE_SERVER" \
           -p 18570:18570/udp \
           --privileged 52e bash
```
>[!NOTE]
>
>## Running on WSL2
>To run the Docker container with GPU acceleration on a `Windows` machine with `WSL2` enabled. Proceed with the following procedure.
><details>
>### Docker Desktop
>To provide access to the GPU, open Docker Desktop and head to `Settings->Docker Engine`. In the given code snippet, >replace the following the existing script with the following.
>
>> [!TIP]\
>> Make sure to create a backup of the code snippet, just in case!
>
>```JSON
>{
  >"builder": {
    >"gc": {
      >"defaultKeepStorage": "20GB",
      >"enabled": true
    >}
  >},
  >"experimental": false,
  >"runtimes": {
    >"nvidia": {
      >"path": "/usr/bin/nvidia-container-runtime",
      >"runtimeArgs": []
    >}
  >}
>}
>```
>
>1. Find the correct image
>```shell
>docker image ls
>```
>
>The output should be similar to this
>```shell
>REPOSITORY                         TAG          IMAGE ID       CREATED         SIZE
>sarax_framework_gpu_accelerated    latest       c43261cdbb5d   9 minutes ago   11.2GB
>```
>
>2. Run the container and name it `sarax_container`
>```shell
>sudo docker run --name sarax_container -it -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg -v /usr/lib/wsl:/usr/lib/wsl --device=/dev/dxg -e DISPLAY=$DISPLAY --device /dev/dri/card0 --device /dev/dri/renderD128 -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR -e PULSE_SERVER=$PULSE_SERVER --gpus all c43 bash
>```
></details>

<!-- 
```shell
sudo apt get nano && nano ~/.bashrc
```

Place this in the bottom of the file
```shell
export LIBVA_DRIVER_NAME=d3d12
export LD_LIBRARY_PATH=/usr/lib/wsl/lib
```

Source the `.bashrc`
```shell
source ~/.bashrc
```

Update the graphics library
```shell
sudo apt update && sudo apt-get install vainfo mesa-va-drives -y
``` -->

4. Inside the container,check the OpenGL info with `glxinfo`
```shell
glxinfo -B
```

The output should be like this
```shell
name of display: :0
display: :0  screen: 0
direct rendering: Yes               # <-- THIS IS IMPORTANT
Extended renderer info (GLX_MESA_query_renderer):
    Vendor: Microsoft Corporation (0xffffffff)
    Device: D3D12 (NVIDIA GeForce GTX 1650) (0xffffffff)
    Version: 21.0.3
    Accelerated: yes
    Video memory: 12052MB
    Unified memory: no
    Preferred profile: core (0x1)
    Max core profile version: 3.3
    Max compat profile version: 3.1
    Max GLES1 profile version: 1.1
    Max GLES[23] profile version: 3.0
OpenGL vendor string: Microsoft Corporation
OpenGL renderer string: D3D12 (NVIDIA GeForce GTX 1650)
OpenGL core profile version string: 3.3 (Core Profile) Mesa 21.0.3
OpenGL core profile shading language version string: 3.30
OpenGL core profile context flags: (none)
OpenGL core profile profile mask: core profile

OpenGL version string: 3.1 Mesa 21.0.3
OpenGL shading language version string: 1.40
OpenGL context flags: (none)

OpenGL ES profile version string: OpenGL ES 3.0 Mesa 21.0.3
OpenGL ES profile shading language version string: OpenGL ES GLSL ES 3.00
```

5. Check if the dedicated graphics card can be accessed within the container
```shell
nvidia-smi
```

The output should look like this
```shell
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 550.76.01              Driver Version: 552.44         CUDA Version: 12.4     |
|-----------------------------------------+------------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M. |
|                                         |                        |               MIG M. |
|=========================================+========================+======================|
|   0  NVIDIA GeForce GTX 1650        On  |   00000000:01:00.0  On |                  N/A |
| N/A   43C    P8              1W /   50W |     954MiB /   4096MiB |      4%      Default |
|                                         |                        |                  N/A |
+-----------------------------------------+------------------------+----------------------+

+-----------------------------------------------------------------------------------------+
| Processes:                                                                              |
|  GPU   GI   CI        PID   Type   Process name                              GPU Memory |
|        ID   ID                                                               Usage      |
|=========================================================================================|
|    0   N/A  N/A        25      G   /Xwayland                                   N/A      |
|    0   N/A  N/A        35      G   /Xwayland                                   N/A      |
|    0   N/A  N/A        36      G   /Xwayland                                   N/A      |
|    0   N/A  N/A       442      G   /Xwayland                                   N/A      |
|    0   N/A  N/A       920      G   /gzserver                                   N/A      |
+-----------------------------------------------------------------------------------------+
```

#### Small performance test
We can test the performance of the container with Gazebo
```shell
gazebo
```

The following condition should be true `fps > 30`

## ROS Noetic
The Docker container already contains ROS Noetic. We can ensure that this is the case by running the following commands.

1. Source the ROS noetic `setup.bash` file
```shell
source /opt/ros/noetic/setup.bash
```

2. Echo the `$ROS_DISTRO`
```shell
echo $ROS_DISTRO
```

## Installing PX4
Following the installation from the [PX4 guide](https://docs.px4.io/main/en/dev_setup/building_px4.html).

### Dependency Installation
To keep things lightweight, we will only be installing the dependencies of PX4.

1. Fetch all the required setup files and run
```shell
wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/ubuntu.sh && wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/requirements.txt && chmod +x ubuntu.sh && ./ubuntu.sh
```

2. Exit the container
```shell
exit
```

3. Restart the container
```shell
docker restart sarax_container
```

4. Enter the container
```shell
docker exec -it sarax_container bash
```

>[!NOTE]
>### Source Installation
><details>
>1. Head to workspace area and download the PX4 source code
>```shell
>cd ~ && git clone https://github.com/PX4/PX4-Autopilot.git --recursive
>```
>
>2. Source the `setup.bash` for Ubuntu to install `PX4` dependencies
>```shell
>source ~/PX4-Autopilot/Tools/setup/ubuntu.sh
>```
>
>3. First build using the `gazebo-classic` simulator
>```shell
>cd ~/PX4-Autopilot/ && make px4_sitl gazebo-classic
>```

## Installing MAVROS
### Binary installation
Install using binaries
```shell
sudo apt-get install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras
```

Install Geographic datasets
```shell
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && chmod +x install_geographiclib_datasets.sh && ./install_geographiclib_datasets.sh
```
Install ROS noetic building dependencies
```shell
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
```

Install the catkin dependencies
```shell
sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y
```
>[!NOTE]
>### Source installation
><details>
>1. Install ROS noetic building dependencies
>```shell
>sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
>```
>
>Followed MAVROS source installation from [MAVROS repo](https://github.com/mavlink/mavros/tree/master/>mavros#installation).
>
>2. Install the catkin dependencies
>```shell
>sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y
>```
>
>3. Create the workspace
>```shell
>mkdir -p ~/catkin_ws/src && cd ~/catkin_ws && catkin init && wstool init src
>```
>
>4. Install `MAVLink` using the `kinetic` reference for all ROS distros as it's not distro-specific and up to date
>```shell
>rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
>```
>
>5. Install MAVROS from the `stable` release
>```shell
>rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
>```
>
>6. Get the dependencies
>```shell
>wstool merge -t src /tmp/mavros.rosinstall && wstool update -t src -j4 && rosdep install --from-paths src --ignore-src -y
>```
>
>7. Install GeographicLib datasets
>```shell
>./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
>```
>
>8. Build the source
>```shell
>catkin build
>```
>
>> [!WARNING]\
> At time of writing, there exist a problem when attempting to build `MAVLink` and `MAVROS`. In particular, the `mavros_extras` is the source of this issue which is referenced [here](https://github.com/mavlink/mavros/issues/1938). >To resolve this issue, proceed with the following section, if, all goes well, continue to installing the Sarax section.
>>
>>### Issue: Unsucessfull build due to `mavros_extras`
>>If an error is produced after running the ```catkin build``` command, proceed with the following steps.
>><details>
>>The error should look similar to below
>>  <details>
>>    <summary>
>>    Error output
>>    </summary>
>>
>>    ```shell
>>    Errors     << mavros_extras:make /root/catkin_ws/logs/mavros_extras/build.make.000.log /root/catkin_ws/src/mavros/mavros_extras/src/plugins/gps_status.cpp: In member function ‘void mavros::extra_plugins::GpsStatusPlugin::handle_gps2_raw(const mavlink_message_t*, mavlink::common::msg::GPS2_RAW&)’: /root/catkin_ws/src/mavros/mavros_extras/src/plugins/gps_status.cpp:107:40: error: ‘struct mavlink::common::msg::GPS2_RAW’ has no member named ‘alt_ellipsoid’
>>    107 |   ros_msg->alt_ellipsoid     = mav_msg.alt_ellipsoid;
>>        |                                        ^~~~~~~~~~~~~
>>     /root/catkin_ws/src/mavros/mavros_extras/src/plugins/gps_status.cpp:108:40: error: ‘struct mavlink::common::msg::GPS2_RAW’ has no member named ‘h_acc’
>>    108 |   ros_msg->h_acc             = mav_msg.h_acc;
>>        |                                        ^~~~~
>>    /root/catkin_ws/src/mavros/mavros_extras/src/plugins/gps_status.cpp:109:40: error: ‘struct mavlink::common::msg::GPS2_RAW’ has no member named ‘v_acc’
>>    109 |   ros_msg->v_acc             = mav_msg.v_acc;
>>        |                                        ^~~~~
>>    /root/catkin_ws/src/mavros/mavros_extras/src/plugins/gps_status.cpp:110:40: error: ‘struct mavlink::common::msg::GPS2_RAW’ has no member named ‘vel_acc’
>>    110 |   ros_msg->vel_acc           = mav_msg.vel_acc;
>>        |                                        ^~~~~~~
>>    /root/catkin_ws/src/mavros/mavros_extras/src/plugins/gps_status.cpp:111:40: error: ‘struct mavlink::common::msg::GPS2_RAW’ has no member named ‘hdg_acc’
>>    111 |   ros_msg->hdg_acc           = mav_msg.hdg_acc;
>>        |                                        ^~~~~~~
>>    make[2]: *** [CMakeFiles/mavros_extras.dir/build.make:232: CMakeFiles/mavros_extras.dir/src/plugins/gps_status.cpp.o] Error 1
>>    make[2]: *** Waiting for unfinished jobs....
>>    make[1]: *** [CMakeFiles/Makefile2:735: CMakeFiles/mavros_extras.dir/all] Error 2
>>    make: *** [Makefile:141: all] Error 2
>>    cd /root/catkin_ws/build/mavros_extras; catkin build --get-env mavros_extras | catkin env -si  /usr/bin/make --jobserver-auth=3,4; cd -
>>    ```
>>  </details>
>>
>>1. Install your preferred text-editor, for this case, we will install `nano`
>>    ```shell
>>    sudo apt-get install nano
>>    ```
>>
>>2. Head to the culprit of the error in `common.xml`
>>    ```shell
>>    cd src/mavlink/message_definitions/v1.0/
>>    ```
>>
>>3. Open the `common.xml` with a text-editor and search for the `<message id="124" name="GPS_RAW2">` section
>>    ```shell
>>    nano common.xml
>>    ```
>>
>>4. Edit the file to include the required fields for the `GPS2_RAW` message
>>    ```xml
>>    <message id="124" name="GPS2_RAW">
>>        <description>Second GPS data.</description>
>>        ...
>>        <extensions/>
>>        <field type="int32_t" name="alt_ellipsoid" units="mm">Altitude (above WGS84, EGM96 ellipsoid). Positive for up</field>
>>        <field type="uint32_t" name="h_acc" units="mm">Position uncertainty.</field>
>>        <field type="uint32_t" name="v_acc" units="mm">Altitude uncertainty.</field>
>>        <field type="uint32_t" name="vel_acc" units="mm">Speed uncertainty.</field>
>>        <field type="uint32_t" name="hdg_acc" units="degE5">Heading / track uncertainty</field>
>>        <field type="uint16_t" name="yaw" units="cdeg">Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use 65535 if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for >north.</field>
>>    </message>
>>    ```
>>
>>5. Head back to `catkin_ws` and clean
>>    ```shell
>>    cd ~/catkin_ws/ && catkin clean
>>    ```
>>
>>6. Build the source
>>    ```shell
>>    catkin build
>>    ```

## Installing Sarax
Setup the Sarax software as mentioned in the setup procedures.

1. Create the workspace
```shell
mkdir -p sarax_ws/src && cd sarax_ws/ && catkin init && wstool init src
```

2. Clone the Sarax branch of the PX4-Autopilot repository
```shell
git clone --recursive -b v1.13.2-sarax-sim https://github.com/SaxionMechatronics/PX4-Autopilot.git
```

3. Head to the `src` directory and clone the Sarax repository
```shell
cd src && git clone https://github.com/SaxionMechatronics/sarax.git
```

4. Install Sarax dependencies
```shell
cd .. && source /opt/ros/noetic/setup.bash  && rosdep install --from-paths src --ignore-src -r -y --skip-keys="python-scipy"
```

5. Build the workspace
```shell
cd ~/sarax_ws/ && catkin build
```

<!-- > [!WARNING]\
> If you run into errors, ensure that the system is equipped with the necessary dependencies by running the following command
>
> <details>
>    <summary>
>    Issue: Build error    
>    </summary>
>
>If an error is produced after running the ```catkin build``` command, proceed with the following commands.
>
>1. Install the `scipy` build dependency
>    ```shell
>    pip install scipy
>    ```
>
>2. Install all the required dependencies using `rosdep`
>    ```shell
>    rosdep install --from-paths src --ignore-src -r -y --skip-keys="python-scipy"
>    ```
>
>3. Clean the workspace
>    ```shell
>    catkin clean
>    ```
>
>4. Run the build
>    ```shell
>    catkin build
>    ```
>
></details> -->

6. Create the `$SARAX_WS` variable, source the `setup.bash` and append it into `.bashrc`
```shell
echo "export SARAX_WS=$PWD" >> ~/.bashrc && echo "source \$SARAX_WS/devel/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

7. Install [`rqt`](https://wiki.ros.org/rqt/UserGuide/Install/Groovy) dependency to run the Sarax GUI properly
```shell
sudo apt-get install ros-noetic-rqt ros-noetic-rqt-common-plugins -y
```

## Running Sarax
To run Sarax, we need to connect the `QGroundControl` application to the simulation.

> [!NOTE]\
> The UDP port built on your system may vary, to find the correct UDP port, follow the steps below.
>   <details>
>       <summary>
>       Finding the correct UDP port
>   </summary>
>
>   1. Head to the build directory for Sarax
>   ```shell
>   cd ~/sarax_ws/PX4-Autopilot/build/px4_sitl_nolockstep/etc/>init.d-posix
>   ```
>
>   2. Run the following command to find the `udp_gcs_port_local` for port-forwarding
>   ```shell
>   grep "udp_gcs_port_local=" px4-rc.mavlink | cut -d'=' -f2
>   ```
>
>The output should look (similar) to this
>   ```shell
>   $((18570+px4_instance)) # We only need the number!
>   ```
>
>Now we know that port `18570` needs to be forwarded.
>
>Since we already created the existing docker container with a port associated to it. We will create a new image from the existing container. This way we don't lose our work.   
>
>   3. Exit the current container
>   ```shell
>   exit
>   ```
>   
>   4. Stop the container
>   ```shell
>   docker stop sarax_container
>   ```
>
>   5. Create a new image from the existing container
>   ```shell
>   docker commit sarax_container new_sarax_image
>   ```
>
>   6. On the host machine, run a new terminal
>   ```shell
>   xhost local:root && XAUTH=/tmp/.docker.xauth
>   ```
>
>   7. Create a new container with the correct the new port
>   ```shell
>   docker run --name sarax_container --runtime nvidia --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -it  -v /tmp/.X11-unix:/tmp/.X11-unix \
>           -e "DISPLAY=$DISPLAY" \
>           -e "QT_X11_NO_MITSHM=1" \
>           -e "XAUTHORITY=$XAUTH" \
>           --device /dev/dri/ \
>           -e "WAYLAND_DISPLAY=$WAYLAND_DISPLAY" -e "XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" \
>           -e "PULSE_SERVER=$PULSE_SERVER" \
>           -p 18570:18570/udp \
>           --privileged 52e bash
>   ```
></details>

1. Open `QGroundControl` and head to Application Settings->Comm Links

2. Create a new Comm Link of `UDP` type and use the server the appropriate server address and `udp_gcs_port_local` port\
![New Comm Link](image.png)

3. Start the simulator and Gazebo
```shell
cd $SARAX_WS/PX4-Autopilot/ && ./sarax_plus_sitl.bash
```

4. In another terminal, start another Docker container terminal
```shell
docker exec -it sarax_container_2 bash
```

5. Launch the Sarax framework
```shell
roslaunch m4e_mani_base sarax_plus_sitl.launch
```

6. Connect to the new `Docker Container` comm link in QGroundControl