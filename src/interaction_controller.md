# interaction_controller Package
#### Author: Ayham Alharbat <a.n.alharbat@gmail.com>

## Introduction

This package is a control package for UAV off-board control with PX4 and ROS, and for PX4 software-In-The-Loop simulation in Gazebo.

The controller is a geometric position controller on SE(3), where the position controller is based on the trajectory tracking controller from [1], and the attitude controller is based on the geometric spring-damper presented in [2] and implemented in [3].

The general control architecture is illustrated in the figure below, the force controller is not yet implemented.

![controller_block_diagram](https://user-images.githubusercontent.com/44285656/104025351-c7a74480-51c4-11eb-9f37-4fbad9c7a000.png)

[1] Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.

[2] Stramigioli, Stefano. Modeling and IPC control of interactive mechanical systems—A coordinate-free approach. 2001.

[3] Rashad, Ramy, Federico Califano, and Stefano Stramigioli. "Port-Hamiltonian passivity-based control on SE (3) of a fully actuated UAV for aerial physical interaction near-hovering." IEEE Robotics and automation letters 4.4 (2019): 4378-4385.

This package is developed and tested in:
- Ubuntu 18.04 LTS
- ROS melodic
- Gazebo 9
- PX4 Autopilot 1.11.1
- Mavros 1.4.0

## SITL quick test

1- First run:

  `roslaunch px4 mavros_posix_sitl.launch vehicle:="iris"`
  
Gazebo should open with a model of the quadcopter `iris`.

2- then:

  `roslaunch interaction_controller start.launch vehicle:="iris" sitl:="true"`

3- Publish a position command at `/command/pose`, for quick test, run:

```
rostopic pub /command/pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 1.0
    y: 1.0
    z: 2.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"
```

## How to configure the controller for another UAV

Let's call your UAV `cool_name`

1- Create a yaml file named `cool_name_param.yaml` file in `config/uav_parameters`

2- The file should have the following format and information:

https://github.com/aihamalharbat/interaction_controller/blob/cfe7c6d2776211bdd5dc694ad8dcdf4435c2a2e9/config/uav_parameters/iris_param.yaml#L1-L12

3- Create a yaml file named `initial_gains_cool_name.yaml` file in `config/controller`

4- The file should have the following format and gains:
https://github.com/aihamalharbat/interaction_controller/blob/cfe7c6d2776211bdd5dc694ad8dcdf4435c2a2e9/config/controller/initial_gains_iris.yaml#L1-L18

The gains can be initially based on `iris` or `peach` gains, they can be tuned during flight using dynamic reconfiguration.
Description about these gains is available in:
https://github.com/aihamalharbat/interaction_controller/blob/cfe7c6d2776211bdd5dc694ad8dcdf4435c2a2e9/config/dynamic/parameters.cfg#L8-L25

## Launch options

You can pass three arguments at `roslaunch`:

`roslaunch interaction_controller start.launch vehicle:="peach" dyn_rec:="true"  sitl:="true"`

1- `vehicle`: The name of UAV, so that the UAV parameters and initial gains are loaded from yaml files

2- `dyn_rec`: If true the dynamic reconfiguration window will open at launch, it is used to tune the gains during fligt

3- `sitl`: if true the sitl parameters will be loaded from `config/sitl/sitl_params.yaml`
https://github.com/aihamalharbat/interaction_controller/blob/cfe7c6d2776211bdd5dc694ad8dcdf4435c2a2e9/config/sitl/sitl_params.yaml#L1-L12

Make sure to update the topics and services names if incorrect.

If `sitl` is false the experimental parameters will be loaded from `interaction_controller/config/exp/exp_params.yaml `
https://github.com/aihamalharbat/interaction_controller/blob/cfe7c6d2776211bdd5dc694ad8dcdf4435c2a2e9/config/exp/exp_params.yaml#L1-L12

Also, make sure to update the topics and services names if incorrect.

## Visualizaion tools

- RViz configuration file in `visuals/rviz/` can used to visualize:

  1- The UAV position and orientation with respect to the inertial (world) frame. The package broadcasts the tf transform from the inertial frame [map] to the body-fixed frame [base-link].
  
  2- The goal position as published in `/command/pose`. P.S.: The command needs to have timeStamp and frame_id to be visualized in RViz.
  
  3- The desired rotation: which is the output of the position controller as illustrated in the controller architecture above. The desired rotation frame have the same origin of the body-fixed frame but its orientation is based on the desired rotation matrix.

- `rqt_multiplot` configuration files in `visuals/rqt_multiplot/` can be used to plot:

  1- The controller outputs `ActCmds.xml`: The normalized thrust and three normalized moments.
  
  2- The controller 3D forces `debug_Forces.xml`: The controller 3D forces, used to debug. 
  
  3- The controller debugging plots `debug_plot_with_SpringDamper.xml`: The full debugging plots.
  
  4- The UAV odometry in experimental tests `hyb_odom_all.xml `: The full odometry of the UAV.

The topics names might need to be updated for your system configuration.

---

# Installation

After installing mavros and mavlink following [this poge](https://docs.px4.io/v1.12/en/ros/mavros_installation.html#binary-installation-debian-ubuntu).

1. Donwload the package, then move the source code to the catkin workspace src directory.
2. Install the dependencies from rosdep

        rosdep install --from-paths src --ignore-src -r -y

3. Install other dependencies:

        sudo apt-get install ros-melodic-mav-msgs
        sudo apt-get install ros-melodic-vrpn-client-ros
        
      
---

# General points to get things working
1. Make sure that you have added the user to group dialout **and then reboot**, so that mavros can communicate with PX4 through USB:

        sudo gpasswd -a $USER dialout
2. 


---

# Running an experimental test

** FOLLOW SAFETY GUIDLINES: **

## Chick-list
1. Check that you are connect to WIFI “Lectoraat Mechatronica” for the Optitrack data

2. Check that Z UP

## Instructions
1. In one terminal run:
        roslaunch mavros px4.launch
        
2. In another terminal run:
        roslaunch interaction_controller peach_exp.launch
      
3. The controller will arm the robot and go to offboard

4. Start sending the trajectory for taking off