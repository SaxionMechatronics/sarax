# Getting Started

## Prerequisites

- ROS 2 Humble
- Gazebo Harmonic
- `ros-humble-ros2-control`, `ros-humble-gz-ros2-control`, `ros-humble-pinocchio`

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-gz-ros2-control ros-humble-pinocchio \
                 ros-humble-xacro ros-humble-generate-parameter-library
```

## Build

```bash
cd ~/wspaces/sarax_ws
colcon build --symlink-install
source install/setup.bash
```

## Quick Start (manipulator only, no drone)

```bash
ros2 launch sarax_bringup manipulator_only.launch.py
```

Send a step reference:

```bash
ros2 launch sarax_bringup send_step_reference.launch.py q1:=0.3 q2:=-0.4
```

Monitor joint states:

```bash
ros2 topic echo /joint_states
```
