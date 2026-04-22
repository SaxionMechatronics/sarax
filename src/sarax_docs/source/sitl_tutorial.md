# SITL Tutorial

## Manipulator Only (No Drone)

Best for controller development — no PX4 required.

```bash
ros2 launch sarax_bringup manipulator_only.launch.py
ros2 launch sarax_bringup send_step_reference.launch.py q1:=0.5 q2:=-0.3
```

## Full SITL (PX4 + Manipulator)

Requires PX4-Autopilot built with the `sarax_plus` airframe.

```bash
# Terminal 1 – Gazebo + ROS 2 side
ros2 launch sarax_bringup sitl.launch.py px4:=true

# Terminal 2 – QGroundControl (optional, for arming/takeoff)
QGroundControl &

# Terminal 3 – manipulator step after drone is hovering
ros2 launch sarax_bringup send_step_reference.launch.py q1:=0.3 q2:=-0.4
```

## Monitoring

```bash
# Joint states
ros2 topic echo /joint_states

# Controller torque output
ros2 topic echo /joint_impedance_controller/state

# RViz (robot model + TF)
ros2 launch sarax_description view_robot.launch.py
```
