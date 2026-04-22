# Troubleshooting

## Controller fails to load

Check the plugin XML is installed and the library is on `LD_LIBRARY_PATH`:

```bash
ros2 control list_controller_types | grep sarax
```

If missing, rebuild and re-source:

```bash
colcon build --packages-select sarax_impedance_controller
source install/setup.bash
```

## "robot_description is empty" at configure

The controller reads the URDF from the `robot_description` topic. Ensure
`robot_state_publisher` is running before activating the controller.

## Joints not tracking reference

1. Verify the controller is active: `ros2 control list_controllers`
2. Check gains are not zero: `ros2 param get /joint_impedance_controller stiffness`
3. Confirm effort interfaces are claimed: the hardware must declare `<command_interface name="effort"/>`

## Gravity compensation wrong direction

The base link orientation is assumed aligned with world gravity. If the drone is
pitching aggressively, the gravity term will be slightly off. Reduce stiffness or
increase damping to absorb the transient.

## Pinocchio model build failure

Ensure the URDF is valid:

```bash
check_urdf $(ros2 pkg prefix sarax_description)/share/sarax_description/urdf/sarax_plus.urdf
```
