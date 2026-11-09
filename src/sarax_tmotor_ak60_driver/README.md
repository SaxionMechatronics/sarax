# SARAX AK60 hardware driver

This package connects real AK60 motors to the existing
`sarax_impedance_controller/JointImpedanceController` through ros2_control.
It has no separate robot description, controller configuration, or launch file.

The hardware path is:

```text
Planner JointTrajectoryPoint -> /joint_impedance_controller/reference
  -> existing SARAX impedance controller -> joint effort interfaces
  -> TmotorAk60System -> SocketCAN -> physical motors
  <- measured position/velocity/effort <- CAN replies
```

## Files and configuration

| File | Purpose |
| --- | --- |
| `src/ak60_protocol.cpp` and its header | Motor ID, effort limit, MIT packet encoding/decoding |
| `src/tmotor_ak60_system.cpp` and its header | One CAN socket, actual joint states, ROS lifecycle and effort commands |
| `sarax_hardware.xml` | Registers the C++ hardware plugin with ROS |
| `CMakeLists.txt`, `package.xml` | Build and install the plugin and its dependencies |
| `test/test_ak60.cpp` | Offline packet/timeout regression tests; never part of runtime control |

System configuration lives in the existing packages:

- `sarax_description/urdf/sarax_plus.urdf.xacro`: robot geometry and motor mappings.
- `sarax_description/urdf/ros2_control.xacro`: real driver and joint interfaces.
- `sarax_bringup/config/controllers.yaml`: existing impedance-controller configuration.
- `sarax_bringup/launch/ak60_hardware.launch.py`: physical bringup using those files.

The hardware launch selects one motor by default: `mani_joint_1` -> motor ID 2.
`use_second_motor:=true` adds `mani_joint_2`; set `motor_2_id` to its verified ID
(the default 1 is only an example). No state is generated for an absent motor.
The full robot geometry remains available; transforms for unmeasured movable
joints require their actual feedback.

The launch uses the existing `PDLaw` for the currently unloaded motor. It sets
stiffness 10, damping 1, maximum position error 0.2 rad and controller/hardware
effort limits +/-5.0. Override these through launch arguments. These settings still
need physical validation. Select the existing `PDGravityLaw` only when the real
arm assembly, joint mapping and units match its URDF model. Neither law's source
code is changed. The simulation configuration is unchanged.

## Running the real system

From the workspace root:

```bash
pixi shell
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/.pixi/envs/default/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ros2 launch sarax_bringup ak60_hardware.launch.py
```

This loads the actual driver, robot_state_publisher, joint_state_broadcaster and
existing joint_impedance_controller. Hardware starts **unconfigured** and both
controllers **inactive**. The default `hardware_verified=false` blocks opening
CAN. Launching publishes no reference points, fabricated joint states or motor
commands. There is no Gazebo, mock hardware, joint-state GUI or effort publisher
in this launch.

After verifying the physical setup, restart with `hardware_verified:=true`.
The interface must already be configured; this package does not configure the
adapter. The detected PEAK PCAN-USB FD uses `peak_usb`; classic CAN at 1 Mbit/s
was used on `can0`.

The following activation sequence sends real motor commands; the existing
startup-motion issue described below must be resolved before another run:

```bash
ros2 control set_hardware_component_state SaraxManipulator inactive -c /controller_manager
ros2 control switch_controllers --activate joint_state_broadcaster -c /controller_manager
ros2 control set_hardware_component_state SaraxManipulator active -c /controller_manager
ros2 topic echo /joint_states
```

Only after fresh finite feedback is stable, activate the impedance controller:

```bash
ros2 control switch_controllers --activate joint_impedance_controller -c /controller_manager
```

It captures the measured position as its initial reference. The input remains
`/joint_impedance_controller/reference`, type
`trajectory_msgs/msg/JointTrajectoryPoint`: one position/velocity for one motor,
two in `[mani_joint_1, mani_joint_2]` order for two motors. Positions are absolute
radians. The existing controller uses the latest point and does not interpolate
`time_from_start`. Future planning nodes should provide the reference stream;
no reference generator is started by hardware bringup. MLS, filtering and motion
planning from the other workspace have not been copied or changed.

To stop and close CAN:

```bash
ros2 control switch_controllers --deactivate joint_impedance_controller joint_state_broadcaster -c /controller_manager
ros2 control set_hardware_component_state SaraxManipulator inactive -c /controller_manager
ros2 control set_hardware_component_state SaraxManipulator unconfigured -c /controller_manager
```

## Credits and license

ROS 2 hardware-interface port contributed by Waleed Elfieky.

Based on the original ROS 1 `tmotor_ak60_drivers_cpp` package from
the SMART Research Group at Saxion University of Applied Sciences:
https://github.com/SaxionMechatronics/sarax

This package is distributed under the BSD 3-Clause license.
Original copyright and license notices are retained.