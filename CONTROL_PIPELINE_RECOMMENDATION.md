# Recommendation: Migrate from raw `ros_gz_bridge` to `gz_ros2_control`

## Context

The initial POC (`sarax_torque_poc`) uses three `ros_gz_bridge` topics:
- ROS → Gazebo: `std_msgs/Float64` for each joint's `cmd_force`
- Gazebo → ROS: `sensor_msgs/JointState` from the `JointStatePublisher` plugin

This works, but it does not scale. Each new sensor, command, or controller adds more bridge plumbing; swapping to real hardware means rewriting the whole pipeline.

## The cleaner alternative

`gz_ros2_control` replaces the bridges with a single Gazebo world-plugin that spawns a full `controller_manager` inside the simulation process. `controller_manager` speaks native ROS 2, so there is no bridge between the controller and the rest of the ROS 2 graph.

### Side-by-side

| Aspect                         | Current POC                                           | `gz_ros2_control` version                          |
|--------------------------------|-------------------------------------------------------|----------------------------------------------------|
| Joint command                  | `ApplyJointForce` plugin + ROS→gz bridge per joint    | `<ros2_control>` block in URDF + `gz_ros2_control-system` plugin in SDF |
| Joint state                    | `JointStatePublisher` plugin + gz→ROS bridge          | `state_interface` claimed by `joint_state_broadcaster`, native ROS `/joint_states` |
| Bridges running                | N bridge topics (grows with joints)                   | Zero bridges for the manipulator                   |
| Sending a torque               | Publish a per-joint Float64 topic                     | Publish one `Float64MultiArray` to a `ForwardCommandController`, or command a trajectory, or load a custom impedance controller |
| Trying a new controller        | Write a new node that re-invents the plumbing         | Add the controller type to `controllers.yaml`, spawn it |
| Real-hardware swap             | Rewrite the whole pipeline                            | Swap one `<hardware>` tag in URDF; controller unchanged |
| Works with PX4-SITL?           | Yes                                                   | Yes — `gz_ros2_control` is a world-plugin, independent of PX4's motor plugins |

### Why this matters for the impedance controller goal

Every controller variant the plan anticipates (PD+gravity, PD+full inverse dynamics, apparent-inertia shaping with torque sensing) shares the same infrastructure: claim `effort` command interfaces, claim `position`/`velocity` state interfaces, run a math function at a fixed rate. `ros2_control` is exactly that framework. Building on top of the current bridge-based pipeline means rebuilding that infrastructure by hand for each controller.

## Migration steps

1. **Install**: `sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gz-ros2-control`
2. **New `sarax_description` package** with a cleaned `sarax_plus.urdf.xacro`:
   - Mesh paths fixed (current URDF has invalid `" OR "` alternatives)
   - ROS 1 `<transmission>` tags replaced with a `<ros2_control>` block declaring `effort` command + `position`/`velocity` state on `mani_joint_1`/`mani_joint_2`
   - `<hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>` for sim; swappable for real hardware later
3. **SDF change**: remove the `ApplyJointForce` plugins + the manipulator's `JointStatePublisher` plugin. Add one `gz_ros2_control-system` plugin pointing to a `controllers.yaml`. Leave PX4's `MulticopterMotorModel` plugins untouched — they are PX4's side of the simulation, independent of ros2_control.
4. **New `sarax_bringup` launch** that starts `robot_state_publisher` with the xacro before Gazebo, then spawns `joint_state_broadcaster` plus whichever command controller you want (`ForwardCommandController` for raw torques during bring-up; the custom `JointImpedanceController` once it exists).
5. **Torque POC after migration**: publish `std_msgs/Float64MultiArray` to `/forward_effort_controller/commands`. The old per-joint bridge code is deleted.

Once that's in place, the eventual `JointImpedanceController` is just another controller type in `controllers.yaml` — no plumbing changes.

## Runtime decoupling from PX4

Nothing about this migration couples the manipulator stack to PX4. `gz_ros2_control`, PX4-SITL, and the multicopter motor plugins are three independent components that all happen to share the same Gazebo world. This preserves the "no PX4 coupling" rule documented in the master plan.

## Bottom line

Do the migration before writing the impedance controller. Everything built on top of `ros2_control` is reusable across controller variants and across sim→hardware; everything built on top of the bridge approach is single-use scaffolding.
