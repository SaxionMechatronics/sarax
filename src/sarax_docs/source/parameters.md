# Parameter Reference

Parameters are declared via `generate_parameter_library` from
`sarax_impedance_controller/parameters.yaml`.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `joints` | `string[]` | `[mani_joint_1, mani_joint_2]` | Controlled joints |
| `stiffness` | `double[]` | `[200.0, 200.0]` | Per-joint $K_d$ (N·m/rad) |
| `damping` | `double[]` | `[20.0, 20.0]` | Per-joint $D_d$ (N·m·s/rad) |
| `effort_limits` | `double[]` | `[30.0, 30.0]` | Torque clamp (N·m) |
| `use_gravity_compensation` | `bool` | `true` | Enable Pinocchio $g(q)$ |
| `max_position_error` | `double` | `0.5` | Stiffness saturation threshold (rad) |
| `impedance_law_type` | `string` | `pd_gravity` | Plugin type: `pd` or `pd_gravity` |
| `root_link` | `string` | `base_link` | Pinocchio model root frame |

All parameters except `joints` support dynamic reconfiguration at runtime.

## Live Gain Update

```bash
ros2 param set /joint_impedance_controller stiffness "[300.0, 300.0]"
ros2 param set /joint_impedance_controller damping "[30.0, 30.0]"
```
