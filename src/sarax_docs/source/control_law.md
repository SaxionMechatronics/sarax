# Control Law

## Joint-Space Impedance (PD + Gravity Compensation)

For generalized coordinates $q = [q_1, q_2]^T$ and desired trajectory $q_d$:

$$\tau = -K_d(q - q_d) - D_d(\dot{q} - \dot{q}_d) + g(q)$$

| Symbol | Meaning |
|--------|---------|
| $K_d$  | Diagonal stiffness matrix (N·m/rad) |
| $D_d$  | Diagonal damping matrix (N·m·s/rad) |
| $g(q)$ | Gravity torque vector from Pinocchio `computeGeneralizedGravity` |

## Gravity Compensation

The gravity term $g(q)$ is evaluated by Pinocchio at each control cycle using the
current joint positions. The manipulator base is assumed upright relative to gravity —
valid under PX4 hover; small corrections are absorbed by the $K_d$ term during attitude
transients.

## Safety Limits

Torque commands are clamped to ±30 N·m per joint (from URDF effort limits). The
stiffness contribution is further saturated when $|q - q_d| >$ `max_position_error`
(default 0.5 rad) to prevent torque saturation on bad references.
