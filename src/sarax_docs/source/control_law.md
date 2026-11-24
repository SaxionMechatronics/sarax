# Control Law

## Joint-Space Impedance (PD + Gravity Compensation)

For generalized coordinates $q = [q_1, q_2]^T$ and desired trajectory $q_d$:

$$\tau = -K_d(q - q_d) - D_d(\dot{q} - \dot{q}_d) + g(q)$$

| Symbol | Meaning |
|--------|---------|
| $K_d$  | Diagonal stiffness matrix (N·m/rad) |
| $D_d$  | Diagonal damping matrix (N·m·s/rad) |
| $g(q)$ | Gravity torque vector from the analytic SARAX arm model (`PDGravityLaw`) |

## Gravity Compensation

`PDGravityLaw` uses the original SARAX hardware gravity equations with the measured
motor angles:

$$g_1 = -g\,(m_2 d_2 + m_3 L_2)\sin q_1, \qquad g_2 = -g\,m_3 d_3 \sin q_2$$

The constants are the `gravity_*` controller parameters (`controllers.yaml`). They are
kept until a system-identification step has checked the URDF inertials against the real
arm; only then is the gravity term taken from the URDF. The manipulator base is assumed
upright relative to gravity, which holds near hover.

## Safety Limits

Torque commands are clamped to ±30 N·m per joint (from URDF effort limits). The
stiffness contribution is further saturated when $|q - q_d| >$ `max_position_error`
(default 0.5 rad) to prevent torque saturation on bad references.

## Reference Watchdog

A reference on `~/reference` must keep arriving. If none arrives for `reference_timeout`
seconds (default 0.5 s, `0` disables), the controller drops it and holds the current pose
($q_d \leftarrow q$, $\dot q_d \leftarrow 0$). This covers a planner or GUI that dies
mid-motion. References set through the `q_ref_N` parameters never expire.

`~/reference_active` (`std_msgs/Bool`, transient local) is `true` while a live reference is
tracked and `false` after activation or a timeout. `send_step_reference.launch.py` therefore
streams the step (`rate:=20` Hz by default) until Ctrl-C, after which the arm holds its pose.
