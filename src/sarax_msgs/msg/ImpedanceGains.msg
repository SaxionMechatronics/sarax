# Live update of per-joint impedance gains for sarax_impedance_controller.
# The joint_names field must match the controller's `joints` parameter, in order.
# stiffness and damping must be the same length as joint_names.

std_msgs/Header header
string[]  joint_names
float64[] stiffness    # Kp, N.m / rad
float64[] damping      # Kd, N.m.s / rad
