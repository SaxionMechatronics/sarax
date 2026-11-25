"""Frame conventions for SARAX+.

Inside ROS / Pinocchio / the planner everything is REP-103:
    world  = ENU  (x East, y North, z Up)
    body   = FLU  (x Forward, y Left, z Up)        -- same as the URDF base_link
PX4 uses:
    world  = NED  (x North, y East, z Down)        -- local position frame
    body   = FRD  (x Forward, y Right, z Down)
Convert ONLY at the PX4 boundary (MAVLink / px4_msgs), with the functions below.

Quaternions are numpy arrays in **xyzw** order (ROS / Pinocchio) unless a name
says ``wxyz`` (PX4 / MAVLink ATTITUDE_QUATERNION order: q1=w, q2=x, q3=y, q4=z).
"""

import math

import numpy as np

# Vector maps.  Both are symmetric involutions (their own inverse).
R_ENU_NED = np.array([[0.0, 1.0, 0.0],
                      [1.0, 0.0, 0.0],
                      [0.0, 0.0, -1.0]])
R_FLU_FRD = np.diag([1.0, -1.0, -1.0])


# ── vectors / positions ──────────────────────────────────────────────────────

def ned_to_enu(v):
    """Position/velocity/acceleration/direction from NED to ENU (shape (...,3))."""
    v = np.asarray(v, dtype=float)
    return v @ R_ENU_NED.T


def enu_to_ned(v):
    """Position/velocity/acceleration/direction from ENU to NED (shape (...,3))."""
    return ned_to_enu(v)


def frd_to_flu(v):
    """Body-frame vector from FRD to FLU (shape (...,3))."""
    v = np.asarray(v, dtype=float)
    return v @ R_FLU_FRD.T


def flu_to_frd(v):
    """Body-frame vector from FLU to FRD (shape (...,3))."""
    return frd_to_flu(v)


# ── rotations / quaternions ──────────────────────────────────────────────────

def quat_xyzw_to_rot(q):
    """Rotation matrix from a unit quaternion [x, y, z, w]."""
    x, y, z, w = (float(c) for c in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def rot_to_quat_xyzw(R):
    """Unit quaternion [x, y, z, w] (w >= 0) from a proper rotation matrix."""
    R = np.asarray(R, dtype=float)
    tr = float(np.trace(R))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    return -q if q[3] < 0.0 else q


def rot_ned_frd_to_enu_flu(R_ned_frd):
    """Body attitude: R (FRD→NED) to R (FLU→ENU)."""
    return R_ENU_NED @ np.asarray(R_ned_frd, dtype=float) @ R_FLU_FRD


def rot_enu_flu_to_ned_frd(R_enu_flu):
    """Body attitude: R (FLU→ENU) to R (FRD→NED)."""
    return R_ENU_NED @ np.asarray(R_enu_flu, dtype=float) @ R_FLU_FRD


def px4_quat_wxyz_to_enu_flu_xyzw(q_wxyz):
    """PX4 attitude quaternion [w,x,y,z] (FRD→NED) to ROS [x,y,z,w] (FLU→ENU)."""
    w, x, y, z = (float(c) for c in q_wxyz)
    R = rot_ned_frd_to_enu_flu(quat_xyzw_to_rot([x, y, z, w]))
    return rot_to_quat_xyzw(R)


def enu_flu_xyzw_to_px4_quat_wxyz(q_xyzw):
    """ROS [x,y,z,w] (FLU→ENU) to PX4 attitude quaternion [w,x,y,z] (FRD→NED)."""
    x, y, z, w = rot_to_quat_xyzw(rot_enu_flu_to_ned_frd(quat_xyzw_to_rot(q_xyzw)))
    return np.array([w, x, y, z])


# ── yaw ──────────────────────────────────────────────────────────────────────

def wrap_angle(a):
    """Wrap an angle to [-pi, pi)."""
    return (float(a) + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat_xyzw(q):
    """Yaw (rotation about the world z axis) of a quaternion [x,y,z,w]."""
    x, y, z, w = (float(c) for c in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_ned_to_enu(yaw_ned):
    """PX4 heading (from North, clockwise) to ENU yaw (from East, counter-clockwise)."""
    return wrap_angle(math.pi / 2.0 - float(yaw_ned))


def yaw_enu_to_ned(yaw_enu):
    """ENU yaw (from East, counter-clockwise) to PX4 heading (from North, clockwise)."""
    return wrap_angle(math.pi / 2.0 - float(yaw_enu))


def quat_xyzw_from_yaw(yaw):
    """Quaternion [x,y,z,w] of a pure rotation about z."""
    return np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])
