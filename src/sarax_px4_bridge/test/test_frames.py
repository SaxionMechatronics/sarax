import math

import numpy as np
import pytest

from sarax_px4_bridge import frames as F


def _rand_rot(rng):
    q = rng.normal(size=4)
    return F.quat_xyzw_to_rot(q / np.linalg.norm(q))


def _rot_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def test_vector_maps_are_involutions():
    v = np.array([1.0, 2.0, 3.0])
    assert np.allclose(F.ned_to_enu(v), [2.0, 1.0, -3.0])
    assert np.allclose(F.enu_to_ned(F.ned_to_enu(v)), v)
    assert np.allclose(F.frd_to_flu(v), [1.0, -2.0, -3.0])
    assert np.allclose(F.flu_to_frd(F.frd_to_flu(v)), v)
    # works on stacks of vectors too
    vs = np.stack([v, 2 * v])
    assert F.ned_to_enu(vs).shape == (2, 3)


def test_quat_rot_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(100):
        R = _rand_rot(rng)
        assert np.allclose(F.quat_xyzw_to_rot(F.rot_to_quat_xyzw(R)), R, atol=1e-12)


def test_attitude_map_commutes_with_vector_maps():
    """R_enu_flu applied to a FLU vector == NED rotation applied to the FRD vector, then mapped."""
    rng = np.random.default_rng(1)
    for _ in range(200):
        R_ned_frd = _rand_rot(rng)
        v_frd = rng.normal(size=3)
        R_enu_flu = F.rot_ned_frd_to_enu_flu(R_ned_frd)
        assert np.isclose(np.linalg.det(R_enu_flu), 1.0)
        assert np.allclose(R_enu_flu @ F.frd_to_flu(v_frd), F.ned_to_enu(R_ned_frd @ v_frd))
        assert np.allclose(F.rot_enu_flu_to_ned_frd(R_enu_flu), R_ned_frd)


def test_px4_quaternion_roundtrip():
    rng = np.random.default_rng(2)
    for _ in range(100):
        q = rng.normal(size=4)
        q_wxyz = q / np.linalg.norm(q)
        q_xyzw = F.px4_quat_wxyz_to_enu_flu_xyzw(q_wxyz)
        back = F.enu_flu_xyzw_to_px4_quat_wxyz(q_xyzw)
        assert np.isclose(abs(back @ q_wxyz), 1.0)  # same rotation (q ~ -q)


@pytest.mark.parametrize("yaw_ned_deg, yaw_enu_deg", [
    (0.0, 90.0),     # facing North
    (90.0, 0.0),     # facing East
    (-90.0, 180.0),  # facing West (the blade scan heading in SITL)
    (180.0, -90.0),  # facing South
])
def test_level_heading(yaw_ned_deg, yaw_enu_deg):
    yaw_ned = math.radians(yaw_ned_deg)
    q_ned = np.array([math.cos(yaw_ned / 2), 0.0, 0.0, math.sin(yaw_ned / 2)])  # wxyz
    q_enu = F.px4_quat_wxyz_to_enu_flu_xyzw(q_ned)
    got = math.degrees(F.yaw_from_quat_xyzw(q_enu))
    assert math.isclose(F.wrap_angle(math.radians(got - yaw_enu_deg)), 0.0, abs_tol=1e-9)
    assert math.isclose(F.wrap_angle(F.yaw_ned_to_enu(yaw_ned) - math.radians(yaw_enu_deg)), 0.0,
                        abs_tol=1e-9)
    assert math.isclose(F.wrap_angle(F.yaw_enu_to_ned(F.yaw_ned_to_enu(yaw_ned)) - yaw_ned), 0.0,
                        abs_tol=1e-9)


def test_tilt_signs():
    """Nose-up pitch and right-wing-down roll keep their physical meaning.

    The pre-Phase-1 planner used (N, E, Up) with the PX4 quaternion unchanged,
    which flips the sign of roll and pitch; this test pins the correct mapping.
    """
    # PX4: heading North, pitch +10 deg (nose up in FRD/NED), roll +5 deg (right wing down)
    R_ned_frd = _rot_from_rpy(math.radians(5.0), math.radians(10.0), 0.0)
    R_enu_flu = F.rot_ned_frd_to_enu_flu(R_ned_frd)
    nose_enu = R_enu_flu @ np.array([1.0, 0.0, 0.0])   # body forward
    left_enu = R_enu_flu @ np.array([0.0, 1.0, 0.0])   # body left
    assert nose_enu[1] > 0.9          # still points North (ENU y)
    assert nose_enu[2] > 0.0          # nose up
    assert left_enu[2] > 0.0          # right wing down => left wing up
    # The old convention (z flip only, same matrix) gets the nose pitch wrong:
    R_old = R_ned_frd                  # used directly in an (N, E, Up) world
    nose_old_up = (R_old @ np.array([1.0, 0.0, 0.0]))[2]   # its z read as "Up"
    assert nose_old_up < 0.0          # i.e. it showed nose DOWN for a nose-up vehicle
