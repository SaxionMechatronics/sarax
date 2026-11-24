"""The URDF is the only definition of the camera pose; the Gazebo SDF must agree with it."""
import re
from pathlib import Path

SRC = Path(__file__).resolve().parents[2]
URDF = SRC / "sarax_description/urdf/sarax_plus.urdf.xacro"
SDF = SRC / "sarax_gz_sim/models/sarax_plus/model.sdf"

EXPECTED = [-0.15, 0.0, 0.05, 0.0, 0.0, 3.14159265]


def _floats(text):
    return [float(v) for v in text.split()]


def _urdf_pose():
    text = URDF.read_text()
    xyz = re.search(r'<xacro:arg name="camera_xyz" default="([^"]+)"', text).group(1)
    rpy = re.search(r'<xacro:arg name="camera_rpy" default="([^"]+)"', text).group(1)
    return _floats(xyz) + _floats(rpy)


def _sdf_pose():
    text = SDF.read_text()
    m = re.search(r"<link name='camera_link'>\s*<pose relative_to='base_link'>([^<]+)</pose>", text)
    return _floats(m.group(1))


def test_urdf_camera_pose_is_rear():
    assert _urdf_pose() == EXPECTED


def test_sdf_camera_matches_urdf():
    assert _sdf_pose() == _urdf_pose()
