"""The URDF joint limits are the single source of the arm torque limit; the SDF must agree."""
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

SRC = Path(__file__).resolve().parents[2]
URDF = SRC / "sarax_description/urdf/sarax_plus.urdf.xacro"
SDF = SRC / "sarax_gz_sim/models/sarax_plus/model.sdf"


def _expand(*args):
    out = subprocess.run(["xacro", str(URDF), *args], check=True, capture_output=True, text=True)
    return ET.fromstring(out.stdout)


def _limit(robot, joint):
    return robot.find(f"joint[@name='{joint}']/limit").attrib


def test_one_torque_limit_for_every_hardware_flavour():
    for hardware in ("gazebo", "mock", "real"):
        robot = _expand(f"hardware:={hardware}")
        for joint in ("mani_joint_1", "mani_joint_2"):
            assert float(_limit(robot, joint)["effort"]) == 5.0
        for param in robot.iter("param"):
            if param.get("name") in ("min", "max"):
                assert abs(float(param.text)) == 5.0
            if param.get("name") == "effort_limit":
                assert float(param.text) == 5.0


def test_effort_limit_arg_is_the_only_knob():
    robot = _expand("effort_limit:=3.0", "hardware:=real")
    assert float(_limit(robot, "mani_joint_2")["effort"]) == 3.0
    assert all(abs(float(p.text)) == 3.0 for p in robot.iter("param")
               if p.get("name") in ("min", "max", "effort_limit"))


def test_second_joint_has_ros1_limits():
    lim = _limit(_expand(), "mani_joint_2")
    assert (float(lim["lower"]), float(lim["upper"]), float(lim["velocity"])) == (-1.9, 2.6, 1.0)


def test_sdf_matches_urdf_limits():
    sdf = ET.parse(SDF).getroot()
    urdf = _expand()
    for joint in ("mani_joint_1", "mani_joint_2"):
        node = sdf.find(f".//joint[@name='{joint}']/axis/limit")
        got = {k: float(node.find(k).text) for k in ("lower", "upper", "effort", "velocity")}
        want = {k: float(v) for k, v in _limit(urdf, joint).items()}
        assert got == want
