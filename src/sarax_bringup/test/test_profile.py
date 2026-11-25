from pathlib import Path

import pytest

from sarax_bringup import profile


def test_unset_defaults_to_sitl(monkeypatch):
    monkeypatch.delenv("SARAX_TARGET", raising=False)
    assert profile.load()["target"] == "sitl"


def test_empty_defaults_to_sitl(monkeypatch):
    monkeypatch.setenv("SARAX_TARGET", "")
    assert profile.resolve_target() == "sitl"


def test_hw_selected(monkeypatch):
    monkeypatch.setenv("SARAX_TARGET", "hw")
    data = profile.load()
    assert profile.get(data, "motors.motor_2.id") == 3
    assert data["use_sim_time"] is False


def test_unknown_target_fails_loudly(monkeypatch):
    monkeypatch.setenv("SARAX_TARGET", "bench")
    with pytest.raises(profile.ProfileError, match="unknown SARAX_TARGET"):
        profile.load()
    assert profile.main(["target"]) == 1


def test_missing_key_names_target(monkeypatch):
    monkeypatch.setenv("SARAX_TARGET", "sitl")
    with pytest.raises(profile.ProfileError, match="'sitl'.*motors.motor_1.id"):
        profile.get(profile.load(), "motors.motor_1.id")


def test_px4_env_keeps_environment_override(monkeypatch, capsys):
    monkeypatch.setenv("SARAX_TARGET", "sitl")
    assert profile.main(["px4-env"]) == 0
    assert 'PX4_PARAM_NAV_DLL_ACT="${PX4_PARAM_NAV_DLL_ACT:-0}"' in capsys.readouterr().out


def test_hw_has_no_sitl_only_px4_params(monkeypatch, capsys):
    monkeypatch.setenv("SARAX_TARGET", "hw")
    assert profile.main(["px4-env"]) == 0
    assert capsys.readouterr().out == ""


def test_hardware_only_bringup_defaults_to_hw(monkeypatch):
    monkeypatch.delenv("SARAX_TARGET", raising=False)
    assert profile.load(default="hw")["target"] == "hw"
    monkeypatch.setenv("SARAX_TARGET", "sitl")
    assert profile.load(default="hw")["target"] == "sitl"


def test_sitl_arm_poses_match_controllers_yaml(monkeypatch):
    """The simulator loads controllers.yaml directly, so it must carry the sitl profile's poses."""
    import yaml
    monkeypatch.setenv("SARAX_TARGET", "sitl")
    poses = profile.get(profile.load(), "arm_poses")
    controllers = Path(profile.profile_dir()).parent / "controllers.yaml"
    params = yaml.safe_load(controllers.read_text())["joint_impedance_controller"]["ros__parameters"]
    for name, values in poses.items():
        assert params[f"pose_{name}"] == values


def test_hw_arm_poses_cover_stowed_and_extended():
    poses = profile.get(profile.load("hw"), "arm_poses")
    assert set(poses) == {"stowed", "extended"}
    assert all(len(values) == 2 for values in poses.values())
