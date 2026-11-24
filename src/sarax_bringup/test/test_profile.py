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
