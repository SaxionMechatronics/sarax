"""The hardware launcher: arguments, argument checks and the CAN step of scripts/hardware.sh."""
import importlib.util
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[3]
LAUNCH = Path(__file__).resolve().parents[1] / "launch"


def load(name):
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), LAUNCH / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def declared(description):
    from launch.actions import DeclareLaunchArgument
    return {a.name: a for a in description.entities if isinstance(a, DeclareLaunchArgument)}


def test_hardware_launch_declares_its_arguments():
    args = declared(load("hardware.launch.py").generate_launch_description())
    assert {"hardware", "can_interface", "hardware_verified", "agent", "agent_args",
            "record", "extra_topics", "bag_dir"} <= set(args)


def test_ak60_launch_selects_mock_or_real():
    args = declared(load("ak60_hardware.launch.py").generate_launch_description())
    assert "hardware" in args


def test_mock_hardware_needs_no_can_interface_but_real_does(monkeypatch):
    from launch import LaunchContext
    from launch.actions import SetLaunchConfiguration
    module = load("ak60_hardware.launch.py")
    description = module.generate_launch_description()
    context = LaunchContext()
    for name, arg in declared(description).items():
        SetLaunchConfiguration(name, arg.default_value[0].perform(context)).visit(context)
    SetLaunchConfiguration("can_interface", "").visit(context)
    SetLaunchConfiguration("hardware", "real").visit(context)
    with pytest.raises(ValueError, match="can_interface is empty"):
        module.launch_hardware(context)
    SetLaunchConfiguration("hardware", "bogus").visit(context)
    with pytest.raises(ValueError, match="real or mock"):
        module.launch_hardware(context)
    spawner_args = lambda actions: actions[-1]._Node__arguments  # noqa: E731
    SetLaunchConfiguration("hardware", "mock").visit(context)
    mock = module.launch_hardware(context)  # robot_state_publisher, controller_manager, spawner
    assert len(mock) == 3 and "--inactive" not in spawner_args(mock)
    SetLaunchConfiguration("can_interface", "can_test").visit(context)
    SetLaunchConfiguration("hardware", "real").visit(context)
    real = module.launch_hardware(context)
    assert len(real) == 3 and "--inactive" in spawner_args(real)


def run_script(*args, env=None):
    import os
    environment = {**os.environ, "SARAX_TARGET": "hw", **(env or {})}
    return subprocess.run(["bash", str(ROOT / "scripts/hardware.sh"), *args],
                          capture_output=True, text=True, timeout=60, env=environment)


def has_can_adapter():
    import json
    out = subprocess.run(["ip", "-details", "-json", "link", "show", "type", "can"],
                         capture_output=True, text=True)
    # Same test as scripts/can_setup.sh: some `ip` versions list non-CAN links as empty objects.
    return any(link.get("linkinfo", {}).get("info_kind") == "can"
               for link in json.loads(out.stdout or "[]"))


def test_script_rejects_unknown_hardware():
    result = run_script("hardware:=nonsense")
    assert result.returncode != 0 and "real or mock" in result.stderr


def test_script_rejects_sitl_target():
    result = run_script("hardware:=mock", env={"SARAX_TARGET": "sitl"})
    assert result.returncode != 0 and "SARAX_TARGET=hw" in result.stderr


@pytest.mark.skipif(has_can_adapter(), reason="a CAN adapter is present")
def test_no_adapter_fails_clearly_before_anything_starts():
    result = run_script("hardware:=real")
    assert result.returncode != 0
    assert "No CAN adapter found" in result.stderr
    assert "nothing was started" in result.stderr
    assert "Launching hardware bringup" not in result.stdout
