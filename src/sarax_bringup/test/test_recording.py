from datetime import datetime
from pathlib import Path
import re

import pytest

from sarax_bringup import recording


def test_extra_topics_whitespace_and_commas():
    assert recording.parse_extra_topics("") == []
    assert recording.parse_extra_topics("/a /b") == ["/a", "/b"]
    assert recording.parse_extra_topics(" /a,/b\n/c ") == ["/a", "/b", "/c"]


def test_extra_topic_must_be_absolute():
    with pytest.raises(ValueError, match="absolute"):
        recording.parse_extra_topics("/a relative")


def test_default_bag_dir_is_timestamped(monkeypatch):
    monkeypatch.delenv("SARAX_BAG_DIR", raising=False)
    now = datetime(2026, 11, 25, 9, 5, 7)
    assert recording.default_bag_dir(now=now) == Path("bags/flight_2026-11-25_09-05-07")
    monkeypatch.setenv("SARAX_BAG_DIR", "/data/bags")
    assert recording.default_bag_dir(now=now) == Path("/data/bags/flight_2026-11-25_09-05-07")
    assert recording.default_bag_dir("x", now=now) == Path("x/flight_2026-11-25_09-05-07")


def test_platform_list_is_what_the_ticket_names():
    assert "/joint_states" in recording.PLATFORM_TOPICS
    assert "/joint_impedance_controller/reference_active" in recording.PLATFORM_TOPICS
    assert "/joint_impedance_controller/commanded_torque" in recording.PLATFORM_TOPICS
    assert recording.PLATFORM_REGEXES == (r"/fmu/out/.*",)


def test_record_command_is_one_regex():
    command = recording.record_command("/tmp/bag")
    assert command[:6] == ["ros2", "bag", "record", "-o", "/tmp/bag", "--regex"]
    assert len(command) == 7  # no positional topics: Humble would intersect them with the regex


def matches(command, topic):
    return re.fullmatch(command[-1], topic) is not None


def test_platform_pattern_matches_the_platform_and_nothing_else():
    command = recording.record_command("b")
    for topic in (*recording.PLATFORM_TOPICS, "/fmu/out/vehicle_odometry", "/fmu/out/x/y"):
        assert matches(command, topic), topic
    for topic in ("/fmu/in/trajectory_setpoint", "/joint_states2", "/x/joint_states",
                  "/joint_impedance_controller/reference", "/sarax/odom"):
        assert not matches(command, topic), topic


def test_record_command_adds_extras():
    command = recording.record_command(
        "b", ["/inspection_target", "/joint_states", "/inspection_target"], [r"/operator/.*"])
    assert matches(command, "/inspection_target")
    assert matches(command, "/operator/status")
    assert matches(command, "/fmu/out/vehicle_status")
    assert not matches(command, "/inspection_target_x")
    assert command[-1].count("joint_states") == 1


def test_extra_topic_is_matched_literally():
    command = recording.record_command("b", ["/a.b"])
    assert matches(command, "/a.b") and not matches(command, "/axb")
