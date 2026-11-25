"""Record with the real ``ros2 bag record`` command and check the bag's contents."""
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from std_msgs.msg import String  # noqa: E402

from sarax_bringup import recording  # noqa: E402

FMU_TOPICS = ["/fmu/out/vehicle_odometry", "/fmu/out/vehicle_status"]
NOT_RECORDED = ["/fmu/in/trajectory_setpoint", "/some/other"]


def bag_topics(bag):
    db = next(Path(bag).glob("*.db3"))
    with sqlite3.connect(db) as con:
        counts = dict(con.execute(
            "select t.name, count(m.id) from topics t left join messages m "
            "on m.topic_id = t.id group by t.name"))
    return counts


def test_bag_has_every_platform_topic_and_the_extras(tmp_path, monkeypatch):
    env = {**os.environ, "ROS_DOMAIN_ID": "79"}
    monkeypatch.setenv("ROS_DOMAIN_ID", "79")
    bag = tmp_path / "flight"
    extra = ["/inspection_target", "/operator/status"]
    command = recording.record_command(bag, extra)
    recorder = subprocess.Popen(command, env=env, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node("bag_test_publishers")
    names = [*recording.PLATFORM_TOPICS, *FMU_TOPICS, *extra, *NOT_RECORDED]
    publishers = {name: node.create_publisher(String, name, 10) for name in names}
    try:
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            for publisher in publishers.values():
                publisher.publish(String(data="x"))
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        os.killpg(recorder.pid, signal.SIGINT)
        recorder.wait(timeout=20)
        node.destroy_node()
        rclpy.shutdown()

    counts = bag_topics(bag)
    for topic in [*recording.PLATFORM_TOPICS, *FMU_TOPICS, *extra]:
        assert counts.get(topic, 0) > 0, f"{topic} missing from the bag: {counts}"
    for topic in NOT_RECORDED:
        assert topic not in counts
