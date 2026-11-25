"""One bag per flight: the platform topic list and the ``ros2 bag record`` command.

The hardware bringup (``hardware.launch.py``) owns the recording. The platform records what
it produces; applications built on it (the vision-repo's target, plan and operator status)
add their topics through the ``extra_topics`` launch argument, so a flight gives one bag.
The PX4 ULog stays on the flight controller.
"""
from datetime import datetime
import os
from pathlib import Path
import re
import shlex

# Exact topics (matched literally). ``~/reference_active`` is the controller's latched flag that tells whether
# a reference (or a fold/extend ramp) currently drives the arm; ``commanded_torque`` is its
# output, the controller state besides the joint states.
PLATFORM_TOPICS = (
    "/joint_states",
    "/joint_impedance_controller/commanded_torque",
    "/joint_impedance_controller/reference_active",
)

# Every PX4 output topic of the XRCE agent. A regex, because the set depends on the
# PX4 version and the uxrce_dds_client configuration.
PLATFORM_REGEXES = (r"/fmu/out/.*",)


def parse_extra_topics(text):
    """Split the ``extra_topics`` launch argument (whitespace or comma separated) into topics.

    Entries must be absolute topic names; ``'/a /b'`` and ``'/a,/b'`` are the same. Empty text
    gives an empty list; duplicates and platform topics are dropped by ``record_command``.
    """
    topics = []
    for token in text.replace(",", " ").split():
        if not token.startswith("/"):
            raise ValueError(f"extra topic {token!r} must be an absolute name starting with '/'")
        topics.append(token)
    return topics


def default_bag_dir(base=None, now=None):
    """``<base>/flight_YYYY-MM-DD_HH-MM-SS``; ``base`` defaults to ``$SARAX_BAG_DIR`` or ``./bags``."""
    base = Path(base or os.environ.get("SARAX_BAG_DIR") or "bags")
    stamp = (now or datetime.now()).strftime("%Y-%m-%d_%H-%M-%S")
    return base / f"flight_{stamp}"


def record_pattern(extra_topics=(), extra_regexes=()):
    """One regular expression that matches the platform topics and the extras."""
    parts = [re.escape(t) for t in PLATFORM_TOPICS]
    parts += [r for r in PLATFORM_REGEXES]
    parts += [re.escape(t) for t in extra_topics]
    parts += list(extra_regexes)
    return "|".join(f"(?:{p})" for p in dict.fromkeys(parts))


def record_command(output, extra_topics=(), extra_regexes=()):
    """Argument list of ``ros2 bag record`` for the platform topics plus the extras.

    ``output`` is the bag directory (must not exist yet; rosbag2 refuses to overwrite).
    Everything goes into a single ``--regex``: in Humble a regex combined with positional
    topics selects only the topics that match both, so the two cannot be mixed. The regex also
    picks up topics whose publisher starts after the recorder.
    """
    return ["ros2", "bag", "record", "-o", str(output),
            "--regex", record_pattern(extra_topics, extra_regexes)]


def describe(command):
    """The command as one shell-quoted line for the launch log."""
    return " ".join(shlex.quote(str(part)) for part in command)
