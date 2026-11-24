"""Platform profiles: one YAML per target (``sitl``, ``hw``), picked by ``SARAX_TARGET``.

Platform values (motor IDs, CAN interface, SITL-only PX4 parameters, frame names,
simulation clock) live in ``config/profiles/<target>.yaml`` instead of in the
bringup scripts and launch files. An unset ``SARAX_TARGET`` means ``sitl``; any
other unknown value is an error.

Shell use (after sourcing the overlay)::

    python3 -m sarax_bringup.profile [--default hw] target   # resolved target name
    python3 -m sarax_bringup.profile get motors.1.id     # one value (dotted key)
    python3 -m sarax_bringup.profile px4-env             # export PX4_PARAM_<NAME> lines
"""
import os
from pathlib import Path
import sys

import yaml

ENV_VAR = "SARAX_TARGET"
DEFAULT_TARGET = "sitl"


class ProfileError(RuntimeError):
    """Unknown target, unreadable profile or missing key."""


def profile_dir():
    """Directory holding the profiles: installed share dir, else the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("sarax_bringup")) / "config" / "profiles"
    except Exception:  # not installed (running from a source checkout)
        return Path(__file__).resolve().parents[1] / "config" / "profiles"


def available_targets():
    return sorted(p.stem for p in profile_dir().glob("*.yaml"))


def resolve_target(target=None, default=DEFAULT_TARGET):
    """Name of the active target: argument, else ``$SARAX_TARGET``, else ``default``.

    Bringups that only make sense on one platform (the AK60 motors) pass their own
    ``default``; an explicit ``SARAX_TARGET`` of another platform still fails there.
    """
    if target is None:
        target = os.environ.get(ENV_VAR, "").strip() or default
    targets = available_targets()
    if target not in targets:
        raise ProfileError(
            f"unknown {ENV_VAR}={target!r}; available targets: {', '.join(targets) or '(none)'}")
    return target


def load(target=None, default=DEFAULT_TARGET):
    """Load the profile of ``target`` (default: from the environment) as a dict."""
    target = resolve_target(target, default)
    path = profile_dir() / f"{target}.yaml"
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ProfileError(f"{path}: profile must be a mapping")
    if data.get("target") != target:
        raise ProfileError(f"{path}: 'target' must be {target!r}, got {data.get('target')!r}")
    return data


def get(profile, key):
    """Value at a dotted ``key``; raises ProfileError naming the target if missing."""
    node = profile
    for part in key.split("."):
        if not isinstance(node, dict) or part not in node:
            raise ProfileError(f"profile {profile.get('target')!r} has no value '{key}'")
        node = node[part]
    return node


def _text(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    default = DEFAULT_TARGET
    if len(argv) >= 2 and argv[0] == "--default":
        default, argv = argv[1], argv[2:]
    try:
        if len(argv) == 1 and argv[0] == "target":
            print(resolve_target(default=default))
        elif len(argv) == 2 and argv[0] == "get":
            print(_text(get(load(default=default), argv[1])))
        elif len(argv) == 1 and argv[0] == "px4-env":
            for name, value in (load(default=default).get("px4_params") or {}).items():
                print(f"export PX4_PARAM_{name}=\"${{PX4_PARAM_{name}:-{_text(value)}}}\"")
        else:
            print(__doc__, file=sys.stderr)
            return 2
    except ProfileError as err:
        print(f"ERROR: {err}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
