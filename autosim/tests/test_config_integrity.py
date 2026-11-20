"""Shipped configs must load and agree with the URDF they point at."""
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

from autosim.bridge.cameras import camera_rig
from autosim.robot.joints import load_joints
from autosim.robot.profiles import apply_profile, resolve_path
from autosim.robot.urdf import UrdfModel
from autosim.runtime import Config

ROOT = UrdfModel.package_root()
def _shipped(folder: Path) -> list[Path]:
    """Standalone configs; ``_base.yaml`` is a partial merged by its children."""
    return sorted(path for path in folder.glob("*.yaml") if not path.name.startswith("_"))


ALL_CONFIGS = (
    sorted((ROOT / "config").glob("*.yaml"))
    + sorted((ROOT / "config" / "robots").glob("*.yaml"))
    + sorted((ROOT / "config" / "scenes").glob("*.yaml"))
)
CONFIGS = _shipped(ROOT / "config") + _shipped(ROOT / "config" / "robots")
ROBOT_CONFIGS = _shipped(ROOT / "config" / "robots")


def _ids(path: Path) -> str:
    return str(path.relative_to(ROOT))


def _skip_reason(data) -> str | None:
    """Skip configs whose external assets are not installed here."""
    habitat = data.get("habitat") or {}
    if str(habitat.get("backend") or "").lower() == "isaac":
        return "isaac backend needs USD assets"
    scene = str(habitat.get("path") or "").strip()
    if scene and not resolve_path(scene).is_file():
        return f"scene not installed: {scene}"
    return None


def _duplicate_keys(node, prefix: str = "") -> list[str]:
    """Dotted paths of repeated keys; PyYAML keeps the last one silently."""
    found: list[str] = []
    if isinstance(node, yaml.MappingNode):
        seen = set()
        for key_node, value_node in node.value:
            key = key_node.value
            if key in seen:
                found.append(f"{prefix}{key}")
            seen.add(key)
            found.extend(_duplicate_keys(value_node, f"{prefix}{key}."))
    elif isinstance(node, yaml.SequenceNode):
        for index, item in enumerate(node.value):
            found.extend(_duplicate_keys(item, f"{prefix}[{index}]."))
    return found


@pytest.mark.parametrize("path", CONFIGS, ids=_ids)
def test_shipped_config_loads(path: Path):
    reason = _skip_reason(yaml.safe_load(path.read_text()))
    if reason:
        pytest.skip(reason)
    Config.load(path)


@pytest.mark.parametrize("path", ALL_CONFIGS, ids=_ids)
def test_config_has_no_duplicate_keys(path: Path):
    assert _duplicate_keys(yaml.compose(path.read_text())) == []


@pytest.mark.parametrize("path", CONFIGS, ids=_ids)
def test_sensor_frames_exist_in_urdf(path: Path):
    data = Config._load_yaml(path)
    reason = _skip_reason(data)
    if reason:
        pytest.skip(reason)
    habitat = data["habitat"]
    robot = apply_profile(habitat.get("robot") or {})
    urdf = str(robot.get("urdf") or "").strip()
    if not urdf:
        pytest.skip("config has no URDF")
    model = UrdfModel.load(str(resolve_path(urdf)))
    links = {model.root} | set(model.parents)
    sensors = habitat.get("sensors") or {}
    frames: list[tuple[str, str]] = []
    for name in ("lidar_2d", "camera", "imu"):
        block = sensors.get(name) or {}
        if block.get("enabled") and block.get("frame"):
            frames.append((f"sensors.{name}.frame", block["frame"]))
    odom = sensors.get("odom") or {}
    if odom.get("enabled") and odom.get("child_frame"):
        frames.append(("sensors.odom.child_frame", odom["child_frame"]))
    stereo = sensors.get("stereo") or {}
    if stereo.get("enabled"):
        for key in ("left_frame", "right_frame"):
            if stereo.get(key):
                frames.append((f"sensors.stereo.{key}", stereo[key]))
    ranges = sensors.get("range") or {}
    if ranges.get("enabled"):
        for frame in ranges.get("sensors") or {}:
            frames.append((f"sensors.range.{frame}", frame))
    for name, spec in (camera_rig(sensors) or {}).items():
        if spec.get("enabled", True) and spec.get("frame"):
            frames.append((f"sensors.cameras.{name}.frame", spec["frame"]))
    missing = [f"{label}={frame}" for label, frame in frames if frame not in links]
    assert missing == [], f"{path.name}: {missing} not in {urdf}"


@pytest.mark.parametrize("path", ROBOT_CONFIGS, ids=_ids)
def test_wheel_radius_matches_urdf(path: Path):
    habitat = Config._load_yaml(path)["habitat"]
    robot = apply_profile(habitat.get("robot") or {})
    if float(robot.get("wheel_radius") or 0.0) <= 0.0:
        return
    joints = load_joints(resolve_path(str(robot["urdf"])))
    assert any("wheel" in name.lower() for name in joints), path.name


@pytest.mark.parametrize("path", CONFIGS, ids=_ids)
def test_channels_are_unique(path: Path):
    reason = _skip_reason(yaml.safe_load(path.read_text()))
    if reason:
        pytest.skip(reason)
    channels = Config.load(path).channel_map()
    seen: dict[str, str] = {}
    duplicates: dict[str, list[str]] = {}
    for key, channel in channels.items():
        if channel in seen:
            duplicates.setdefault(channel, [seen[channel]]).append(key)
        else:
            seen[channel] = key
    assert duplicates == {}


def test_topics_doc_matches_channel_maps():
    result = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "gen_topics_doc.py"), "--check"],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_google_wheels_declare_radius_for_every_wheel_joint():
    """One ``wheel_radius`` cannot drive a 64 mm front and 38 mm rear pair."""
    robot = Config.load(ROOT / "config" / "robots" / "google_robot.yaml")["habitat"][
        "robot"
    ]
    joints = load_joints(resolve_path(robot["urdf"]))
    wheels = {name for name in joints if "wheel" in name.lower()}
    assert wheels == set(robot["wheel_radii"])
    assert wheels == set(robot["wheel_separations"])
    assert (
        robot["wheel_radii"]["joint_wheel_left"]
        > robot["wheel_radii"]["joint_rear_wheel_left"]
    )


def test_cmd_vel_type_selects_the_reader_message():
    from automsgs.msgs.geometry_msgs.twist_pb2 import Twist
    from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped

    from autosim.runtime.runner import cmd_vel_message_type

    assert cmd_vel_message_type({"cmd_vel_type": "twist"}) is Twist
    assert cmd_vel_message_type({"cmd_vel_type": "twist_stamped"}) is TwistStamped
    assert cmd_vel_message_type({}) is TwistStamped
