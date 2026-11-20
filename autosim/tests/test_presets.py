"""Scene/robot/sensor composition for the short CLI."""

from pathlib import Path

import pytest

from autosim.runtime import Config
from autosim.runtime.presets import (
    compose_config,
    format_catalog,
    robot_names,
    scene_names,
    sensor_names,
)

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / "config" / "default.yaml"


def test_catalog_lists_shipped_selectors():
    assert "17DRP5sb8fy" in scene_names()
    assert "29hnd4uzFmX" in scene_names()
    assert "unitree_go2" in robot_names()
    assert "google_robot" in robot_names()
    assert "camera" in sensor_names()
    assert "empty" in format_catalog()


def test_every_scene_config_has_a_path():
    for name in scene_names():
        data = Config.read(ROOT / "config" / "scenes" / f"{name}.yaml")
        assert str(data["habitat"]["path"]).strip(), name


def test_compose_scene_and_robot():
    settings = compose_config(
        DEFAULT,
        scene="jtcxE69GiFV",
        robot="unitree_go2",
        sensors=["basic"],
    )
    habitat = settings["habitat"]
    assert habitat["path"].endswith("jtcxE69GiFV/jtcxE69GiFV.glb")
    assert habitat["robot"]["profile"] == "unitree_go2"
    assert habitat["sensors"]["camera"]["enabled"] is True
    assert habitat["sensors"]["lidar_2d"]["enabled"] is True
    assert habitat["sensors"]["imu"]["enabled"] is True
    assert habitat["sensors"]["odom"]["enabled"] is True
    assert habitat["sensors"]["lidar_3d"]["enabled"] is False


def test_robot_defaults_apply_when_sensors_are_omitted():
    settings = compose_config(DEFAULT, robot="google_robot")
    sensors = settings["habitat"]["sensors"]
    assert sensors["lidar_2d"]["enabled"] is False
    assert sensors["stereo"]["enabled"] is True
    assert sensors["range"]["enabled"] is True


def test_sensor_selection_can_disable_robot_extras():
    settings = compose_config(
        DEFAULT,
        robot="google_robot",
        sensors="camera,imu,odom",
    )
    sensors = settings["habitat"]["sensors"]
    assert sensors["camera"]["enabled"] is True
    assert sensors["imu"]["enabled"] is True
    assert sensors["odom"]["enabled"] is True
    assert sensors["stereo"]["enabled"] is False
    assert sensors["range"]["enabled"] is False
    assert sensors["chase"]["enabled"] is False


def test_empty_scene_uses_studio_floor():
    settings = compose_config(DEFAULT, scene="empty", robot="google_robot")
    habitat = settings["habitat"]
    assert habitat["path"] == ""
    assert habitat["scenario"]["enabled"] is True
    assert habitat["scenario"]["type"] == "floor"
    assert habitat["spawn"] == [0.0, 0.0, 0.0]


def test_unknown_selector_is_actionable():
    with pytest.raises(ValueError, match="unknown scene"):
        compose_config(DEFAULT, scene="missing_scene")
    with pytest.raises(ValueError, match="unknown sensor"):
        compose_config(DEFAULT, sensors=["missing_sensor"])
