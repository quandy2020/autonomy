from pathlib import Path

import pytest

from autosim.config import Config


ROOT = Path(__file__).resolve().parents[1]


def minimal_config(**overrides):
    data = {
        "habitat": {
            "path": "",
            "spawn": [0.0, 0.0, 0.0],
            "gpu": 0,
            "headless": True,
            "robot": {
                "urdf": "",
                "max_linear": 0.5,
                "max_angular": 1.0,
                "wheel_separation": 0.3,
                "base_frame": "base_link",
                "map_frame": "map",
                "watchdog_sec": 0.5,
                "control_hz": 50.0,
                "cmd_vel": "/cmd_vel",
                "truth": {"enabled": False, "channel": "/gt/pose"},
            },
            "sensors": {
                "lidar_2d": {
                    "enabled": False,
                    "channel": "/scan",
                    "frame": "laser_link",
                    "rate_hz": 10.0,
                    "angle_min": -3.14,
                    "angle_max": 3.14,
                    "range_min": 0.1,
                    "range_max": 30.0,
                    "num_beams": 360,
                    "noise": 0.0,
                },
                "lidar_3d": {
                    "enabled": False,
                    "channel": "/points",
                    "frame": "lidar_link",
                    "rate_hz": 10.0,
                    "horizontal": {
                        "angle_min": -3.14,
                        "angle_max": 3.14,
                        "num_beams": 360,
                    },
                    "vertical": {
                        "angle_min": -0.26,
                        "angle_max": 0.26,
                        "num_rings": 16,
                    },
                    "range_min": 0.1,
                    "range_max": 30.0,
                    "noise": 0.0,
                },
                "camera": {
                    "enabled": False,
                    "width": 640,
                    "height": 480,
                    "rgb_channel": "/camera/rgb/image_raw",
                    "depth_channel": "/camera/depth/image_raw",
                    "info_channel": "/camera/camera_info",
                    "frame": "camera_link",
                    "rate_hz": 15.0,
                },
                "imu": {
                    "enabled": False,
                    "channel": "/imu",
                    "frame": "imu_link",
                    "rate_hz": 100.0,
                    "noise": 0.0,
                },
                "odom": {
                    "enabled": False,
                    "channel": "/odom",
                    "frame": "odom",
                    "child_frame": "base_link",
                    "noise": 0.0,
                },
            },
        },
    }
    data.update(overrides)
    return data


def test_load_default_yaml():
    settings = Config.load(ROOT / "config" / "default.yaml")
    assert settings.channel_map()["cmd_vel"] == "/cmd_vel"
    assert settings.channel_map()["scan"] == "/scan"
    assert settings.channel_map()["points"] == "/points"
    assert "lidar" not in settings["habitat"]["sensors"]
    assert settings["habitat"]["robot"]["truth"]["enabled"] is False
    assert settings["habitat"]["path"] == ""
    assert settings["habitat"]["sensors"]["lidar_2d"]["enabled"] is False
    assert settings["habitat"]["sensors"]["lidar_3d"]["vertical"]["num_rings"] == 16
    assert settings["habitat"]["sensors"]["camera"]["width"] == 640


def test_reject_empty_channel():
    bad = minimal_config()
    bad["habitat"]["sensors"]["lidar_2d"]["channel"] = ""
    with pytest.raises(ValueError):
        Config.load(bad)


def test_reject_duplicate_channels():
    bad = minimal_config()
    bad["habitat"]["robot"]["cmd_vel"] = "/scan"
    with pytest.raises(ValueError):
        Config.load(bad)
