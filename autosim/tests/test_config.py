# autosim/tests/test_config.py
from pathlib import Path

import pytest

from autosim.config import load_config, ConfigError


ROOT = Path(__file__).resolve().parents[1]


def test_load_default_yaml():
    cfg = load_config(ROOT / "config" / "default.yaml")
    assert cfg["channels"]["cmd_vel"] == "/cmd_vel"
    assert cfg["channels"]["scan"] == "/scan"
    assert cfg["truth"]["enabled"] is False
    assert cfg["scene"]["backend"] == "minimal"


def test_reject_empty_channel():
    bad = {
        "scene": {"backend": "minimal", "path": "", "spawn": [0.0, 0.0, 0.0]},
        "robot": {
            "max_linear": 0.5,
            "max_angular": 1.0,
            "wheel_separation": 0.3,
        },
        "rates": {"control_hz": 50.0, "scan_hz": 10.0, "rgb_hz": 15.0, "imu_hz": 100.0},
        "channels": {
            "cmd_vel": "",
            "scan": "/scan",
            "rgb": "/camera/rgb/image_raw",
            "depth": "/camera/depth/image_raw",
            "camera_info": "/camera/camera_info",
            "imu": "/imu",
            "odom": "/odom",
            "gt_pose": "/gt/pose",
        },
        "noise": {"odom": 0.0, "imu": 0.0, "lidar": 0.0},
        "truth": {"enabled": False},
        "habitat": {"gpu": 0, "width": 640, "height": 480, "headless": True},
        "watchdog_sec": 0.5,
    }
    with pytest.raises(ConfigError):
        load_config(bad)


def test_reject_duplicate_channels():
    bad = {
        "scene": {"backend": "minimal", "path": "", "spawn": [0.0, 0.0, 0.0]},
        "robot": {"max_linear": 0.5, "max_angular": 1.0, "wheel_separation": 0.3},
        "rates": {"control_hz": 50.0, "scan_hz": 10.0, "rgb_hz": 15.0, "imu_hz": 100.0},
        "channels": {
            "cmd_vel": "/scan",
            "scan": "/scan",
            "rgb": "/camera/rgb/image_raw",
            "depth": "/camera/depth/image_raw",
            "camera_info": "/camera/camera_info",
            "imu": "/imu",
            "odom": "/odom",
            "gt_pose": "/gt/pose",
        },
        "noise": {"odom": 0.0, "imu": 0.0, "lidar": 0.0},
        "truth": {"enabled": False},
        "habitat": {"gpu": 0, "width": 640, "height": 480, "headless": True},
        "watchdog_sec": 0.5,
    }
    with pytest.raises(ConfigError):
        load_config(bad)
