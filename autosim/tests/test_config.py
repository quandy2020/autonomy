# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
            "robot": {
                "urdf": "",
                "max_linear": 0.5,
                "max_angular": 1.0,
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
                },
                "odom": {
                    "enabled": False,
                    "channel": "/odom",
                    "frame": "odom",
                    "child_frame": "base_footprint",
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
    assert settings["habitat"]["path"].endswith("17DRP5sb8fy.glb")
    assert settings["habitat"]["sensors"]["lidar_2d"]["enabled"] is True
    assert settings["habitat"]["sensors"]["lidar_3d"]["enabled"] is False
    assert settings["habitat"]["sensors"]["lidar_3d"]["vertical"]["num_rings"] == 16
    assert settings["habitat"]["sensors"]["camera"]["width"] == 640
    assert settings["habitat"]["sensors"]["lidar_2d"]["noise"] == 0.01
    assert settings["habitat"]["sensors"]["lidar_3d"]["noise"] == 0.015
    assert settings["habitat"]["sensors"]["odom"]["noise"] == 0.02
    assert settings["habitat"]["sensors"]["imu"]["noise"]["gyro"] == 0.01
    assert settings["habitat"]["sensors"]["imu"]["noise"]["accel"] == 0.05
    assert settings["habitat"]["sensors"]["camera"]["noise"]["depth"] == 0.0
    assert settings["habitat"]["map"]["enabled"] is True
    assert settings["habitat"]["map"]["ply"]["channel"] == "/map_points"
    assert settings["habitat"]["map"]["grid"]["channel"] == "/map"
    assert settings["habitat"]["robot"]["tf"]["enabled"] is True
    assert settings.channel_map()["tf"] == "/tf"
    assert settings.channel_map()["clock"] == "/clock"
    assert settings.channel_map()["footprint"] == "/footprint"
    assert settings["habitat"]["sensors"]["odom"]["child_frame"] == "base_footprint"
    assert settings["habitat"]["robot"]["footprint"]["frame"] == "base_footprint"


def test_map_ply_channel_and_stride():
    data = minimal_config()
    data["habitat"]["map"] = {
        "enabled": True,
        "frame": "map",
        "rate_hz": 0.0,
        "ply": {
            "file": "",
            "channel": "/map_points",
            "stride": 2,
            "range_max": 10.0,
            "horizontal": {"angle_min": -1, "angle_max": 1, "num_beams": 4},
            "vertical": {"angle_min": -0.1, "angle_max": 0.1, "num_rings": 2},
        },
        "grid": {
            "channel": "/map",
            "resolution": 0.05,
            "z_min": 0.0,
            "z_max": 2.0,
        },
    }
    settings = Config.load(data)
    assert settings["habitat"]["map"]["enabled"] is True
    assert settings["habitat"]["map"]["ply"]["channel"] == "/map_points"
    assert settings["habitat"]["map"]["ply"]["stride"] == 2
    assert settings.channel_map()["map_cloud"] == "/map_points"


def test_reject_invalid_map_stride():
    bad = minimal_config()
    bad["habitat"]["map"] = {
        "enabled": True,
        "frame": "map",
        "rate_hz": 0.0,
        "ply": {
            "file": "",
            "channel": "/map/points",
            "stride": 0,
            "range_max": 10.0,
            "horizontal": {"angle_min": -1, "angle_max": 1, "num_beams": 4},
            "vertical": {"angle_min": -0.1, "angle_max": 0.1, "num_rings": 2},
        },
        "grid": {
            "channel": "/map",
            "resolution": 0.05,
            "z_min": 0.0,
            "z_max": 2.0,
        },
    }
    with pytest.raises(ValueError, match="stride"):
        Config.load(bad)


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


def test_reject_negative_noise():
    bad = minimal_config()
    bad["habitat"]["sensors"]["lidar_2d"]["noise"] = -0.1
    with pytest.raises(ValueError, match="noise"):
        Config.load(bad)


def test_reject_negative_imu_noise():
    bad = minimal_config()
    bad["habitat"]["sensors"]["imu"]["noise"] = {"gyro": -1.0, "accel": 0.0}
    with pytest.raises(ValueError, match="gyro"):
        Config.load(bad)
