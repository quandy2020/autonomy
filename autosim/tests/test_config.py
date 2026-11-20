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
import yaml

from autosim.runtime import Config


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
    assert settings["habitat"]["backend"] == "habitat"
    assert settings.channel_map()["cmd_vel"] == "/cmd_vel"
    assert settings.channel_map()["scan"] == "/scan"
    assert settings.channel_map()["points"] == "/points"
    assert settings["habitat"]["robot"]["truth"]["enabled"] is False
    # Habitat default: GLB scene resource, no procedural scenario.
    assert str(settings["habitat"]["path"]).endswith(".glb")
    assert settings["habitat"]["scenario"]["enabled"] is False
    assert settings["habitat"]["sensors"]["lidar_2d"]["enabled"] is True
    assert settings["habitat"]["sensors"]["lidar_3d"]["enabled"] is False
    assert settings["habitat"]["sensors"]["camera"]["enabled"] is True
    assert settings["habitat"]["sensors"]["imu"]["enabled"] is True
    assert settings["habitat"]["sensors"]["odom"]["enabled"] is True
    assert settings["habitat"]["map"]["enabled"] is True
    assert settings["habitat"]["map"]["publish"] is True
    assert settings["habitat"]["mode"] == "nav"
    assert settings["habitat"]["map"]["ply"]["channel"] == "/overall/map"
    assert settings["habitat"]["map"]["grid"]["channel"] == "/map"
    assert settings["habitat"]["robot"]["tf"]["enabled"] is True
    assert settings.channel_map()["tf"] == "/tf"
    assert settings.channel_map()["clock"] == "/clock"
    assert settings.channel_map()["gt_pose"] == "/gt/pose"
    assert "gt_odom" not in settings.channel_map()
    assert settings.channel_map()["map_cloud"] == "/overall/map"
    assert settings["habitat"]["sensors"]["odom"]["child_frame"] == "base_footprint"


def test_truth_odometry_channel_is_optional():
    settings = Config.load(minimal_config())
    assert settings.channel_map()["gt_pose"] == "/gt/pose"
    assert "gt_odom" not in settings.channel_map()


def test_reject_non_string_truth_odometry_channel():
    bad = minimal_config()
    bad["habitat"]["robot"]["truth"]["odometry_channel"] = 42
    with pytest.raises(ValueError, match="truth.odometry_channel"):
        Config.load(bad)


def test_reject_duplicate_truth_odometry_channel():
    bad = minimal_config()
    bad["habitat"]["robot"]["truth"]["odometry_channel"] = "/odom"
    with pytest.raises(ValueError, match="duplicate channel"):
        Config.load(bad)


def test_load_fake_yaml():
    settings = Config.load(ROOT / "config" / "fake.yaml")
    assert settings["habitat"]["backend"] == "fake"
    assert settings["habitat"]["scenario"]["enabled"] is True
    assert int(settings["habitat"]["scenario"]["obstacle_number"]) > 0
    assert settings["habitat"]["robot"]["footprint"]["enabled"] is True
    assert settings.channel_map()["map_cloud"] == "/overall/map"
    assert settings.channel_map()["depth_points"] == "/camera/depth/points"
    assert settings.channel_map()["fov_marker"] == "/camera/depth/fov"
    assert settings.channel_map()["footprint"] == "/footprint"


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


def test_map_grid_only_empty_ply_channel():
    """Occupancy-only publish: empty ply.channel skips dense PointCloud2."""
    data = minimal_config()
    data["habitat"]["map"] = {
        "enabled": True,
        "frame": "map",
        "rate_hz": 1.0,
        "ply": {"channel": "", "stride": 1},
        "grid": {
            "channel": "/map",
            "resolution": 0.05,
            "z_min": 0.0,
            "z_max": 2.0,
        },
    }
    settings = Config.load(data)
    assert settings["habitat"]["map"]["ply"]["channel"] == ""
    assert "map_cloud" not in settings.channel_map()
    assert settings.channel_map()["map_grid"] == "/map"


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


def test_extends_deep_merges_and_keeps_child_keys(tmp_path):
    base = minimal_config()
    base_path = tmp_path / "base.yaml"
    base_path.write_text(yaml.safe_dump(base))
    child_path = tmp_path / "child.yaml"
    child_path.write_text(
        yaml.safe_dump(
            {
                "extends": "base.yaml",
                "habitat": {
                    "robot": {"max_linear": 1.5},
                    "sensors": {"camera": {"frame": "front_camera"}},
                },
            }
        )
    )
    settings = Config.load(child_path)
    assert settings["habitat"]["robot"]["max_linear"] == 1.5
    assert settings["habitat"]["robot"]["max_angular"] == 1.0
    assert settings["habitat"]["sensors"]["camera"]["frame"] == "front_camera"
    assert settings["habitat"]["sensors"]["camera"]["width"] == 640
    assert "extends" not in settings.data


def test_extends_resolves_relative_to_the_declaring_file(tmp_path):
    nested = tmp_path / "nested"
    nested.mkdir()
    (nested / "base.yaml").write_text(yaml.safe_dump(minimal_config()))
    child = nested / "child.yaml"
    child.write_text(
        yaml.safe_dump({"extends": "base.yaml", "habitat": {"gpu": 2}})
    )
    assert Config.load(child)["habitat"]["gpu"] == 2


def test_extends_chain_and_list_order(tmp_path):
    (tmp_path / "one.yaml").write_text(
        yaml.safe_dump({"habitat": {"robot": {"max_linear": 0.3}}})
    )
    (tmp_path / "two.yaml").write_text(
        yaml.safe_dump({"extends": "one.yaml", "habitat": {"robot": {"max_linear": 0.4}}})
    )
    (tmp_path / "base.yaml").write_text(yaml.safe_dump(minimal_config()))
    child = tmp_path / "child.yaml"
    child.write_text(
        yaml.safe_dump(
            {
                "extends": ["base.yaml", "two.yaml"],
                "habitat": {"robot": {"max_angular": 2.0}},
            }
        )
    )
    settings = Config.load(child)
    assert settings["habitat"]["robot"]["max_linear"] == 0.4
    assert settings["habitat"]["robot"]["max_angular"] == 2.0


def test_extends_rejects_cycles(tmp_path):
    (tmp_path / "a.yaml").write_text(yaml.safe_dump({"extends": "b.yaml"}))
    (tmp_path / "b.yaml").write_text(yaml.safe_dump({"extends": "a.yaml"}))
    with pytest.raises(ValueError, match="extends cycle"):
        Config.load(tmp_path / "a.yaml")


def test_extends_rejects_non_string_entries(tmp_path):
    (tmp_path / "base.yaml").write_text(yaml.safe_dump(minimal_config()))
    child = tmp_path / "child.yaml"
    child.write_text(yaml.safe_dump({"extends": ["base.yaml", 3]}))
    with pytest.raises(ValueError, match="non-empty strings"):
        Config.load(child)


def test_config_expands_environment_variables(monkeypatch):
    monkeypatch.setenv("AUTOSIM_TEST_SCENE_ROOT", "/tmp/scenes")
    data = minimal_config()
    data["habitat"]["path"] = "${AUTOSIM_TEST_SCENE_ROOT}/room.glb"
    assert Config.load(data)["habitat"]["path"] == "/tmp/scenes/room.glb"


def test_config_env_default_and_missing_variable(monkeypatch):
    monkeypatch.delenv("AUTOSIM_TEST_MISSING", raising=False)
    data = minimal_config()
    data["habitat"]["path"] = "${AUTOSIM_TEST_MISSING:-/fallback}/room.glb"
    assert Config.load(data)["habitat"]["path"] == "/fallback/room.glb"
    data["habitat"]["path"] = "${AUTOSIM_TEST_MISSING}/room.glb"
    with pytest.raises(ValueError, match="AUTOSIM_TEST_MISSING"):
        Config.load(data)
