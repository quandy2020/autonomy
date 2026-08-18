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

"""Tests for URDF path resolve and sensor mounts."""

from pathlib import Path

import pytest

from autosim.config import Config
from autosim.urdf import UrdfModel


ROOT = Path(__file__).resolve().parents[1]


def minimal_robot_config(urdf: str = ""):
    return {
        "habitat": {
            "path": "",
            "spawn": [0.0, 0.0, 0.0],
            "gpu": 0,
            "robot": {
                "urdf": urdf,
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


def test_resolve_husky_relative():
    path = UrdfModel.resolve("urdf/husky.urdf")
    assert path is not None
    assert path.is_file()
    assert path.name == "husky.urdf"


def test_load_husky_laser_height():
    model = UrdfModel.load("urdf/husky.urdf")
    assert model is not None
    x, y, z = model.laser_xyz()
    assert abs(x - 0.15) < 1e-6
    assert abs(y - 0.0) < 1e-6
    assert abs(z - 0.465) < 1e-6
    ix, _, iz = model.imu_xyz()
    assert abs(ix - 0.1) < 1e-6
    assert abs(iz - 0.345) < 1e-6
    cx, cy, cz = model.camera_xyz()
    assert abs(cx - 0.2) < 1e-6
    assert abs(cy - 0.0) < 1e-6
    assert abs(cz - 0.485) < 1e-6


def test_husky_prefers_base_footprint_tree():
    model = UrdfModel.load("urdf/husky.urdf")
    assert model is not None
    assert model.root == "base_footprint"
    assert model.odom_child_frame() == "base_footprint"
    assert model.body_frame() == "base_link"
    bx, by, bz = model.link_xyz_from("base_footprint", "base_link")
    assert abs(bx - 0.0) < 1e-6
    assert abs(by - 0.0) < 1e-6
    assert abs(bz - 0.165) < 1e-6
    lx, ly, lz = model.link_xyz_from("base_link", "laser_link")
    assert abs(lx - 0.15) < 1e-6
    assert abs(ly - 0.0) < 1e-6
    assert abs(lz - 0.3) < 1e-6
    footprint = model.footprint_polygon()
    assert len(footprint) == 4
    assert abs(footprint[0][0] - 0.4935) < 1e-6
    assert abs(footprint[0][1] - 0.285) < 1e-6
    assert abs(footprint[2][0] + 0.4935) < 1e-6
    assert abs(footprint[2][1] + 0.285) < 1e-6


def test_turtlebot3_derives_offset_footprint_from_urdf():
    model = UrdfModel.load("urdf/turtlebot3_burger.urdf")
    assert model is not None
    assert model.root == "base_footprint"
    footprint = model.footprint_polygon()
    assert len(footprint) == 4
    assert abs(footprint[0][0] - 0.038) < 1e-6
    assert abs(footprint[0][1] - 0.07) < 1e-6
    assert abs(footprint[2][0] + 0.102) < 1e-6
    assert abs(footprint[2][1] + 0.07) < 1e-6


def test_load_empty_urdf():
    assert UrdfModel.load("") is None
    assert UrdfModel.resolve("") is None


def test_default_yaml_urdf():
    settings = Config.load(ROOT / "config" / "default.yaml")
    assert settings["habitat"]["robot"]["urdf"] == "urdf/husky.urdf"


def test_reject_missing_urdf():
    bad = minimal_robot_config(urdf="urdf/does_not_exist.urdf")
    with pytest.raises(ValueError, match="urdf"):
        Config.load(bad)


def test_mock_simulator_loads_mounts():
    from autosim.simulator import Simulator

    data = minimal_robot_config(urdf="urdf/husky.urdf")
    sim = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings=data,
        open_session=False,
    )
    assert sim.urdf is not None
    ox, oy, oz = sim.laser_origin()
    assert abs(oy - 0.465) < 1e-6
    assert abs(ox - 0.15) < 1e-6
    assert abs(oz - 0.0) < 1e-6
