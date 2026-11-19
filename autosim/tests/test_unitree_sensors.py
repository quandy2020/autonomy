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

"""Unitree sensor frames resolve to links shipped by their URDFs."""

from pathlib import Path

import pytest

from autosim.robot.urdf import UrdfModel
from autosim.robot.profiles import apply_profile
from autosim.runtime import Config


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    ("profile", "camera_frame", "imu_frame", "lidar_frame"),
    [
        ("unitree_g1", "d435_link", "imu_in_torso", "mid360_link"),
        ("unitree_go2", "front_camera", "imu", "laser_link"),
    ],
)
def test_unitree_sensor_configs_resolve(profile, camera_frame, imu_frame, lidar_frame):
    config = Config.load(ROOT / "config" / "robots" / f"{profile}.yaml")
    sensors = config["habitat"]["sensors"]
    robot = apply_profile(config["habitat"]["robot"])
    urdf = UrdfModel.load(robot["urdf"])
    assert urdf is not None

    assert sensors["camera"]["enabled"] is True
    assert sensors["camera"]["frame"] == camera_frame
    assert sensors["camera"]["follow_urdf"] is True
    assert sensors["imu"]["enabled"] is True
    assert sensors["imu"]["frame"] == imu_frame
    assert sensors["lidar_2d"]["enabled"] is True
    assert sensors["lidar_2d"]["frame"] == lidar_frame

    assert camera_frame in urdf.parents
    assert imu_frame in urdf.parents
    if lidar_frame == "laser_link":
        assert lidar_frame not in urdf.parents
        assert sensors["lidar_2d"]["height"] == pytest.approx(0.1)
    else:
        assert lidar_frame in urdf.parents


def test_g1_uses_the_urdf_mid360_mount():
    config = Config.load(ROOT / "config" / "robots" / "unitree_g1.yaml")
    robot = apply_profile(config["habitat"]["robot"])
    urdf = UrdfModel.load(robot["urdf"])
    assert urdf is not None
    assert urdf.laser_xyz() == pytest.approx(urdf.mounts["mid360_link"])
    assert urdf.camera_xyz() == pytest.approx(urdf.mounts["d435_link"])
    assert urdf.imu_xyz() == pytest.approx(urdf.mounts["imu_in_torso"])
