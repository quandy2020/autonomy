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
from autosim.robot.joints import load_joints
from autosim.robot.profiles import apply_profile, shipped_robot_defaults
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


@pytest.mark.parametrize("profile", ["unitree_g1", "unitree_go2"])
def test_unitree_cameras_declare_body_axes(profile):
    """d435_link and front_camera are URDF links, not optical frames."""
    config = Config.load(ROOT / "config" / "robots" / f"{profile}.yaml")
    camera = config["habitat"]["sensors"]["camera"]
    assert camera["axes"] == "body"


def test_g1_home_pose_keeps_arms_at_the_sides():
    config = Config.load(ROOT / "config" / "robots" / "unitree_g1.yaml")
    robot = apply_profile(config["habitat"]["robot"])
    urdf = UrdfModel.load(robot["urdf"])
    assert urdf is not None
    pose = shipped_robot_defaults("unitree_g1")["joint_positions"]
    joints = load_joints(urdf.path)
    for name, value in pose.items():
        assert joints[name].check(value) == pytest.approx(value)
    # The URDF zero pose holds the forearms straight out; the shipped home
    # pose bends the elbows so the hands hang beside the hips.
    positions = {name: joint.home() for name, joint in joints.items() if not joint.mimic}
    zero_hand = urdf.link_transform("left_rubber_hand", positions)
    home_hand = urdf.link_transform("left_rubber_hand", {**positions, **pose})
    assert home_hand[0, 3] < zero_hand[0, 3] - 0.15
    assert home_hand[2, 3] < zero_hand[2, 3] - 0.15


def test_shipped_robot_defaults_cover_shipped_profiles_only():
    wheeled = shipped_robot_defaults("turtlebot3_burger")
    assert wheeled["profile"] == "turtlebot3_burger"
    assert wheeled["cmd_vel"] == "/cmd_vel"
    assert wheeled["wheel_radius"] > 0.0
    assert shipped_robot_defaults("no_such_robot") == {}
