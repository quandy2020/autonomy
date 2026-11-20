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

"""Sensors mounted on moving links: lidar pose, IMU frame, joint velocity."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from autosim.backends.habitat.articulation import Articulation
from autosim.backends.habitat.simulator import Simulator
from autosim.bridge.messages import Messages
from autosim.robot.joints import Joint
from tests.test_articulation import FakeAO


def unitree_simulator(profile, frame, height=None, root_height=0.85):
    lidar = {
        "enabled": True,
        "num_beams": 8,
        "angle_min": -3.14,
        "angle_max": 3.14,
        "range_min": 0.1,
        "range_max": 10.0,
        "frame": frame,
    }
    if height is not None:
        lidar["height"] = height
    return Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={
            "habitat": {
                "path": "",
                "spawn": [0.0, 0.0, 0.0],
                "robot": {
                    "urdf": f"robot/{profile}/{profile}.urdf",
                    "root_height": root_height,
                },
                "sensors": {"lidar_2d": lidar},
            }
        },
        open_session=False,
    )


def test_lidar_follows_the_waist_joint():
    simulator = unitree_simulator("unitree_g1", "mid360_link")
    simulator.floor_y = 0.0
    simulator.x = simulator.y = simulator.yaw = 0.0
    origin0, rotation0 = simulator.laser_pose()
    assert simulator.laser_heading() == pytest.approx(0.0, abs=1e-9)

    simulator.joint_states = lambda: {"waist_yaw_joint": 0.6}
    origin1, rotation1 = simulator.laser_pose()

    assert not np.allclose(rotation0, rotation1)
    assert simulator.laser_heading() == pytest.approx(0.6, abs=1e-6)
    # The mid360 sits on the torso axis, so only the heading changes.
    assert np.linalg.norm(origin1 - origin0) < 0.02
    assert origin1[2] == pytest.approx(origin0[2])


def test_lidar_pose_uses_the_go2_urdf_mount():
    simulator = unitree_simulator("unitree_go2", "laser_link", root_height=0.45)
    simulator.floor_y = 0.0
    simulator.x, simulator.y, simulator.yaw = 1.0, 2.0, 0.5
    origin, rotation = simulator.laser_pose()
    # Head_upper sits 0.285 m ahead and 0.01 m above base; laser_link adds
    # another 0.055 m of height.
    assert origin == pytest.approx(
        [1.0 + 0.285 * math.cos(0.5), 2.0 + 0.285 * math.sin(0.5), 0.515]
    )
    assert rotation[0, 0] == pytest.approx(math.cos(0.5))
    assert rotation[1, 0] == pytest.approx(math.sin(0.5))
    # The planar helper still reports the same height for map / topdown code.
    assert simulator.laser_origin()[1] == pytest.approx(0.515)


def test_lidar_pose_falls_back_to_the_configured_height():
    simulator = unitree_simulator("unitree_go2", "lidar_link", height=0.1, root_height=0.45)
    simulator.floor_y = 0.0
    simulator.x, simulator.y, simulator.yaw = 1.0, 2.0, 0.5
    origin, _ = simulator.laser_pose()
    assert origin == pytest.approx([1.0, 2.0, 0.55])


def test_laser_scan_reports_sweep_timing():
    message = Messages.encode_laser_scan(
        ranges=np.zeros(360, dtype=np.float32),
        stamp=(1, 0),
        frame_id="laser_link",
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=2.0 * math.pi / 360.0,
        range_min=0.1,
        range_max=30.0,
        scan_time=0.1,
    )
    assert message.scan_time == pytest.approx(0.1)
    assert message.time_increment == pytest.approx(0.1 / 359.0)


def test_imu_orientation_covariance_matches_the_pose_source():
    known = Messages.encode_inertial(
        (0, 0), "imu", (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 9.81)
    )
    assert list(known.orientation_covariance)[0] == pytest.approx(0.0)

    unknown = Messages.encode_inertial(
        (0, 0),
        "imu",
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 9.81),
        orientation_variance=None,
    )
    assert list(unknown.orientation_covariance)[0] == pytest.approx(-1.0)


def test_joint_velocity_uses_the_command_interval():
    robot = FakeAO()
    controller = Articulation(
        robot,
        {
            "arm": Joint("arm", "revolute", "a", -1, 1),
            "slide": Joint("slide", "prismatic", "b", 0, 0.5),
        },
    )
    controller.set_positions({"arm": 0.2}, dt=0.1)
    assert controller.velocities()["arm"] == pytest.approx(2.0)
    assert controller.velocities()["slide"] == pytest.approx(0.0)

    controller.set_positions({}, dt=0.1)
    assert controller.velocities()["arm"] == pytest.approx(0.0)


def test_imu_sample_rotates_into_the_link_frame():
    from autosim.runtime.runner import Runner

    runner = object.__new__(Runner)
    yaw_90 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    runner.simulator = SimpleNamespace(link_rotation=lambda frame: yaw_90)
    orientation, angular, specific = runner.imu_sample("imu", 0.0, 1.0, (2.0, 0.0, 0.0))
    assert angular == pytest.approx((0.0, 0.0, 1.0))
    assert specific == pytest.approx((0.0, -2.0, 0.0))
    assert orientation == pytest.approx((0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))


def test_imu_sample_falls_back_to_the_base_frame():
    from autosim.runtime.runner import Runner

    runner = object.__new__(Runner)
    runner.simulator = SimpleNamespace()
    orientation, angular, specific = runner.imu_sample("imu", 0.5, 1.0, (2.0, 0.0, 0.0))
    assert angular == pytest.approx((0.0, 0.0, 1.0))
    assert specific == pytest.approx((2.0, 0.0, 0.0))
    assert orientation == pytest.approx((0.0, 0.0, math.sin(0.25), math.cos(0.25)))
