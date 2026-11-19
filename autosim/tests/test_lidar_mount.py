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

"""Lidar mount resolution for models that ship no lidar link."""

import pytest

from autosim.backends.habitat.simulator import Simulator
from autosim.robot.urdf import UrdfModel


def bare_simulator(root_height=0.0, lidar_height=None):
    lidar = {
        "enabled": True,
        "num_beams": 8,
        "angle_min": -3.14,
        "angle_max": 3.14,
        "range_min": 0.1,
        "range_max": 10.0,
    }
    if lidar_height is not None:
        lidar["height"] = lidar_height
    return Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={
            "habitat": {
                "path": "",
                "spawn": [0.0, 0.0, 0.0],
                "robot": {"urdf": "", "root_height": root_height},
                "sensors": {"lidar_2d": lidar},
            }
        },
        open_session=False,
    )


def test_lidar_mount_uses_the_urdf_link_when_present():
    simulator = bare_simulator()
    simulator.urdf = UrdfModel.load("robot/turtlebot3_burger/turtlebot3_burger.urdf")
    assert simulator.urdf is not None
    assert simulator.lidar_mount() == pytest.approx(simulator.urdf.laser_xyz())


def test_lidar_mount_falls_back_above_the_root():
    simulator = bare_simulator(root_height=0.85, lidar_height=0.25)
    simulator.urdf = None
    assert simulator.lidar_mount() == pytest.approx((0.0, 0.0, 1.10))
    simulator.floor_y = 0.0
    simulator.x = simulator.y = simulator.yaw = 0.0
    assert simulator.laser_origin()[1] == pytest.approx(1.10)
    assert simulator.laser_origin_at(2.0, 3.0, 0.0)[1] == pytest.approx(1.10)


def test_lidar_mount_defaults_to_twenty_centimetres():
    simulator = bare_simulator()
    simulator.urdf = None
    assert simulator.lidar_mount() == pytest.approx((0.0, 0.0, 0.2))
