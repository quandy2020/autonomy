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

"""Fake backend: scenario map, RGBD raycast cloud, runner without habitat-sim."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

from autosim.config import Config
from autosim.fake import Map as FakeMap
from autosim.fake import Robot as FakeRobot
from autosim.fake import Sensor as FakeSensor
from autosim.runner import Runner


ROOT = Path(__file__).resolve().parents[1]

SCENARIO = {
    "enabled": True,
    "type": "maze3d",
    "seed": 510,
    "resolution": 0.2,
    "x_length": 10,
    "y_length": 10,
    "z_length": 2,
    "road_width": 0.5,
    "add_wall_x": 1,
    "add_wall_y": 1,
}


def test_fake_map_builds_points_and_grid():
    built = FakeMap({"enabled": True}).build(SCENARIO)
    assert built.points.shape[0] > 0
    assert built.points.shape[1] == 3
    assert built.occupied is not None
    assert built.resolution > 0.0
    cloud, grid, resolution, ox, oy, width, height = built.sample()
    assert cloud.shape[0] == built.points.shape[0]
    assert grid is not None
    assert width > 0 and height > 0
    assert resolution > 0.0


def test_fake_sensor_raycasts_within_rgbd_fov():
    built = FakeMap({"enabled": True}).build(SCENARIO)
    sensor = FakeSensor(
        {
            "width": 80,
            "height": 60,
            "hfov_deg": 90.0,
            "vfov_deg": 70.0,
            "sensor_height": 0.6,
            "range_min": 0.1,
            "range_max": 30.0,
            "depth_points_stride": 2,
        }
    )
    points = sensor.sample_points(
        built.occupied, built.origin, built.resolution, (0.0, 0.0, 0.0)
    )
    assert points.shape[0] <= 80 * 60
    assert points.shape[0] > 0
    # camera_link: x forward.
    assert float(np.min(points[:, 0])) > -1e-3


def test_fake_sensor_hits_wall_ahead_not_behind():
    """A thin occupied slab in +X is hit when facing +X; not when facing -X."""
    occupied = np.zeros((20, 20, 10), dtype=bool)
    occupied[15, :, :] = True  # wall at +X
    origin = np.array([-2.0, -2.0, 0.0], dtype=np.float64)
    resolution = 0.2
    sensor = FakeSensor(
        {
            "width": 32,
            "height": 24,
            "hfov_deg": 60.0,
            "sensor_height": 0.5,
            "range_min": 0.1,
            "range_max": 10.0,
            "depth_points_stride": 4,
        }
    )
    facing_pos = sensor.sample_points(occupied, origin, resolution, (0.0, 0.0, 0.0))
    facing_neg = sensor.sample_points(occupied, origin, resolution, (0.0, 0.0, math.pi))
    assert facing_pos.shape[0] > 0
    assert float(np.min(facing_pos[:, 0])) > 0.0
    # Wall sits ~1 m ahead when facing +X; opposite yaw should not see that slab.
    wall_band = lambda cloud: int(np.sum((cloud[:, 0] > 0.8) & (cloud[:, 0] < 1.3)))
    assert wall_band(facing_pos) > wall_band(facing_neg)


def test_fake_sensor_occludes_far_voxel():
    """Near wall wins over a farther wall along the same ray (occlusion)."""
    occupied = np.zeros((30, 10, 10), dtype=bool)
    occupied[10, :, :] = True
    occupied[25, :, :] = True
    origin = np.array([-1.0, -1.0, 0.0], dtype=np.float64)
    resolution = 0.2
    sensor = FakeSensor(
        {
            "width": 16,
            "height": 12,
            "hfov_deg": 40.0,
            "sensor_height": 0.5,
            "range_min": 0.05,
            "range_max": 20.0,
            "depth_points_stride": 2,
        }
    )
    points = sensor.sample_points(occupied, origin, resolution, (0.0, 0.0, 0.0))
    assert points.shape[0] > 0
    # Near wall around x≈10*0.2-1=1.0 from origin at 0 → depth ~1m class.
    assert float(np.max(points[:, 0])) < 3.5


def test_dda_does_not_tunnel_thin_shell():
    """1-voxel-thick front shell occludes a back shell (posts-style hollow)."""
    from autosim.scenario.map.volume import Volume

    occupied = np.zeros((40, 8, 8), dtype=bool)
    occupied[12, :, :] = True  # front face
    occupied[28, :, :] = True  # back face (must stay hidden)
    origin = np.array([-2.0, -0.8, 0.0], dtype=np.float64)
    resolution = 0.2
    ray_o = np.array([0.0, 0.0, 0.5], dtype=np.float64)
    dirs = np.array([[1.0, 0.0, 0.0]], dtype=np.float64)
    hits = Volume.cast_hits(
        occupied, origin, resolution, ray_o, dirs, 0.05, 20.0, ground_z=-1.0
    )
    assert np.isfinite(hits[0])
    # Front wall at voxel 12 → world x = -2 + 12*0.2 = 0.4; distance ≈ 0.4
    assert 0.2 < float(hits[0]) < 1.0
    # Must not report the back wall (~3.6 m).
    assert float(hits[0]) < 2.0


def test_traffic_cloud_includes_obstacles_and_pedestrians():
    from autosim.scenario.traffic import Traffic

    traffic = Traffic.from_block(
        {
            "x_length": 10,
            "y_length": 10,
            "spawn_radius": 0.8,
            "obstacle": {
                "enabled": True,
                "count": 2,
                "radius": 0.3,
                "height": 1.0,
                "speed_min": 0.1,
                "speed_max": 0.1,
                "seed": 1,
            },
            "pedestrian": {
                "enabled": True,
                "count": 2,
                "radius": 0.2,
                "height": 1.7,
                "speed": 0.5,
                "seed": 2,
            },
        },
        None,
    )
    cloud = traffic.cloud(0.2)
    assert cloud.shape[1] == 3
    assert cloud.shape[0] > 20
    # Obstacles and pedestrians both contribute.
    assert traffic.obstacle.surface_points(0.2).shape[0] > 0
    assert traffic.pedestrian.surface_points(0.2).shape[0] > 0


def test_config_fake_requires_scenario():
    data = Config.load(ROOT / "config" / "fake.yaml").data
    data["habitat"]["scenario"]["enabled"] = False
    with pytest.raises(ValueError, match="scenario"):
        Config.validate(data)


def test_config_fake_yaml_backend():
    settings = Config.load(ROOT / "config" / "fake.yaml")
    assert settings["habitat"]["backend"] == "fake"
    assert settings["habitat"]["scenario"]["enabled"] is True


def test_fake_runner_smoke_without_habitat(monkeypatch):
    import autosim.runner as runner_module

    monkeypatch.setattr(runner_module.time, "sleep", lambda seconds: None)
    monkeypatch.setattr(
        runner_module.SensorWorker,
        "submit",
        lambda self, fn, *args: fn(*args),
    )

    from tests.test_runner_smoke import FakeLink

    before = set(sys.modules)
    settings = Config.load(ROOT / "config" / "fake.yaml")
    settings.data["habitat"]["robot"]["control_hz"] = 20.0
    settings.data["habitat"]["map"]["rate_hz"] = 20.0
    settings.data["habitat"]["sensors"]["camera"]["rate_hz"] = 20.0
    settings.data["habitat"]["sensors"]["camera"]["width"] = 64
    settings.data["habitat"]["sensors"]["camera"]["height"] = 48
    settings.data["habitat"]["sensors"]["camera"]["depth_points_stride"] = 4

    runner = Runner(settings, max_steps=5, link=FakeLink(), simulator=None)
    assert runner.backend == "fake"
    assert runner.simulator is None
    runner.run()
    assert "habitat_sim" not in sys.modules or "habitat_sim" in before

    writers = runner.link.node.writers
    assert "/odom" in writers
    assert "/footprint" in writers
    assert "/camera/depth/points" in writers
    assert "/camera/depth/fov" in writers
    assert "/overall/map" in writers
    assert "/map" in writers
    assert len(writers["/odom"].msgs) >= 1
    assert len(writers["/footprint"].msgs) >= 1
    assert len(writers["/footprint"].msgs[0].polygon.points) >= 3
    assert len(writers["/overall/map"].msgs) >= 1
    assert len(writers["/camera/depth/points"].msgs) >= 1
    assert len(writers["/camera/depth/fov"].msgs) >= 1


def test_fake_robot_moves_on_cmd_vel():
    robot = FakeRobot(
        max_linear=1.0,
        max_angular=1.0,
        watchdog_sec=1.0,
        max_linear_accel=0.0,
        max_angular_accel=0.0,
        x=0.0,
        y=0.0,
        yaw=0.0,
    )
    robot.set_twist(0.5, 0.0, t=0.0)
    x, y, yaw = robot.step(dt=0.1, t=0.1)
    assert abs(x - 0.05) < 1e-6
    assert abs(y) < 1e-9
    assert abs(yaw) < 1e-9
