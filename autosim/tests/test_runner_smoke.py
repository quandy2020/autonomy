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

from autosim.config import Config
from autosim.runner import Runner
from tests.fake_simulator import FakeSimulator
from autosim.urdf import UrdfModel


class FakeWriter:
    def __init__(self):
        self.msgs = []

    def write(self, msg):
        self.msgs.append(msg)


class FakeReader:
    def __init__(self):
        self.message = None
        self.has = False

    def set(self, msg):
        self.message = msg
        self.has = True

    def has_msg(self):
        return self.has

    def get_msg(self):
        self.has = False
        return self.message


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.writers = {}
        self.readers = {}

    def create_writer(self, channel, dtype, qos_depth=1):
        writer = FakeWriter()
        self.writers[channel] = writer
        return writer

    def create_reader(self, channel, dtype, qos_depth=1):
        reader = FakeReader()
        self.readers[channel] = reader
        return reader


class FakeLink:
    def __init__(self):
        self.node = None

    def init(self, name):
        return None

    def shutdown(self):
        return None

    def is_shutdown(self):
        return False

    def Node(self, name):
        self.node = FakeNode(name)
        return self.node


def test_runner_publishes_scan_and_points_when_enabled(monkeypatch):
    import autosim.runner as runner_module

    monkeypatch.setattr(runner_module.time, "sleep", lambda seconds: None)

    root = Path(__file__).resolve().parents[1]
    settings = Config.load(root / "config" / "default.yaml")
    sensors = settings.data["habitat"]["sensors"]
    sensors["lidar_2d"]["enabled"] = True
    sensors["lidar_2d"]["num_beams"] = 16
    sensors["lidar_3d"]["enabled"] = True
    sensors["lidar_3d"]["horizontal"]["num_beams"] = 8
    sensors["lidar_3d"]["vertical"]["num_rings"] = 2
    sensors["camera"]["enabled"] = False
    sensors["imu"]["enabled"] = False
    sensors["odom"]["enabled"] = True
    settings.data["habitat"]["map"]["enabled"] = True
    settings.data["habitat"]["map"]["rate_hz"] = 0.0
    settings.data["habitat"]["map"]["ply"]["file"] = ""
    settings.data["habitat"]["map"]["ply"]["channel"] = "/map/points"
    settings.data["habitat"]["map"]["ply"]["horizontal"]["num_beams"] = 8
    settings.data["habitat"]["map"]["ply"]["vertical"]["num_rings"] = 2
    settings.data["habitat"]["robot"]["tf"]["enabled"] = True
    settings.data["habitat"]["robot"]["clock"]["enabled"] = True

    link = FakeLink()
    simulator = FakeSimulator()
    simulator.urdf = UrdfModel.load("urdf/husky.urdf")
    runner = Runner(settings, max_steps=5, link=link, simulator=simulator)
    runner.run()
    assert "/scan" in runner.node.writers
    assert len(runner.node.writers["/scan"].msgs) >= 1
    assert "/points" in runner.node.writers
    assert len(runner.node.writers["/points"].msgs) >= 1
    assert len(runner.node.writers["/odom"].msgs) >= 1
    assert "/map/points" in runner.node.writers
    assert "/map" in runner.node.writers
    assert len(runner.node.writers["/map/points"].msgs) >= 1
    assert len(runner.node.writers["/map"].msgs) >= 1
    assert "/tf" in runner.node.writers
    assert "/tf_static" in runner.node.writers
    assert "/clock" in runner.node.writers
    assert "/footprint" in runner.node.writers
    assert len(runner.node.writers["/tf"].msgs) >= 1
    odom_msg = runner.node.writers["/odom"].msgs[0]
    assert odom_msg.child_frame_id == "base_footprint"
    footprint = runner.node.writers["/footprint"].msgs[0]
    assert footprint.header.frame_id == "base_footprint"
    assert len(footprint.polygon.points) == 4
    assert abs(footprint.polygon.points[0].x - 0.4935) < 1e-6
    assert abs(footprint.polygon.points[0].y - 0.285) < 1e-6
    tf_edges = {
        (item.header.frame_id, item.child_frame_id): (
            item.transform.translation.x,
            item.transform.translation.y,
            item.transform.translation.z,
        )
        for message in runner.node.writers["/tf"].msgs
        for item in message.transforms
    }
    assert ("map", "odom") in tf_edges
    assert ("odom", "base_footprint") in tf_edges
    assert ("base_footprint", "base_link") in tf_edges
    latest_tf = runner.node.writers["/tf"].msgs[-1]
    stamps = {
        (item.header.stamp.sec, item.header.stamp.nanosec)
        for item in latest_tf.transforms
    }
    assert len(stamps) == 1
    latest_edges = {
        (item.header.frame_id, item.child_frame_id)
        for item in latest_tf.transforms
    }
    assert ("odom", "base_footprint") in latest_edges
    assert ("base_footprint", "base_link") in latest_edges
    static_tf = runner.node.writers["/tf_static"].msgs[0]
    edges = {
        (item.header.frame_id, item.child_frame_id): (
            item.transform.translation.x,
            item.transform.translation.y,
            item.transform.translation.z,
        )
        for item in static_tf.transforms
    }
    assert ("base_footprint", "base_link") in edges
    assert ("base_link", "base_scan") in edges
    assert ("base_scan", "laser_link") in edges
    assert ("base_link", "imu_link") in edges
    assert ("base_link", "front_left_wheel_link") in edges
    assert ("base_link", "front_right_wheel_link") in edges
    assert abs(edges[("base_footprint", "base_link")][2] - 0.165) < 1e-6
    assert abs(edges[("base_link", "base_scan")][0] - 0.15) < 1e-6
    assert abs(edges[("base_link", "base_scan")][2] - 0.3) < 1e-6
    assert abs(edges[("base_link", "imu_link")][0] - 0.1) < 1e-6
    assert abs(edges[("base_link", "imu_link")][2] - 0.18) < 1e-6
    assert "/camera/rgb/image_raw" not in runner.node.writers


def test_runner_skips_odom_when_disabled(monkeypatch):
    import autosim.runner as runner_module

    monkeypatch.setattr(runner_module.time, "sleep", lambda seconds: None)

    root = Path(__file__).resolve().parents[1]
    settings = Config.load(root / "config" / "default.yaml")
    sensors = settings.data["habitat"]["sensors"]
    sensors["lidar_2d"]["enabled"] = False
    sensors["lidar_3d"]["enabled"] = False
    sensors["camera"]["enabled"] = False
    sensors["imu"]["enabled"] = False
    sensors["odom"]["enabled"] = False
    settings.data["habitat"]["map"]["enabled"] = False
    settings.data["habitat"]["robot"]["tf"]["enabled"] = False
    settings.data["habitat"]["robot"]["clock"]["enabled"] = False

    link = FakeLink()
    runner = Runner(settings, max_steps=3, link=link, simulator=FakeSimulator())
    runner.run()
    assert "/odom" not in runner.node.writers
    assert "/map" not in runner.node.writers


def test_subsample_points():
    import numpy as np

    cloud = np.arange(12, dtype=np.float32).reshape(4, 3)
    subsampled = Runner.subsample_points(cloud, 2)
    assert subsampled.shape == (2, 3)
    np.testing.assert_allclose(subsampled[0], cloud[0])
    np.testing.assert_allclose(subsampled[1], cloud[2])
