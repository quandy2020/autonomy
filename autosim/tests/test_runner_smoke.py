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
    # Run encode/publish inline so assertions are not racing the worker queue.
    monkeypatch.setattr(
        runner_module.SensorWorker,
        "submit",
        lambda self, fn, *args: fn(*args),
    )

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
    settings.data["habitat"]["mode"] = "nav"
    settings.data["habitat"]["map"]["enabled"] = True
    settings.data["habitat"]["map"]["publish"] = True
    settings.data["habitat"]["map"]["rate_hz"] = 0.0
    settings.data["habitat"]["map"]["ply"]["file"] = ""
    settings.data["habitat"]["map"]["ply"]["source"] = ""
    settings.data["habitat"]["map"]["ply"]["channel"] = "/map/points"
    settings.data["habitat"]["map"]["ply"]["range_max"] = 10.0
    settings.data["habitat"]["map"]["ply"]["horizontal"] = {
        "angle_min": -1.0,
        "angle_max": 1.0,
        "num_beams": 8,
    }
    settings.data["habitat"]["map"]["ply"]["vertical"] = {
        "angle_min": -0.1,
        "angle_max": 0.1,
        "num_rings": 2,
    }
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
    assert odom_msg.child_frame_id == "base_link"
    footprint = runner.node.writers["/footprint"].msgs[0]
    assert footprint.header.frame_id == "base_link"
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
    assert ("odom", "base_link") in tf_edges
    assert ("odom", "base_footprint") not in tf_edges
    assert ("base_link", "base_footprint") in tf_edges
    assert ("base_link", "base_scan") in tf_edges or ("base_link", "laser_link") in tf_edges
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
    assert ("map", "odom") in latest_edges
    assert ("odom", "base_link") in latest_edges
    static_tf = runner.node.writers["/tf_static"].msgs[0]
    edges = {
        (item.header.frame_id, item.child_frame_id): (
            item.transform.translation.x,
            item.transform.translation.y,
            item.transform.translation.z,
        )
        for item in static_tf.transforms
    }
    # map→odom is dynamic (odom drift correction); must not be latched on /tf_static.
    assert ("map", "odom") not in edges
    assert ("base_link", "base_footprint") in edges
    assert ("base_link", "base_scan") in edges
    assert ("base_scan", "laser_link") in edges
    assert ("base_link", "imu_link") in edges
    assert ("base_link", "front_left_wheel_link") in edges
    assert ("base_link", "front_right_wheel_link") in edges
    assert abs(edges[("base_link", "base_footprint")][2] + 0.165) < 1e-6
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


def test_runner_camera_rgb_depth_share_stamp(monkeypatch):
    """Every published RGB frame must share stamp with the paired depth frame."""
    import autosim.runner as runner_module

    monkeypatch.setattr(runner_module.time, "sleep", lambda seconds: None)

    root = Path(__file__).resolve().parents[1]
    settings = Config.load(root / "config" / "default.yaml")
    settings.data["habitat"]["mode"] = "slam"
    settings.data["habitat"]["map"]["enabled"] = False
    sensors = settings.data["habitat"]["sensors"]
    sensors["lidar_2d"]["enabled"] = False
    sensors["lidar_3d"]["enabled"] = False
    sensors["camera"]["enabled"] = True
    sensors["camera"]["rate_hz"] = 50.0
    sensors["imu"]["enabled"] = False
    sensors["odom"]["enabled"] = False
    settings.data["habitat"]["robot"]["tf"]["enabled"] = False
    settings.data["habitat"]["robot"]["clock"]["enabled"] = False
    settings.data["habitat"]["robot"]["footprint"]["enabled"] = False

    link = FakeLink()
    runner = Runner(settings, max_steps=40, link=link, simulator=FakeSimulator())
    runner.run()
    rgb_msgs = runner.node.writers["/camera/rgb/image_raw"].msgs
    depth_msgs = runner.node.writers["/camera/depth/image_raw"].msgs
    info_msgs = runner.node.writers["/camera/camera_info"].msgs
    assert len(rgb_msgs) >= 1
    assert len(rgb_msgs) == len(depth_msgs) == len(info_msgs)
    for rgb, depth, info in zip(rgb_msgs, depth_msgs, info_msgs):
        assert (rgb.header.stamp.sec, rgb.header.stamp.nanosec) == (
            depth.header.stamp.sec,
            depth.header.stamp.nanosec,
        )
        assert (rgb.header.stamp.sec, rgb.header.stamp.nanosec) == (
            info.header.stamp.sec,
            info.header.stamp.nanosec,
        )


def test_map_grid_published_when_sensor_worker_drops(monkeypatch):
    """OccupancyGrid must publish even if the async sensor queue is saturated."""
    import autosim.runner as runner_module

    monkeypatch.setattr(runner_module.time, "sleep", lambda seconds: None)

    root = Path(__file__).resolve().parents[1]
    settings = Config.load(root / "config" / "default.yaml")
    sensors = settings.data["habitat"]["sensors"]
    sensors["lidar_2d"]["enabled"] = True
    sensors["lidar_2d"]["num_beams"] = 8
    sensors["lidar_3d"]["enabled"] = False
    sensors["camera"]["enabled"] = False
    sensors["imu"]["enabled"] = False
    sensors["odom"]["enabled"] = False
    settings.data["habitat"]["mode"] = "nav"
    settings.data["habitat"]["map"]["enabled"] = True
    settings.data["habitat"]["map"]["publish"] = True
    settings.data["habitat"]["map"]["rate_hz"] = 0.0
    settings.data["habitat"]["map"]["ply"]["source"] = ""
    settings.data["habitat"]["map"]["ply"]["file"] = ""
    settings.data["habitat"]["map"]["ply"]["channel"] = "/map/points"
    settings.data["habitat"]["map"]["ply"]["range_max"] = 10.0
    settings.data["habitat"]["map"]["ply"]["horizontal"] = {
        "angle_min": -1.0,
        "angle_max": 1.0,
        "num_beams": 4,
    }
    settings.data["habitat"]["map"]["ply"]["vertical"] = {
        "angle_min": -0.1,
        "angle_max": 0.1,
        "num_rings": 2,
    }
    settings.data["habitat"]["robot"]["tf"]["enabled"] = False
    settings.data["habitat"]["robot"]["clock"]["enabled"] = False
    settings.data["habitat"]["robot"]["footprint"]["enabled"] = False

    link = FakeLink()
    runner = Runner(settings, max_steps=1, link=link, simulator=FakeSimulator())
    runner._sensor_worker.submit = lambda fn, *args: None  # drop async cloud work
    runner.run()
    assert "/map" in runner.node.writers
    assert len(runner.node.writers["/map"].msgs) >= 1


def test_runner_slam_skips_map_and_map_odom(monkeypatch):
    """SLAM mode must not publish GT /map or identity map→odom (Cartographer owns both)."""
    import autosim.runner as runner_module

    monkeypatch.setattr(runner_module.time, "sleep", lambda seconds: None)

    root = Path(__file__).resolve().parents[1]
    settings = Config.load(root / "config" / "default.yaml")
    settings.data["habitat"]["mode"] = "slam"
    sensors = settings.data["habitat"]["sensors"]
    sensors["lidar_2d"]["enabled"] = True
    sensors["lidar_2d"]["num_beams"] = 8
    sensors["lidar_3d"]["enabled"] = False
    sensors["camera"]["enabled"] = False
    sensors["imu"]["enabled"] = False
    # Keep map.enabled for internal occupancy; publishing must stay off in slam.
    settings.data["habitat"]["map"]["enabled"] = True
    settings.data["habitat"]["map"]["publish"] = True  # must be ignored under mode: slam
    settings.data["habitat"]["map"]["ply"]["source"] = ""
    settings.data["habitat"]["map"]["ply"]["file"] = ""
    settings.data["habitat"]["map"]["ply"]["channel"] = "/overall/map"
    settings.data["habitat"]["map"]["ply"]["range_max"] = 10.0
    settings.data["habitat"]["map"]["ply"]["horizontal"] = {
        "angle_min": -1.0,
        "angle_max": 1.0,
        "num_beams": 4,
    }
    settings.data["habitat"]["map"]["ply"]["vertical"] = {
        "angle_min": -0.1,
        "angle_max": 0.1,
        "num_rings": 2,
    }
    settings.data["habitat"]["robot"]["tf"]["enabled"] = True
    settings.data["habitat"]["robot"]["tf"]["publish_map_odom"] = True  # ignored in slam
    settings.data["habitat"]["robot"]["clock"]["enabled"] = False
    settings.data["habitat"]["robot"]["footprint"]["enabled"] = False

    link = FakeLink()
    runner = Runner(settings, max_steps=3, link=link, simulator=FakeSimulator())
    runner.run()
    assert runner.mode == "slam"
    assert runner.map_publish is False
    assert "/map" not in runner.node.writers
    assert "/overall/map" not in runner.node.writers
    assert "/tf" in runner.node.writers
    tf_edges = {
        (item.header.frame_id, item.child_frame_id)
        for message in runner.node.writers["/tf"].msgs
        for item in message.transforms
    }
    assert ("map", "odom") not in tf_edges
    assert ("odom", "base_link") in tf_edges
    assert ("odom", "base_footprint") not in tf_edges



def test_runner_slam_glues_odom_to_ground_truth(monkeypatch):
    """SLAM must keep wheel odom == GT so GT-cast /scan matches odom→base TF."""
    import autosim.runner as runner_module

    monkeypatch.setattr(runner_module.time, "sleep", lambda seconds: None)

    root = Path(__file__).resolve().parents[1]
    settings = Config.load(root / "config" / "default.yaml")
    settings.data["habitat"]["mode"] = "slam"
    settings.data["habitat"]["map"]["enabled"] = False
    sensors = settings.data["habitat"]["sensors"]
    # Intentional mismatch on odom noise only; lidar stays clean (yaml noise=0).
    sensors["lidar_2d"]["enabled"] = False
    sensors["lidar_3d"]["enabled"] = False
    sensors["camera"]["enabled"] = False
    sensors["imu"]["enabled"] = False
    sensors["odom"]["enabled"] = True
    sensors["odom"]["noise"] = 0.05
    settings.data["habitat"]["robot"]["tf"]["enabled"] = True
    settings.data["habitat"]["robot"]["clock"]["enabled"] = False
    settings.data["habitat"]["robot"]["footprint"]["enabled"] = False

    link = FakeLink()
    runner = Runner(settings, max_steps=20, link=link, simulator=FakeSimulator())
    assert float(runner.lidar_2d.get("noise", 0.0)) == 0.0
    assert runner.lidar_2d.get("frame", "laser_link") == "laser_link"
    runner.robot.set_twist(0.3, 0.5, 0.0)
    runner.run()
    gx, gy, gyaw = runner.robot.pose()
    ox, oy, oyaw = runner.robot.odometry_pose()
    assert abs(gx - ox) < 1e-9
    assert abs(gy - oy) < 1e-9
    assert abs(gyaw - oyaw) < 1e-9


def test_subsample_points():
    import numpy as np

    cloud = np.arange(12, dtype=np.float32).reshape(4, 3)
    subsampled = Runner.subsample_points(cloud, 2)
    assert subsampled.shape == (2, 3)
    np.testing.assert_allclose(subsampled[0], cloud[0])
    np.testing.assert_allclose(subsampled[1], cloud[2])
