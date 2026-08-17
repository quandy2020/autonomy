from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np

try:
    import autolink
except ImportError:  # pragma: no cover
    autolink = None

from autosim.bridge import Bridge
from autosim.clock import SimClock
from autosim.codec import (
    encode_camera_info,
    encode_image,
    encode_imu,
    encode_laser_scan,
    encode_odometry,
    encode_pose_stamped,
    parse_cmd_vel,
)
from autosim.config import load_config
from autosim.drive import Drive
from autosim.imu import ImuEstimator
from autosim.lidar import Lidar
from autosim.odometry import WheelOdometry
from autosim.rgbd import Rgbd
from autosim.truth import GroundTruth
from autosim.world import create_world

from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped
from automsgs.msgs.geometry_msgs.pose_stamped_pb2 import PoseStamped
from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
from automsgs.msgs.sensor_msgs.camera_info_pb2 import CameraInfo
from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.sensor_msgs.imu_pb2 import Imu
from automsgs.msgs.sensor_msgs.laser_scan_pb2 import LaserScan


class BridgeRunner:
    def __init__(self, cfg: dict, max_steps: Optional[int] = None) -> None:
        if autolink is None:
            raise ImportError("autolink Python package is required")
        self.cfg = cfg
        self.max_steps = max_steps
        self.clock = SimClock()
        spawn = cfg["scene"]["spawn"]
        self.drive = Drive(
            max_linear=cfg["robot"]["max_linear"],
            max_angular=cfg["robot"]["max_angular"],
            watchdog_sec=cfg["watchdog_sec"],
            x=float(spawn[0]),
            y=float(spawn[1]),
            yaw=float(spawn[2]),
        )
        self.world = create_world(cfg)
        self.world.reset(*self.drive.pose())
        self.lidar = Lidar(
            angle_min=cfg["lidar"]["angle_min"],
            angle_max=cfg["lidar"]["angle_max"],
            num_beams=cfg["lidar"]["num_beams"],
            range_min=cfg["lidar"]["range_min"],
            range_max=cfg["lidar"]["range_max"],
        )
        self.rgbd = Rgbd()
        self.odom = WheelOdometry(noise_std=float(cfg["noise"]["odom"]))
        self.imu_est = ImuEstimator()
        self.imu_est.reset(yaw=self.drive.pose()[2], t=0.0)
        self.truth = GroundTruth()
        autolink.init("autosim")
        self.node = autolink.Node("autosim_bridge")
        types = {
            "cmd_vel": TwistStamped,
            "scan": LaserScan,
            "rgb": Image,
            "depth": Image,
            "camera_info": CameraInfo,
            "imu": Imu,
            "odom": Odometry,
            "gt_pose": PoseStamped,
        }
        if not cfg["truth"]["enabled"]:
            types = {k: v for k, v in types.items() if k != "gt_pose"}
        self.bridge = Bridge(self.node, cfg["channels"], types)
        self._acc = {"scan": 0.0, "rgb": 0.0, "imu": 0.0}

    def run(self) -> None:
        dt = 1.0 / float(self.cfg["rates"]["control_hz"])
        steps = 0
        try:
            while not autolink.is_shutdown():
                self._cycle(dt)
                steps += 1
                if self.max_steps is not None and steps >= self.max_steps:
                    break
                time.sleep(dt)
        finally:
            self.world.close()
            autolink.shutdown()

    def _cycle(self, dt: float) -> None:
        t = self.clock.now()
        reader = self.bridge.cmd_reader
        if hasattr(reader, "has_msg") and reader.has_msg():
            msg = reader.get_msg()
            v, w = parse_cmd_vel(msg)
            self.drive.set_twist(v, w, t=t)
        x, y, yaw = self.drive.step(dt=dt, t=t + dt)
        self.world.set_pose(x, y, yaw)
        self.world.step()
        self.clock.tick(dt)
        stamp = self.clock.stamp()
        frames = self.cfg["robot"]["frames"]
        v, w = self.drive.velocity()

        ox, oy, oyaw = self.odom.update(x, y, yaw)
        self.bridge.publish(
            "odom",
            encode_odometry(
                ox, oy, oyaw, v, w, stamp, frames["odom"], frames["base"]
            ),
        )

        self._acc["scan"] += dt
        self._acc["rgb"] += dt
        self._acc["imu"] += dt

        if self._acc["scan"] >= 1.0 / self.cfg["rates"]["scan_hz"]:
            self._acc["scan"] = 0.0
            ranges = self.lidar.sample(self.world)
            self.bridge.publish(
                "scan",
                encode_laser_scan(
                    ranges=ranges,
                    stamp=stamp,
                    frame_id=frames["laser"],
                    angle_min=self.lidar.angle_min,
                    angle_max=self.lidar.angle_max,
                    angle_increment=self.lidar.angle_increment,
                    range_min=self.lidar.range_min,
                    range_max=self.lidar.range_max,
                    scan_time=1.0 / self.cfg["rates"]["scan_hz"],
                ),
            )

        if self._acc["rgb"] >= 1.0 / self.cfg["rates"]["rgb_hz"]:
            self._acc["rgb"] = 0.0
            rgb, depth = self.rgbd.sample(self.world)
            self.bridge.publish(
                "rgb",
                encode_image(rgb, stamp, frames["camera"], "rgb8"),
            )
            # 16UC1-ish: publish float depth as 32FC1 bytes
            self.bridge.publish(
                "depth",
                encode_image(depth.astype(np.float32), stamp, frames["camera"], "32FC1"),
            )
            h, w_img = rgb.shape[:2]
            fx = fy = 0.5 * w_img
            cx, cy = 0.5 * w_img, 0.5 * h
            k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            self.bridge.publish(
                "camera_info",
                encode_camera_info(stamp, frames["camera"], w_img, h, k),
            )

        if self._acc["imu"] >= 1.0 / self.cfg["rates"]["imu_hz"]:
            self._acc["imu"] = 0.0
            gyro_z, ax, ay = self.imu_est.update(yaw, v, self.clock.now())
            half = 0.5 * yaw
            quat = (0.0, 0.0, math.sin(half), math.cos(half))
            self.bridge.publish(
                "imu",
                encode_imu(stamp, frames["imu"], quat, (0.0, 0.0, gyro_z), (ax, ay, 9.81)),
            )

        if self.cfg["truth"]["enabled"]:
            tx, ty, tyaw = self.truth.sample(x, y, yaw)
            self.bridge.publish(
                "gt_pose",
                encode_pose_stamped(tx, ty, tyaw, stamp, frames["map"]),
            )


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(prog="autosim")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "default.yaml",
    )
    args = parser.parse_args(argv)
    cfg = load_config(args.config)
    BridgeRunner(cfg).run()


if __name__ == "__main__":
    main()
