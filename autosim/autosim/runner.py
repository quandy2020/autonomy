"""Main loop: orchestrate robot, sensors, simulator, and communication bridge."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np

from autosim.bridge import Bridge
from autosim.clock import Clock
from autosim.config import Config
from autosim.messages import Messages
from autosim.robot import Robot
from autosim.sensors import Sensors
from autosim.simulator import Simulator

from automsgs.msgs.geometry_msgs.pose_stamped_pb2 import PoseStamped
from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped
from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
from automsgs.msgs.sensor_msgs.camera_info_pb2 import CameraInfo
from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.sensor_msgs.imu_pb2 import Imu
from automsgs.msgs.sensor_msgs.laser_scan_pb2 import LaserScan
from automsgs.msgs.sensor_msgs.point_cloud2_pb2 import PointCloud2


class Runner:
    """Fixed-rate sensor–actuator bridge main loop."""

    def __init__(
        self,
        settings: Config,
        max_steps: Optional[int] = None,
        link: Any = None,
        simulator: Optional[Simulator] = None,
    ) -> None:
        """Assemble runtime dependencies.

        Args:
            settings: Loaded :class:`~autosim.config.Config`.
            max_steps: Optional max iterations; ``None`` runs until shutdown.
            link: Injectable autolink module/stub (tests); defaults to ``import autolink``.
            simulator: Injectable :class:`~autosim.simulator.Simulator`; created from config if omitted.

        Raises:
            ImportError: ``link`` was not provided and autolink cannot be imported.
        """
        if link is None:
            try:
                import autolink as link
            except ImportError as exc:
                raise ImportError("autolink Python package is required") from exc

        self.link = link
        self.settings = settings
        self.max_steps = max_steps
        self.clock = Clock()

        robot_cfg = settings["habitat"]["robot"]
        sensors_cfg = settings["habitat"]["sensors"]
        lidar_2d = sensors_cfg["lidar_2d"]
        lidar_3d = sensors_cfg["lidar_3d"]
        camera_cfg = sensors_cfg["camera"]
        imu_cfg = sensors_cfg["imu"]
        odom_cfg = sensors_cfg["odom"]

        spawn = settings["habitat"]["spawn"]
        self.robot = Robot(
            max_linear=robot_cfg["max_linear"],
            max_angular=robot_cfg["max_angular"],
            watchdog_sec=robot_cfg["watchdog_sec"],
            odometry_noise=float(odom_cfg["noise"]),
            x=float(spawn[0]),
            y=float(spawn[1]),
            yaw=float(spawn[2]),
        )
        self.simulator = simulator if simulator is not None else Simulator.create(settings.data)
        self.simulator.reset(*self.robot.pose())
        self.sensors = Sensors(
            angle_min=lidar_2d["angle_min"],
            angle_max=lidar_2d["angle_max"],
            num_beams=lidar_2d["num_beams"],
            range_min=lidar_2d["range_min"],
            range_max=lidar_2d["range_max"],
            noise=float(lidar_2d.get("noise", 0.0)),
            lidar_3d=lidar_3d,
        )
        self.lidar_2d = lidar_2d
        self.lidar_3d = lidar_3d
        self.camera = camera_cfg
        self.imu = imu_cfg
        self.odom = odom_cfg
        self.robot_cfg = robot_cfg

        self.robot.reset_inertial(yaw=self.robot.pose()[2], t=0.0)
        self.link.init("autosim")
        self.node = self.link.Node("autosim_bridge")
        types: dict[str, Any] = {
            "cmd_vel": TwistStamped,
            "odom": Odometry,
        }
        if lidar_2d["enabled"]:
            types["scan"] = LaserScan
        if lidar_3d["enabled"]:
            types["points"] = PointCloud2
        if camera_cfg.get("enabled", True):
            types["rgb"] = Image
            types["depth"] = Image
            types["camera_info"] = CameraInfo
        if imu_cfg.get("enabled", True):
            types["imu"] = Imu
        if robot_cfg["truth"]["enabled"]:
            types["gt_pose"] = PoseStamped
        self.bridge = Bridge(self.node, settings.channel_map(), types)
        self.scan_elapsed = 0.0
        self.points_elapsed = 0.0
        self.camera_elapsed = 0.0
        self.inertial_elapsed = 0.0

    def run(self) -> None:
        """Loop at the control rate until shutdown or ``max_steps``.

        On exit, closes the simulation session and calls ``link.shutdown()``.
        """
        dt = 1.0 / float(self.robot_cfg["control_hz"])
        steps = 0
        try:
            while not self.link.is_shutdown():
                self.cycle(dt)
                steps += 1
                if self.max_steps is not None and steps >= self.max_steps:
                    break
                time.sleep(dt)
        finally:
            self.simulator.close()
            self.link.shutdown()

    def cycle(self, dt: float) -> None:
        """Run one control cycle: read commands, integrate, sample, and publish by rate.

        Args:
            dt: Cycle duration in seconds, typically ``1 / control_hz``.
        """
        t = self.clock.now()
        reader = self.bridge.command_reader
        if hasattr(reader, "has_msg") and reader.has_msg():
            message = reader.get_msg()
            linear, angular = Messages.parse_command_velocity(message)
            self.robot.set_twist(linear, angular, t=t)
        x, y, yaw = self.robot.step(dt=dt, t=t + dt)
        self.simulator.set_pose(x, y, yaw)
        self.simulator.step()
        self.clock.tick(dt)
        stamp = self.clock.stamp()
        linear, angular = self.robot.velocity()

        ox, oy, oyaw = self.robot.update_odometry(x, y, yaw)
        self.bridge.publish(
            "odom",
            Messages.encode_odometry(
                ox,
                oy,
                oyaw,
                linear,
                angular,
                stamp,
                self.odom["frame"],
                self.odom["child_frame"],
            ),
        )

        self.scan_elapsed += dt
        self.points_elapsed += dt
        self.camera_elapsed += dt
        self.inertial_elapsed += dt

        if self.lidar_2d["enabled"] and self.scan_elapsed >= 1.0 / self.lidar_2d["rate_hz"]:
            self.scan_elapsed = 0.0
            ranges = self.sensors.sample_laser(self.simulator)
            self.bridge.publish(
                "scan",
                Messages.encode_laser_scan(
                    ranges=ranges,
                    stamp=stamp,
                    frame_id=self.lidar_2d["frame"],
                    angle_min=self.sensors.angle_min,
                    angle_max=self.sensors.angle_max,
                    angle_increment=self.sensors.angle_increment,
                    range_min=self.sensors.range_min,
                    range_max=self.sensors.range_max,
                    scan_time=1.0 / self.lidar_2d["rate_hz"],
                ),
            )

        if self.lidar_3d["enabled"] and self.points_elapsed >= 1.0 / self.lidar_3d["rate_hz"]:
            self.points_elapsed = 0.0
            points = self.sensors.sample_points(self.simulator)
            self.bridge.publish(
                "points",
                Messages.encode_point_cloud2(points, stamp, self.lidar_3d["frame"]),
            )

        if self.camera.get("enabled", True) and self.camera_elapsed >= 1.0 / self.camera["rate_hz"]:
            self.camera_elapsed = 0.0
            color, depth = self.sensors.sample_camera(self.simulator)
            self.bridge.publish(
                "rgb",
                Messages.encode_image(color, stamp, self.camera["frame"], "rgb8"),
            )
            self.bridge.publish(
                "depth",
                Messages.encode_image(
                    depth.astype(np.float32), stamp, self.camera["frame"], "32FC1"
                ),
            )
            height, width = color.shape[:2]
            fx = fy = 0.5 * width
            cx, cy = 0.5 * width, 0.5 * height
            camera_matrix = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            self.bridge.publish(
                "camera_info",
                Messages.encode_camera_info(
                    stamp, self.camera["frame"], width, height, camera_matrix
                ),
            )

        if self.imu.get("enabled", True) and self.inertial_elapsed >= 1.0 / self.imu["rate_hz"]:
            self.inertial_elapsed = 0.0
            gyro_z, accel_x, accel_y = self.robot.update_inertial(
                yaw, linear, self.clock.now()
            )
            half = 0.5 * yaw
            orientation = (0.0, 0.0, math.sin(half), math.cos(half))
            self.bridge.publish(
                "imu",
                Messages.encode_inertial(
                    stamp,
                    self.imu["frame"],
                    orientation,
                    (0.0, 0.0, gyro_z),
                    (accel_x, accel_y, 9.81),
                ),
            )

        if self.robot_cfg["truth"]["enabled"]:
            tx, ty, tyaw = self.robot.ground_truth()
            self.bridge.publish(
                "gt_pose",
                Messages.encode_pose_stamped(
                    tx, ty, tyaw, stamp, self.robot_cfg["map_frame"]
                ),
            )

    @staticmethod
    def main(argv: Optional[list[str]] = None) -> None:
        """CLI entry: parse ``--config`` and start :class:`Runner`.

        Args:
            argv: Optional argument list; uses ``sys.argv`` when ``None``.
        """
        parser = argparse.ArgumentParser(prog="autosim")
        parser.add_argument(
            "--config",
            type=Path,
            default=Path(__file__).resolve().parents[1] / "config" / "default.yaml",
        )
        args = parser.parse_args(argv)
        settings = Config.load(args.config)
        Runner(settings).run()
