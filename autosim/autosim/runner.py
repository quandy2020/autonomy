"""Fixed-rate control loop for the Habitat sensor–actuator bridge.

Wires :class:`~autosim.config.Config`, :class:`~autosim.robot.Robot`,
:class:`~autosim.sensors.Sensors`, :class:`~autosim.simulator.Simulator`,
and :class:`~autosim.bridge.Bridge` into one publish/subscribe cycle.
"""

from __future__ import annotations

import argparse
import math
import time
import warnings
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np

from autosim.bridge import Bridge
from autosim.clock import Clock
from autosim.config import Config
from autosim.map import Map
from autosim.messages import Messages
from autosim.robot import Robot
from autosim.sensors import Sensors
from autosim.simulator import Simulator

from automsgs.msgs.geometry_msgs.pose_stamped_pb2 import PoseStamped
from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped
from automsgs.msgs.map_msgs.occupancy_grid_pb2 import OccupancyGrid
from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
from automsgs.msgs.rosgraph_msgs.clock_pb2 import Clock as ClockMsg
from automsgs.msgs.sensor_msgs.camera_info_pb2 import CameraInfo
from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.sensor_msgs.imu_pb2 import Imu
from automsgs.msgs.sensor_msgs.laser_scan_pb2 import LaserScan
from automsgs.msgs.sensor_msgs.point_cloud2_pb2 import PointCloud2
from automsgs.msgs.tf2_msgs.tf_message_pb2 import TFMessage


class Runner:
    """Sensor–actuator bridge driven at ``control_hz``."""

    def __init__(
        self,
        settings: Config,
        max_steps: Optional[int] = None,
        link: Any = None,
        simulator: Optional[Simulator] = None,
    ) -> None:
        """Assemble plant, sensors, and autolink I/O.

        Args:
            settings: Validated :class:`~autosim.config.Config`.
            max_steps: Stop after N cycles; ``None`` runs until shutdown.
            link: Injected autolink module (tests); otherwise ``import autolink``.
            simulator: Injected :class:`~autosim.simulator.Simulator`; else built from config.

        Raises:
            ImportError: autolink is missing and ``link`` was not provided.
        """
        self.link = self.resolve_link(link)
        self.settings = settings
        self.max_steps = max_steps
        self.clock = Clock()
        self.apply_settings(settings)
        self.assemble_plant(simulator)
        self.bind_bridge()
        self.scan_elapsed = 0.0
        self.points_elapsed = 0.0
        self.camera_elapsed = 0.0
        self.inertial_elapsed = 0.0
        self.map_elapsed = 0.0
        self.map_published = False
        self.map_builder: Optional[Map] = None
        if self.map_cfg.get("enabled", False):
            self.map_builder = Map(self.map_cfg)

    @staticmethod
    def resolve_link(link: Any) -> Any:
        """Return ``link`` or import the autolink package.

        Args:
            link: Optional pre-bound autolink module.

        Returns:
            Usable autolink module.

        Raises:
            ImportError: Package not installed.
        """
        if link is not None:
            return link
        try:
            import autolink as resolved
        except ImportError as exc:
            raise ImportError("autolink Python package is required") from exc
        return resolved

    def apply_settings(self, settings: Config) -> None:
        """Cache nested habitat robot and sensor blocks.

        Args:
            settings: Loaded configuration.
        """
        sensors = settings["habitat"]["sensors"]
        self.robot_cfg = settings["habitat"]["robot"]
        self.lidar_2d = sensors["lidar_2d"]
        self.lidar_3d = sensors["lidar_3d"]
        self.camera = sensors["camera"]
        self.imu = sensors["imu"]
        self.odom = sensors["odom"]
        self.spawn = settings["habitat"]["spawn"]
        self.map_cfg = settings["habitat"].get("map") or {"enabled": False}
        self.tf_cfg = self.robot_cfg.get("tf") or {"enabled": False}
        self.clock_cfg = self.robot_cfg.get("clock") or {"enabled": False}
        self.tf_static_sent = False

    def assemble_plant(self, simulator: Optional[Simulator]) -> None:
        """Construct robot, simulator, and sensor sampler.

        Args:
            simulator: Optional injected backend; otherwise :meth:`Simulator.create`.
        """
        imu_noise = self.imu.get("noise") or {}
        self.robot = Robot(
            max_linear=self.robot_cfg["max_linear"],
            max_angular=self.robot_cfg["max_angular"],
            watchdog_sec=self.robot_cfg["watchdog_sec"],
            odometry_noise=float(self.odom.get("noise", 0.0)),
            gyro_noise=float(imu_noise.get("gyro", 0.0)),
            accel_noise=float(imu_noise.get("accel", 0.0)),
            gyro_bias=float(imu_noise.get("gyro_bias", 0.0)),
            accel_bias=float(imu_noise.get("accel_bias", 0.0)),
            wheel_separation=float(self.robot_cfg.get("wheel_separation", 0.5)),
            x=float(self.spawn[0]),
            y=float(self.spawn[1]),
            yaw=float(self.spawn[2]),
        )
        self.simulator = simulator if simulator is not None else Simulator.create(
            self.settings.data
        )
        self.simulator.reset(*self.robot.pose())
        cam_noise = self.camera.get("noise") or {}
        self.sensors = Sensors(
            angle_min=self.lidar_2d["angle_min"],
            angle_max=self.lidar_2d["angle_max"],
            num_beams=self.lidar_2d["num_beams"],
            range_min=self.lidar_2d["range_min"],
            range_max=self.lidar_2d["range_max"],
            noise=float(self.lidar_2d.get("noise", 0.0)),
            depth_noise=float(cam_noise.get("depth", 0.0)),
            lidar_3d=self.lidar_3d,
        )
        self.robot.reset_inertial(yaw=self.robot.pose()[2], t=0.0)

    def message_types(self) -> Dict[str, Any]:
        """Build logical-key → protobuf type map for enabled streams.

        Returns:
            Writer/reader type dictionary for :class:`~autosim.bridge.Bridge`.
        """
        types: Dict[str, Any] = {"cmd_vel": TwistStamped}
        if self.odom.get("enabled", True):
            types["odom"] = Odometry
        if self.lidar_2d["enabled"]:
            types["scan"] = LaserScan
        if self.lidar_3d["enabled"]:
            types["points"] = PointCloud2
        if self.camera.get("enabled", False):
            types["rgb"] = Image
            types["depth"] = Image
            types["camera_info"] = CameraInfo
        if self.imu.get("enabled", False):
            types["imu"] = Imu
        if self.robot_cfg["truth"]["enabled"]:
            types["gt_pose"] = PoseStamped
        if self.map_cfg.get("enabled", False):
            if str((self.map_cfg.get("ply") or {}).get("channel") or "").strip():
                types["map_cloud"] = PointCloud2
            types["map_grid"] = OccupancyGrid
        if self.tf_cfg.get("enabled", False):
            types["tf"] = TFMessage
            types["tf_static"] = TFMessage
        if self.clock_cfg.get("enabled", False):
            types["clock"] = ClockMsg
        return types

    def bind_bridge(self) -> None:
        """Initialize autolink and attach the channel bridge."""
        self.link.init("autosim")
        self.node = self.link.Node("autosim_bridge")
        self.bridge = Bridge(self.node, self.settings.channel_map(), self.message_types())

    def run(self) -> None:
        """Spin at ``control_hz`` until shutdown or ``max_steps``.

        Always closes the simulator and calls ``link.shutdown()`` on exit.
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
        """Execute one control period.

        Args:
            dt: Period length in seconds (``1 / control_hz``).
        """
        self.poll_command()
        x, y, yaw = self.robot.step(dt=dt, t=self.clock.now() + dt)
        self.simulator.set_pose(x, y, yaw)
        self.simulator.step()
        self.clock.tick(dt)
        stamp = self.clock.stamp()
        linear, angular = self.robot.velocity()
        self.advance_timers(dt)
        self.emit("odom", self.publish_odometry, x, y, yaw, linear, angular, stamp)
        self.emit("scan", self.publish_scan, stamp)
        self.emit("cloud", self.publish_cloud, stamp)
        self.emit("camera", self.publish_camera, stamp)
        self.emit("imu", self.publish_imu, yaw, linear, stamp)
        self.emit("truth", self.publish_truth, stamp)
        self.emit("map", self.publish_map, stamp)
        self.emit("tf", self.publish_tf, stamp)
        self.emit("clock", self.publish_clock, stamp)

    def emit(self, name: str, action: Any, *args: Any) -> None:
        """Run a publisher; log and continue on failure.

        Args:
            name: Stream label for warnings.
            action: Callable to invoke.
            *args: Forwarded to ``action``.
        """
        try:
            action(*args)
        except Exception as exc:
            warnings.warn(f"autosim {name} publish failed: {exc!r}", stacklevel=2)

    def poll_command(self) -> None:
        """Apply the latest ``cmd_vel`` if the reader has a message."""
        reader = self.bridge.command_reader
        if hasattr(reader, "has_msg") and reader.has_msg():
            linear, angular = Messages.parse_command_velocity(reader.get_msg())
            self.robot.set_twist(linear, angular, t=self.clock.now())

    def advance_timers(self, dt: float) -> None:
        """Accumulate per-sensor rate timers.

        Args:
            dt: Elapsed control period (s).
        """
        self.scan_elapsed += dt
        self.points_elapsed += dt
        self.camera_elapsed += dt
        self.inertial_elapsed += dt
        self.map_elapsed += dt

    def publish_odometry(
        self,
        x: float,
        y: float,
        yaw: float,
        linear: float,
        angular: float,
        stamp: Tuple[int, int],
    ) -> None:
        """Publish wheel odometry when ``odom.enabled``.

        Args:
            x, y, yaw: Ground-truth planar pose.
            linear, angular: Command velocities (m/s, rad/s).
            stamp: ``(sec, nanosec)`` header time.
        """
        if not self.odom.get("enabled", True):
            return
        ox, oy, oyaw = self.robot.update_odometry(x, y, yaw)
        variance = float(self.odom.get("noise", 0.0)) ** 2
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
                pose_variance=variance,
                twist_variance=variance,
            ),
        )

    def publish_scan(self, stamp: Tuple[int, int]) -> None:
        """Publish a 2D ``LaserScan`` when enabled and due.

        Args:
            stamp: Message timestamp.
        """
        if not self.lidar_2d["enabled"] or self.scan_elapsed < 1.0 / self.lidar_2d["rate_hz"]:
            return
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

    def publish_cloud(self, stamp: Tuple[int, int]) -> None:
        """Publish a 3D ``PointCloud2`` when enabled and due.

        Args:
            stamp: Message timestamp.
        """
        if not self.lidar_3d["enabled"] or self.points_elapsed < 1.0 / self.lidar_3d["rate_hz"]:
            return
        self.points_elapsed = 0.0
        points = self.sensors.sample_points(self.simulator)
        self.bridge.publish(
            "points",
            Messages.encode_point_cloud2(points, stamp, self.lidar_3d["frame"]),
        )

    def publish_camera(self, stamp: Tuple[int, int]) -> None:
        """Publish RGB, depth, and ``CameraInfo`` when enabled and due.

        Args:
            stamp: Message timestamp.
        """
        if not self.camera.get("enabled", False):
            return
        if self.camera_elapsed < 1.0 / self.camera["rate_hz"]:
            return
        self.camera_elapsed = 0.0
        color, depth = self.sensors.sample_camera(self.simulator)
        frame = self.camera["frame"]
        self.bridge.publish("rgb", Messages.encode_image(color, stamp, frame, "rgb8"))
        self.bridge.publish(
            "depth",
            Messages.encode_image(depth.astype(np.float32), stamp, frame, "32FC1"),
        )
        height, width = color.shape[:2]
        camera_matrix = Messages.camera_intrinsics(
            width, height, float(self.camera.get("hfov_deg", 90.0))
        )
        self.bridge.publish(
            "camera_info",
            Messages.encode_camera_info(stamp, frame, width, height, camera_matrix),
        )

    def publish_imu(self, yaw: float, linear: float, stamp: Tuple[int, int]) -> None:
        """Publish IMU when enabled and due.

        Args:
            yaw: Current heading (rad).
            linear: Current linear speed (m/s).
            stamp: Message timestamp.
        """
        if not self.imu.get("enabled", False):
            return
        if self.inertial_elapsed < 1.0 / self.imu["rate_hz"]:
            return
        self.inertial_elapsed = 0.0
        gyro_z, accel_x, accel_y = self.robot.update_inertial(yaw, linear, self.clock.now())
        half = 0.5 * yaw
        orientation = (0.0, 0.0, math.sin(half), math.cos(half))
        imu_noise = self.imu.get("noise") or {}
        gyro_var = float(imu_noise.get("gyro", 0.0)) ** 2
        accel_var = float(imu_noise.get("accel", 0.0)) ** 2
        self.bridge.publish(
            "imu",
            Messages.encode_inertial(
                stamp,
                self.imu["frame"],
                orientation,
                (0.0, 0.0, gyro_z),
                (accel_x, accel_y, 9.81),
                gyro_variance=gyro_var,
                accel_variance=accel_var,
            ),
        )

    def publish_map(self, stamp: Tuple[int, int]) -> None:
        """Publish panoramic ``/map/points`` and ``/map`` when due.

        Args:
            stamp: Message timestamp.
        """
        if self.map_builder is None or not self.map_cfg.get("enabled", False):
            return
        rate = float(self.map_cfg.get("rate_hz", 0.0))
        if rate <= 0.0:
            if self.map_published:
                return
        elif self.map_elapsed < 1.0 / rate and self.map_published:
            return
        self.map_elapsed = 0.0
        origin = (float(self.spawn[0]), float(self.spawn[1]))
        cloud, grid, resolution, ox, oy, _, _ = self.map_builder.sample(self.simulator, origin)
        frame = self.map_cfg["frame"]
        ply_channel = str((self.map_cfg.get("ply") or {}).get("channel") or "").strip()
        if ply_channel:
            self.bridge.publish("map_cloud", Messages.encode_point_cloud2(cloud, stamp, frame))
        self.bridge.publish(
            "map_grid",
            Messages.encode_occupancy_grid(grid, resolution, ox, oy, stamp, frame),
        )
        self.map_published = True

    def publish_tf(self, stamp: Tuple[int, int]) -> None:
        """Publish ``map→odom→base_link`` and static sensor mounts."""
        if not self.tf_cfg.get("enabled", False):
            return
        map_frame = self.robot_cfg.get("map_frame", "map")
        odom_frame = self.odom["frame"]
        base_frame = self.odom["child_frame"]
        ox, oy, oyaw = self.robot.odometry_pose()
        dynamic = [
            Messages.encode_transform(stamp, map_frame, odom_frame, (0.0, 0.0, 0.0), 0.0),
            Messages.encode_transform(stamp, odom_frame, base_frame, (ox, oy, 0.0), oyaw),
        ]
        self.bridge.publish("tf", Messages.encode_tf_message(dynamic))
        if self.tf_static_sent:
            return
        mounts = []
        urdf = getattr(self.simulator, "urdf", None)
        if urdf is not None:
            for link, xyz in (
                (self.lidar_2d.get("frame", "laser_link"), urdf.laser_xyz()),
                (self.imu.get("frame", "imu_link"), urdf.imu_xyz()),
                (self.camera.get("frame", "camera_link"), urdf.camera_xyz()),
            ):
                mounts.append(Messages.encode_transform(stamp, base_frame, link, xyz, 0.0))
        else:
            mounts.append(
                Messages.encode_transform(stamp, base_frame, self.lidar_2d.get("frame", "laser_link"), (0.0, 0.0, 0.2))
            )
        self.bridge.publish("tf_static", Messages.encode_tf_message(mounts))
        self.tf_static_sent = True

    def publish_clock(self, stamp: Tuple[int, int]) -> None:
        """Publish simulation ``/clock`` when enabled."""
        if not self.clock_cfg.get("enabled", False):
            return
        self.bridge.publish("clock", Messages.encode_clock(stamp))

    def publish_truth(self, stamp: Tuple[int, int]) -> None:
        """Publish ground-truth pose when ``robot.truth.enabled``.

        Args:
            stamp: Message timestamp.
        """
        if not self.robot_cfg["truth"]["enabled"]:
            return
        tx, ty, tyaw = self.robot.ground_truth()
        self.bridge.publish(
            "gt_pose",
            Messages.encode_pose_stamped(tx, ty, tyaw, stamp, self.robot_cfg["map_frame"]),
        )

    @staticmethod
    def main(argv: Optional[list[str]] = None) -> None:
        """CLI entry: load ``--config`` and start the runner.

        Args:
            argv: Argument vector; defaults to ``sys.argv``.
        """
        parser = argparse.ArgumentParser(prog="autosim")
        parser.add_argument(
            "--config",
            type=Path,
            default=Path(__file__).resolve().parents[1] / "config" / "default.yaml",
        )
        args = parser.parse_args(argv)
        Runner(Config.load(args.config)).run()
