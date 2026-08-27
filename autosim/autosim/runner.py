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

"""Fixed-rate control loop for the Habitat sensor–actuator bridge.

Wires :class:`~autosim.config.Config`, :class:`~autosim.robot.Robot`,
:class:`~autosim.sensors.Sensors`, :class:`~autosim.simulator.Simulator`,
and :class:`~autosim.bridge.Bridge` into one publish/subscribe cycle.
"""

from __future__ import annotations

import argparse
import math
import queue
import threading
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
from automsgs.msgs.geometry_msgs.polygon_stamped_pb2 import PolygonStamped
from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped
from automsgs.msgs.map_msgs.occupancy_grid_pb2 import OccupancyGrid
from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
from automsgs.msgs.builtin_interfaces.time_pb2 import Time as TimeMsg
from automsgs.msgs.sensor_msgs.camera_info_pb2 import CameraInfo
from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.sensor_msgs.imu_pb2 import Imu
from automsgs.msgs.sensor_msgs.laser_scan_pb2 import LaserScan
from automsgs.msgs.sensor_msgs.point_cloud2_pb2 import PointCloud2
from automsgs.msgs.tf2_msgs.tf_message_pb2 import TFMessage

TF_STATIC_REPUBLISH_SEC = 1.0


class SensorWorker:
    """Background thread for protobuf encoding and bridge publishing.

    Habitat render / ray-cast calls (which require the GL context) still run
    on the main thread.  Once raw numpy arrays are ready, the main thread
    submits ``(callable, args)`` to this worker for protobuf encoding and
    autolink publishing, so the main control loop is not stalled by I/O.

    A bounded queue (``maxsize=1``) drops stale frames when the worker falls
    behind, keeping end-to-end latency bounded.
    """

    def __init__(self) -> None:
        self._queue: queue.Queue = queue.Queue(maxsize=1)
        self._thread = threading.Thread(target=self._run, daemon=True, name="autosim-sensor")
        self._stop = threading.Event()

    def start(self) -> None:
        """Start the worker thread."""
        self._thread.start()

    def stop(self) -> None:
        """Signal the worker to stop and wait for it to exit."""
        self._stop.set()
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        self._thread.join(timeout=2.0)

    def submit(self, fn: Any, *args: Any) -> None:
        """Enqueue a sensor task, dropping the previous one if the queue is full."""
        try:
            self._queue.put_nowait((fn, args))
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait((fn, args))
            except queue.Full:
                pass

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                item = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                break
            fn, args = item
            try:
                fn(*args)
            except Exception as exc:
                warnings.warn(f"autosim sensor worker error: {exc!r}", stacklevel=2)


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
        self.clock = Clock(start_sec=time.time())
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
            # Let 2D lidar clip Habitat hits against the published occupancy grid.
            self.simulator.map_builder = self.map_builder
            try:
                origin = (float(self.spawn[0]), float(self.spawn[1]))
                self.map_builder.sample(self.simulator, origin)
            except Exception as exc:
                warnings.warn(
                    f"autosim occupancy warmup failed (laser wall-clip disabled): {exc!r}",
                    stacklevel=2,
                )
        # Worker for protobuf encoding + bridge publish only.
        # All Habitat GL/ray-cast calls remain on the main thread.
        self._sensor_worker = SensorWorker()

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
        self.tf_cfg = dict(self.robot_cfg.get("tf") or {"enabled": False})
        self.clock_cfg = self.robot_cfg.get("clock") or {"enabled": False}
        self.footprint = self.robot_cfg.get("footprint") or {"enabled": False}
        self.tf_static_elapsed = TF_STATIC_REPUBLISH_SEC
        # slam: Cartographer owns map→odom and /map; autosim publishes odom→base_link.
        # nav:  identity map→odom→base_link (+ URDF mounts); may publish GT /map.
        mode = str(settings["habitat"].get("mode", "nav")).strip().lower()
        self.mode = mode if mode in ("slam", "nav") else "nav"
        self.odom = dict(self.odom)
        self.odom["child_frame"] = "base_link"
        if isinstance(self.footprint, dict) and self.footprint.get("enabled", False):
            self.footprint = dict(self.footprint)
            self.footprint["frame"] = "base_link"
        if self.mode == "slam":
            self.tf_cfg["publish_map_odom"] = False
            self.map_publish = False
        else:
            self.tf_cfg["publish_map_odom"] = True
            self.map_publish = bool(self.map_cfg.get("publish", True))

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
        snap = getattr(self.simulator, "snap_to_navmesh", None)
        if callable(snap):
            snap()
            self.robot.teleport(*self.simulator.pose())
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
        if self.footprint.get("enabled", False):
            types["footprint"] = PolygonStamped
        if self.map_cfg.get("enabled", False) and self.map_publish:
            if str((self.map_cfg.get("ply") or {}).get("channel") or "").strip():
                types["map_cloud"] = PointCloud2
            types["map_grid"] = OccupancyGrid
        if self.tf_cfg.get("enabled", False):
            types["tf"] = TFMessage
            types["tf_static"] = TFMessage
        if self.clock_cfg.get("enabled", False):
            types["clock"] = TimeMsg
        return types

    def bind_bridge(self) -> None:
        """Initialize autolink and attach the channel bridge."""
        self.link.init("autosim")
        self.node = self.link.Node("autosim_bridge")
        self.bridge = Bridge(self.node, self.settings.channel_map(), self.message_types())

    def run(self) -> None:
        """Spin at ``control_hz`` until shutdown or ``max_steps``.

        Always closes the simulator and calls ``link.shutdown()`` on exit.
        Integration uses the real elapsed dt so TF/odom match ``cmd_vel`` even
        when a slow Habitat/camera tick overruns the nominal period.
        """
        period = 1.0 / float(self.robot_cfg["control_hz"])
        last = time.perf_counter() - period
        steps = 0
        self._sensor_worker.start()
        try:
            while not self.link.is_shutdown():
                now = time.perf_counter()
                elapsed = now - last
                last = now
                # On schedule (and in tests that stub sleep) keep the nominal
                # period so sensor timers and cmd_vel stay aligned. After a
                # Habitat/camera overrun, integrate the real elapsed time so
                # TF/odom still match the commanded twist.
                if elapsed > period:
                    dt = min(elapsed, 0.25)
                else:
                    dt = period
                self.cycle(dt)
                steps += 1
                if self.max_steps is not None and steps >= self.max_steps:
                    break
                remaining = period - (time.perf_counter() - now)
                if remaining > 0.0:
                    time.sleep(remaining)
        finally:
            self._sensor_worker.stop()
            self.simulator.close()
            self.link.shutdown()

    def cycle(self, dt: float) -> None:
        """Execute one control period.

        Physics, TF, and odometry always run on this thread so ``cmd_vel``
        stays smooth. Habitat GL/ray-cast cannot leave the session thread, so
        camera and lidar sampling still happen here — but at most one expensive
        sensor per period, and the static map is built only once.

        Args:
            dt: Period length in seconds (``1 / control_hz``).
        """
        self.poll_command()
        x, y, yaw = self.robot.step(dt=dt, t=self.clock.now() + dt)
        # SLAM: keep wheel odom glued to GT so Habitat laser rays (cast at GT)
        # match odom→base_link TF used by Cartographer / Autoviz. Noisy odom
        # with GT-cast scans is what makes /scan look rotated on the map.
        if self.mode == "slam":
            self.robot.odometry_x = float(x)
            self.robot.odometry_y = float(y)
            self.robot.odometry_yaw = float(yaw)
        self.simulator.set_pose(x, y, yaw)
        self.simulator.step()
        self.clock.tick(dt)
        stamp = self.clock.stamp()
        linear, angular = self.robot.velocity()
        # Do not fast-forward sensor due-ness after a hitch, or lidar+camera
        # all fire on the next tick and hitch again.
        self.advance_timers(min(dt, 0.05))
        self.emit("odom", self.publish_odometry, x, y, yaw, linear, angular, stamp)
        self.emit("imu", self.publish_imu, yaw, linear, stamp)
        self.emit("truth", self.publish_truth, stamp)
        self.emit("footprint", self.publish_footprint, stamp)
        self.emit("tf", self.publish_tf, stamp)
        self.emit("clock", self.publish_clock, stamp)
        # Occupancy / PLY map is static; never block teleop after first sample.
        self.emit("map", self.submit_map, stamp)
        # One Habitat-heavy sample per tick so TF/image don't hitch together.
        if self.scan_is_due() or self.cloud_is_due():
            self.emit("scan", self.submit_scan, stamp)
            self.emit("points", self.submit_cloud, stamp)
        else:
            self.emit("camera", self.submit_camera, stamp)

    def scan_is_due(self) -> bool:
        """True when 2D lidar should sample this period."""
        if not self.lidar_2d["enabled"]:
            return False
        return self.scan_elapsed >= 1.0 / self.lidar_2d["rate_hz"]

    def cloud_is_due(self) -> bool:
        """True when 3D lidar should sample this period."""
        if not self.lidar_3d["enabled"]:
            return False
        return self.points_elapsed >= 1.0 / self.lidar_3d["rate_hz"]

    def submit_scan(self, stamp: Tuple[int, int]) -> None:
        """Ray-cast and publish lidar on the control thread.

        Instantaneous Habitat casts must share the same stamp as ``publish_tf``
        in this cycle. Async encode used to let Autoviz see the scan after many
        newer TFs arrived; with a latest-TF fallback that looked like spin
        motion distortion.
        """
        if not self.scan_is_due():
            return
        self.scan_elapsed = 0.0
        ranges = self.sensors.sample_laser(self.simulator)
        self.encode_publish_scan(ranges, stamp)

    def encode_publish_scan(self, ranges: Any, stamp: Tuple[int, int]) -> None:
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
                # Instantaneous cast: do not advertise a spinning-lidar duration.
                scan_time=0.0,
            ),
        )

    def submit_cloud(self, stamp: Tuple[int, int]) -> None:
        """Ray-cast 3D lidar on the main thread, then encode/publish async."""
        if not self.cloud_is_due():
            return
        self.points_elapsed = 0.0
        points = self.sensors.sample_points(self.simulator)
        self._sensor_worker.submit(self.encode_publish_cloud, points, stamp)

    def encode_publish_cloud(self, points: Any, stamp: Tuple[int, int]) -> None:
        intensity = None
        if points.size > 0:
            intensity = np.linalg.norm(points, axis=1).astype(np.float32)
        self.bridge.publish(
            "points",
            Messages.encode_point_cloud2(points, stamp, self.lidar_3d["frame"], intensity=intensity),
        )

    def submit_camera(self, stamp: Tuple[int, int]) -> None:
        """Render RGB-D on the main (GL-context) thread, then encode/publish async."""
        if not self.camera.get("enabled", False):
            return
        if self.camera_elapsed < 1.0 / self.camera["rate_hz"]:
            return
        self.camera_elapsed = 0.0
        color, depth = self.sensors.sample_camera(self.simulator)
        self._sensor_worker.submit(self.encode_publish_camera, color, depth, stamp)

    def encode_publish_camera(self, color: Any, depth: Any, stamp: Tuple[int, int]) -> None:
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

    def submit_map(self, stamp: Tuple[int, int]) -> None:
        """Publish the static map without blocking the control loop after first sample.

        The semantic PLY is millions of points. Reloading and re-encoding it
        every ``rate_hz`` stalls Habitat (TF/cmd_vel hitch) and Autoviz (image
        decode waits behind a PointCloud2 rebuild). Sample once, then only
        republish the small OccupancyGrid for late subscribers.
        """
        if (
            self.map_builder is None
            or not self.map_cfg.get("enabled", False)
            or not self.map_publish
        ):
            return
        rate = float(self.map_cfg.get("rate_hz", 0.0))
        first = not self.map_published
        if not first:
            if rate <= 0.0:
                return
            if self.map_elapsed < 1.0 / rate:
                return
        self.map_elapsed = 0.0
        self.map_published = True
        origin = (float(self.spawn[0]), float(self.spawn[1]))
        cloud, grid, resolution, ox, oy, _, _ = self.map_builder.sample(self.simulator, origin)
        if not first:
            self._sensor_worker.submit(
                self.encode_publish_map_grid, grid, resolution, ox, oy, stamp
            )
            return
        cloud_rgb = getattr(self.map_builder, "cloud_rgb", None)
        ply_cfg = self.map_cfg.get("ply") or {}
        stride = int(ply_cfg.get("stride", 1))
        if stride > 1:
            cloud = Runner.subsample_points(cloud, stride)
            if cloud_rgb is not None:
                cloud_rgb = cloud_rgb[::stride]
        self._sensor_worker.submit(
            self.encode_publish_map, cloud, cloud_rgb, grid, resolution, ox, oy, stamp
        )

    def encode_publish_map(
        self,
        cloud: Any,
        cloud_rgb: Any,
        grid: Any,
        resolution: float,
        ox: float,
        oy: float,
        stamp: Tuple[int, int],
    ) -> None:
        ply_cfg = self.map_cfg.get("ply") or {}
        frame = self.map_cfg["frame"]
        ply_channel = str(ply_cfg.get("channel") or "").strip()
        if ply_channel:
            if cloud_rgb is not None and cloud_rgb.shape[0] == cloud.shape[0]:
                self.bridge.publish(
                    "map_cloud",
                    Messages.encode_point_cloud2(cloud, stamp, frame, rgb=cloud_rgb),
                )
            else:
                if cloud_rgb is not None and cloud_rgb.shape[0] != cloud.shape[0]:
                    import warnings
                    warnings.warn(
                        f"map cloud_rgb shape mismatch: xyz={cloud.shape[0]} "
                        f"rgb={cloud_rgb.shape[0]}; falling back to flat white",
                        stacklevel=2,
                    )
                self.bridge.publish(
                    "map_cloud",
                    Messages.encode_point_cloud2(cloud, stamp, frame),
                )
        self.bridge.publish(
            "map_grid",
            Messages.encode_occupancy_grid(grid, resolution, ox, oy, stamp, frame),
        )

    def encode_publish_map_grid(
        self,
        grid: Any,
        resolution: float,
        ox: float,
        oy: float,
        stamp: Tuple[int, int],
    ) -> None:
        """Republish OccupancyGrid only (late subscribers); skip the PLY cloud."""
        frame = self.map_cfg["frame"]
        self.bridge.publish(
            "map_grid",
            Messages.encode_occupancy_grid(grid, resolution, ox, oy, stamp, frame),
        )

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
        self.tf_static_elapsed += dt

    def odom_child_frame(self) -> str:
        """Dynamic child frame used by both ``/odom`` and ``/tf``.

        Both ``slam`` and ``nav`` use ``base_link`` so the tree is
        ``(map→)odom→base_link`` without inserting ``base_footprint``.
        """
        return self.body_frame()

    def body_frame(self) -> str:
        """Robot body frame that owns sensor mounts."""
        urdf = getattr(self.simulator, "urdf", None)
        if urdf is None:
            return str(self.odom.get("child_frame", "base_link"))
        return urdf.body_frame()

    def footprint_points(self) -> Tuple[Tuple[float, float, float], ...]:
        """Footprint vertices, preferring URDF-derived geometry over YAML points."""
        urdf = getattr(self.simulator, "urdf", None)
        if urdf is not None:
            polygon = tuple(urdf.footprint_polygon())
            if polygon:
                return polygon
        points = self.footprint.get("points") or []
        polygon = []
        for point in points:
            if len(point) == 2:
                polygon.append((float(point[0]), float(point[1]), 0.0))
            else:
                polygon.append((float(point[0]), float(point[1]), float(point[2])))
        return tuple(polygon)

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
                self.odom_child_frame(),
                pose_variance=variance,
                twist_variance=variance,
            ),
        )

    def publish_footprint(self, stamp: Tuple[int, int]) -> None:
        """Publish robot footprint as ``geometry_msgs/PolygonStamped``."""
        if not self.footprint.get("enabled", False):
            return
        polygon = self.footprint_points()
        if len(polygon) < 3:
            return
        msg = PolygonStamped()
        msg.header.frame_id = str(
            self.footprint.get("frame") or self.odom.get("child_frame", "base_footprint")
        )
        msg.header.stamp.sec = int(stamp[0])
        msg.header.stamp.nanosec = int(stamp[1])
        for x, y, z in polygon:
            point = msg.polygon.points.add()
            point.x = float(x)
            point.y = float(y)
            point.z = float(z)
        self.bridge.publish("footprint", msg)

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
                scan_time=0.0,
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
        intensity = None
        if points.size > 0:
            intensity = np.linalg.norm(points, axis=1).astype(np.float32)
        self.bridge.publish(
            "points",
            Messages.encode_point_cloud2(
                points,
                stamp,
                self.lidar_3d["frame"],
                intensity=intensity,
            ),
        )

    @staticmethod
    def subsample_points(
        cloud: np.ndarray,
        stride: int,
    ) -> np.ndarray:
        """Keep every ``stride``-th point for map / lidar downsampling."""
        step = max(1, int(stride))
        array = np.asarray(cloud, dtype=np.float32).reshape(-1, 3)
        if array.size == 0 or step <= 1:
            return array
        return array[::step]

    def publish_map(self, stamp: Tuple[int, int]) -> None:
        """Publish panoramic ``/map/points`` and ``/map`` when due.

        Args:
            stamp: Message timestamp.
        """
        if (
            self.map_builder is None
            or not self.map_cfg.get("enabled", False)
            or not self.map_publish
        ):
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
        ply_cfg = self.map_cfg.get("ply") or {}
        stride = int(ply_cfg.get("stride", 1))
        cloud = self.subsample_points(cloud, stride)
        frame = self.map_cfg["frame"]
        ply_channel = str(ply_cfg.get("channel") or "").strip()
        if ply_channel:
            intensity = None
            if cloud.size > 0:
                intensity = np.linalg.norm(cloud, axis=1).astype(np.float32)
            self.bridge.publish(
                "map_cloud",
                Messages.encode_point_cloud2(cloud, stamp, frame, intensity=intensity),
            )
        self.bridge.publish(
            "map_grid",
            Messages.encode_occupancy_grid(grid, resolution, ox, oy, stamp, frame),
        )
        self.map_published = True

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

    def publish_tf(self, stamp: Tuple[int, int]) -> None:
        """Publish robot TF for Autoviz / Cartographer / planning.

        - ``nav``: ``map→odom`` corrects wheel drift so ``map→base`` = GT (laser
          rays are cast from GT; identity ``map→odom`` misplaces ``/scan`` on the
          GT map). ``odom→base_link`` stays wheel odometry.
        - ``slam``: only ``odom→base_link`` (+ URDF); Cartographer owns ``map→odom``.
        """
        if not self.tf_cfg.get("enabled", False):
            return
        map_frame = self.robot_cfg.get("map_frame", "map")
        odom_frame = self.odom["frame"]
        odom_child = self.odom_child_frame()
        body_frame = self.body_frame()
        # Must match /odom (wheel pose), not ground truth — Cartographer looks up
        # odom→base from TF while integrating /odom; GT vs wheel mismatch makes
        # map→odom jump, especially during pure rotation.
        ox, oy, oyaw = self.robot.odometry_pose()
        base_z = float(self.robot_cfg.get("base_link_height", 0.0))
        publish_map_odom = bool(self.tf_cfg.get("publish_map_odom", True))
        # Drift-corrected map→odom so Autoviz LaserScan (GT rays) lands on GT map.
        mx, my, myaw = self.robot.map_to_odom(
            self.robot.pose(), self.robot.odometry_pose()
        )
        map_odom = Messages.encode_transform(
            stamp, map_frame, odom_frame, (mx, my, 0.0), myaw
        )
        dynamic = [
            Messages.encode_transform(stamp, odom_frame, odom_child, (ox, oy, base_z), oyaw),
        ]
        if publish_map_odom:
            dynamic.insert(0, map_odom)
        # map→odom is dynamic (tracks odom drift); never latch it on /tf_static.
        mounts = []
        urdf = getattr(self.simulator, "urdf", None)
        if urdf is not None:
            for child, (parent, xyz) in urdf.parents.items():
                if child == odom_child:
                    continue
                mounts.append(
                    Messages.encode_transform(
                        stamp,
                        parent,
                        child,
                        xyz,
                        0.0,
                    )
                )
            # odom→base_link skips URDF base_footprint→base_link; publish the
            # inverse so footprint / Autoviz still see base_footprint.
            if (
                odom_child == "base_link"
                and "base_link" in urdf.parents
                and urdf.parents["base_link"][0] == "base_footprint"
            ):
                fx, fy, fz = urdf.parents["base_link"][1]
                mounts.append(
                    Messages.encode_transform(
                        stamp,
                        "base_link",
                        "base_footprint",
                        (-float(fx), -float(fy), -float(fz)),
                        0.0,
                    )
                )
        else:
            mounts.append(
                Messages.encode_transform(
                    stamp,
                    body_frame,
                    self.lidar_2d.get("frame", "laser_link"),
                    (0.0, 0.0, 0.2),
                )
            )
        # One TFMessage, one stamp: every link is a single rigid snapshot.
        # Publishing URDF joints only at 1 Hz on /tf made child lookups
        # interpolate to an older common time, so the tree fanned apart
        # during yaw (ghost axes) and lagged cmd_vel.
        self.bridge.publish("tf", Messages.encode_tf_message(dynamic + mounts))
        if self.tf_static_elapsed >= TF_STATIC_REPUBLISH_SEC:
            self.bridge.publish(
                "tf_static", Messages.encode_tf_message(mounts)
            )
            self.tf_static_elapsed = 0.0

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
