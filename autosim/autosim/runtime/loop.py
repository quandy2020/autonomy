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

"""Fixed-rate control loop for the sensor–actuator bridge.

Wires :class:`~autosim.config.Config`, :class:`~autosim.robot.Robot`,
:class:`~autosim.sensing.Sensors`, a :class:`~autosim.backend.WorldBackend`,
and :class:`~autosim.io.bridge.Bridge` into one publish/subscribe cycle.

Backend differences use :func:`~autosim.backend.capability` flags — this module
must not branch on ``habitat.backend`` name strings.

Naming: attributes are nouns (``last_scan``, ``sensor_worker``); methods are
verb phrases (``poll_command``, ``submit_scan``, ``warmup_map``).
"""

from __future__ import annotations

import argparse
import logging
import math
import time
import warnings
from pathlib import Path as FilePath
from typing import Any, Dict, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

from autosim.backend import capability, create_backend
from autosim.config import Config
from autosim.dataset import DatasetCollector
from autosim.io.bridge import Bridge
from autosim.io.clock import Clock
from autosim.io.messages import Messages
from autosim.map import Map
from autosim.paths import project_root
from autosim.policy import PolicyDriver
from autosim.robot import Robot
from autosim.robot.profiles import apply_profile
from autosim.robot.urdf import UrdfModel
from autosim.runtime.worker import SensorWorker
from autosim.scenario.traffic import Traffic
from autosim.sensing.cameras import camera_rig, resolve_surround
from autosim.sensing.sensors import Sensors
from autosim.sensing.stitch import PANORAMA_UUID, panorama_spec

from automsgs.msgs.geometry_msgs.pose_stamped_pb2 import PoseStamped
from automsgs.msgs.geometry_msgs.polygon_stamped_pb2 import PolygonStamped
from automsgs.msgs.geometry_msgs.twist_stamped_pb2 import TwistStamped
from automsgs.msgs.map_msgs.occupancy_grid_pb2 import OccupancyGrid
from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
from automsgs.msgs.nav_msgs.path_pb2 import Path
from automsgs.msgs.builtin_interfaces.time_pb2 import Time as TimeMsg
from automsgs.msgs.sensor_msgs.camera_info_pb2 import CameraInfo
from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.sensor_msgs.imu_pb2 import Imu
from automsgs.msgs.sensor_msgs.laser_scan_pb2 import LaserScan
from automsgs.msgs.sensor_msgs.point_cloud2_pb2 import PointCloud2
from automsgs.msgs.tf2_msgs.tf_message_pb2 import TFMessage
from automsgs.msgs.visualization_msgs.marker_pb2 import Marker

TF_STATIC_REPUBLISH_SEC = 1.0


class Runner:
    """Sensor–actuator bridge driven at ``control_hz``."""

    def __init__(
        self,
        settings: Config,
        max_steps: Optional[int] = None,
        link: Any = None,
        simulator: Any = None,
    ) -> None:
        """Assemble plant, sensors, and autolink I/O.

        Args:
            settings: Validated :class:`~autosim.config.Config`.
            max_steps: Stop after N cycles; ``None`` runs until shutdown.
            link: Injected autolink module (tests); otherwise ``import autolink``.
            simulator: Injected WorldBackend; else built via backend registry.

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
        self.surround_elapsed = {camera.name: 0.0 for camera in self.surround}
        self.depth_points_elapsed = 0.0
        self.inertial_elapsed = 0.0
        self.map_elapsed = 0.0
        self.map_published = False
        self.map_cloud_msg: Optional[Any] = None
        self.map_builder: Optional[Any] = None
        # Accumulated /odom poses for optional Path (nav_msgs) trajectory.
        self.odom_path_poses: list[Tuple[float, float, float, Tuple[int, int]]] = []
        self.odom_path_last: Optional[Tuple[float, float]] = None
        self.last_scan: Optional[Any] = None
        self.last_rgb: Optional[Any] = None
        self.last_action: Tuple[float, float] = (0.0, 0.0)
        self.policy_driver: Optional[PolicyDriver] = PolicyDriver.create_when_enabled(
            self.settings["habitat"]
        )
        self.dataset: Optional[DatasetCollector] = DatasetCollector.create_when_enabled(
            self.settings["habitat"]
        )
        self.warmup_map()
        # Worker for protobuf encoding + bridge publish only.
        # All Habitat GL/ray-cast calls remain on the main thread.
        self.sensor_worker = SensorWorker()

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
        habitat = settings["habitat"]
        # Kept for diagnostics / logs only — behaviour uses backend capabilities.
        self.backend = str(habitat.get("backend", "habitat")).strip().lower()
        sensors = habitat["sensors"]
        self.robot_cfg = apply_profile(habitat["robot"])
        self.lidar_2d = sensors["lidar_2d"]
        self.lidar_3d = sensors["lidar_3d"]
        self.camera = sensors["camera"]
        self.sensors_block = sensors
        # Resolved in assemble_plant once WorldBackend capabilities are known.
        self.surround: tuple = ()
        self.panorama = None
        self.imu = sensors["imu"]
        self.odom = sensors["odom"]
        self.spawn = habitat["spawn"]
        self.map_cfg = habitat.get("map") or {"enabled": False}
        self.scenario_cfg = habitat.get("scenario") or {"enabled": False}
        self.tf_cfg = dict(self.robot_cfg.get("tf") or {"enabled": False})
        self.clock_cfg = self.robot_cfg.get("clock") or {"enabled": False}
        self.footprint = self.robot_cfg.get("footprint") or {"enabled": False}
        self.tf_static_elapsed = TF_STATIC_REPUBLISH_SEC
        # slam: Cartographer owns map→odom and /map; autosim publishes odom→base_link.
        # nav:  identity map→odom→base_link (+ URDF mounts); may publish GT /map.
        mode = str(habitat.get("mode", "nav")).strip().lower()
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

    def assemble_plant(self, simulator: Any = None) -> None:
        """Construct robot, WorldBackend, and sensor sampler.

        Args:
            simulator: Optional injected backend; otherwise ``create_backend``.
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
            gravity=float(self.imu.get("gravity", Robot.GRAVITY)),
            wheel_separation=float(self.robot_cfg.get("wheel_separation", 0.5)),
            max_linear_accel=float(self.robot_cfg.get("max_linear_accel", 0.8)),
            max_linear_decel=float(self.robot_cfg.get("max_linear_decel", 1.2)),
            max_angular_accel=float(self.robot_cfg.get("max_angular_accel", 2.0)),
            x=float(self.spawn[0]),
            y=float(self.spawn[1]),
            yaw=float(self.spawn[2]),
        )
        self.simulator = (
            simulator if simulator is not None else create_backend(self.settings.data)
        )
        self.resolve_camera_rig()
        self.simulator.reset(*self.robot.pose())
        snap = getattr(self.simulator, "snap_to_navmesh", None)
        if callable(snap):
            snap()
        ensure = getattr(self.simulator, "ensure_semantic_spawn", None)
        if callable(ensure):
            ensure()
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
        grid = getattr(self.simulator, "scenario_grid", None)
        if grid is None and getattr(self.simulator, "map_builder", None) is not None:
            grid = getattr(self.simulator.map_builder, "grid", None)
        self.traffic = Traffic.from_block(
            self.settings["habitat"].get("scenario"),
            grid,
        )
        self.simulator.traffic = self.traffic
        ped = self.traffic.pedestrian
        if ped is not None and int(getattr(ped, "position", np.zeros((0, 2))).shape[0]) > 0:
            field = self.traffic.field
            logger.info(
                "traffic pedestrians=%d field=%s arena=%s",
                int(ped.position.shape[0]),
                "yes" if field is not None else "no",
                "yes" if ped.arena is not None else "no",
            )
        self.urdf_for_footprint = self.footprint_urdf()

    def resolve_camera_rig(self) -> None:
        """Enable surround / panorama only when the backend supports them."""
        sensors = getattr(self, "sensors_block", None) or {}
        try:
            camera_urdf = UrdfModel.load(str(self.robot_cfg.get("urdf") or ""))
        except (FileNotFoundError, ValueError):
            camera_urdf = None
        if capability(self.simulator, "surround"):
            self.surround = resolve_surround(camera_rig(sensors), urdf=camera_urdf)
        else:
            self.surround = ()
        self.panorama = (
            panorama_spec(sensors) if capability(self.simulator, "panorama") else None
        )

    def warmup_map(self) -> None:
        """Warm occupancy / cloud map for nav publish and laser clipping."""
        if not self.map_cfg.get("enabled", False):
            return
        # Backends that own a procedural / floorplan map expose map_builder.
        built = getattr(self.simulator, "map_builder", None)
        if built is not None:
            self.map_builder = built
            if getattr(built, "grid", None) is not None:
                self.traffic = Traffic.from_block(self.scenario_cfg, built.grid)
                self.simulator.traffic = self.traffic
            return
        if not capability(self.simulator, "navmesh_map_sample"):
            return
        self.map_builder = Map(self.map_cfg)
        self.simulator.map_builder = self.map_builder
        try:
            self.map_builder.sample(self.simulator, self.map_sample_origin())
        except Exception as exc:
            warnings.warn(
                f"autosim occupancy warmup failed (laser wall-clip disabled): {exc!r}",
                stacklevel=2,
            )

    def message_types(self) -> Dict[str, Any]:
        """Build logical-key → protobuf type map for enabled streams.

        Returns:
            Writer/reader type dictionary for :class:`~autosim.bridge.Bridge`.
        """
        types: Dict[str, Any] = {"cmd_vel": TwistStamped}
        if self.odom.get("enabled", True):
            types["odom"] = Odometry
            if str(self.odom.get("path_channel") or "").strip():
                types["odom_path"] = Path
        if self.lidar_2d["enabled"]:
            types["scan"] = LaserScan
        if self.lidar_3d["enabled"]:
            types["points"] = PointCloud2
        depth_points_channel = str(
            self.camera.get("depth_points_channel") or ""
        ).strip()
        fov_marker_channel = str(self.camera.get("fov_marker_channel") or "").strip()
        if self.camera.get("enabled", False):
            types["rgb"] = Image
            types["depth"] = Image
            types["camera_info"] = CameraInfo
            if depth_points_channel:
                types["depth_points"] = PointCloud2
            if fov_marker_channel:
                types["fov_marker"] = Marker
            if self.camera.get("semantic_enabled", False) and str(
                self.camera.get("semantic_channel") or ""
            ).strip():
                types["semantic"] = Image
            if str(self.camera.get("semantic_ids_channel") or "").strip():
                types["semantic_ids"] = Image
        elif depth_points_channel:
            # Fake / Isaac: depth cloud without publishing RGB-D images.
            types["depth_points"] = PointCloud2
            if fov_marker_channel:
                types["fov_marker"] = Marker
        for camera in self.surround:
            types[camera.rgb_key] = Image
            types[camera.info_key] = CameraInfo
        if self.panorama is not None:
            types["surround_panorama"] = Image
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
        self.sensor_worker.start()
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
            self.sensor_worker.stop()
            if self.dataset is not None:
                self.dataset.close()
            if self.simulator is not None:
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
        self.poll_command(dt)
        x, y, yaw = self.robot.step(dt=dt, t=self.clock.now() + dt)
        # SLAM: keep wheel odom glued to GT so Habitat laser rays (cast at GT)
        # match odom→base_link TF used by Cartographer / Autoviz. Noisy odom
        # with GT-cast scans is what makes /scan look rotated on the map.
        if self.mode == "slam":
            self.robot.odometry_x = float(x)
            self.robot.odometry_y = float(y)
            self.robot.odometry_yaw = float(yaw)
        self.simulator.set_pose(x, y, yaw)
        # Step Traffic before USD sync so pedestrians/obstacles draw at the
        # current pose (IsaacSimulator.step → traffic_usd.sync).
        self.traffic.step(dt)
        self.simulator.step()
        self.clock.tick(dt)
        stamp = self.clock.stamp()
        linear, angular = self.robot.velocity()
        # Do not fast-forward sensor due-ness after a hitch, or lidar+camera
        # all fire on the next tick and hitch again.
        self.advance_timers(min(dt, 0.05))
        self.emit("odom", self.publish_odometry, x, y, yaw, linear, angular, stamp)
        # Pass full sim ``dt`` (not the capped timer step) so hitch intervals
        # still fill /imu at rate_hz — lightning rejects IMU Δt > 0.1 s.
        self.emit("imu", self.publish_imu, yaw, linear, stamp, dt)
        self.emit("truth", self.publish_truth, stamp)
        self.emit("footprint", self.publish_footprint, stamp)
        self.emit("tf", self.publish_tf, stamp)
        self.emit("clock", self.publish_clock, stamp)
        # Occupancy / PLY map is static; never block teleop after first sample.
        self.emit("map", self.submit_map, stamp)
        # Habitat: serialize heavy GL samples so TF and image do not hitch.
        # Isaac (serialize_sensors=False): lidar is cheap CPU cast — never starve
        # RGB when /scan is also due, or Autoviz looks frozen under teleop.
        serialize = capability(self.simulator, "serialize_sensors", True)
        lidar_due = self.scan_is_due() or self.cloud_is_due()
        if serialize and lidar_due:
            self.emit("scan", self.submit_scan, stamp)
            self.emit("points", self.submit_cloud, stamp)
        else:
            if lidar_due:
                self.emit("scan", self.submit_scan, stamp)
                self.emit("points", self.submit_cloud, stamp)
            self.emit("camera", self.submit_camera, stamp)
            # Depth cloud without RGB (fake) when camera.enabled is false.
            self.emit("depth_points", self.submit_depth_points_only, stamp)
        self.emit("surround", self.submit_surround, stamp)
        self.emit("dataset", self.record_dataset_frame, stamp, dt)

    def record_dataset_frame(self, stamp: Tuple[int, int], dt: float) -> None:
        """Append an imitation-learning frame when ``habitat.dataset`` is enabled."""
        if self.dataset is None:
            return
        x, y, yaw = self.robot.pose()
        vx, wz = self.robot.velocity()
        self.dataset.record_frame(
            float(dt),
            timestamp=float(self.clock.now()),
            pose=(float(x), float(y), float(yaw)),
            velocity=(float(vx), float(wz)),
            action=self.last_action,
            rgb=self.last_rgb,
            laser=self.last_scan,
        )

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
        if not self.scan_is_due() or self.sensors is None:
            return
        self.scan_elapsed = 0.0
        ranges = self.sensors.sample_laser(self.simulator)
        self.last_scan = np.asarray(ranges, dtype=np.float32)
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
        if not self.cloud_is_due() or self.sensors is None:
            return
        self.points_elapsed = 0.0
        linear, angular = self.robot.velocity()
        scan_period = 1.0 / max(float(self.lidar_3d.get("rate_hz", 10.0)), 1e-3)
        points = self.sensors.sample_points(
            self.simulator,
            linear=linear,
            angular=angular,
            scan_period=scan_period,
        )
        self.sensor_worker.submit(self.encode_publish_cloud, points, stamp)

    def encode_publish_cloud(self, points: Any, stamp: Tuple[int, int],
                             rings: Any = None) -> None:
        intensity = None
        ring = None
        if isinstance(points, tuple):
            points, rings = points
        pts = np.asarray(points)
        if pts.size > 0:
            intensity = np.linalg.norm(pts.reshape(-1, 3), axis=1).astype(np.float32)
            if rings is not None:
                ring = np.asarray(rings, dtype=np.float32).reshape(-1)
        self.bridge.publish(
            "points",
            Messages.encode_point_cloud2(
                pts, stamp, self.lidar_3d["frame"], intensity=intensity, ring=ring
            ),
        )

    def submit_camera(self, stamp: Tuple[int, int]) -> None:
        """Render and publish RGB-D on the control thread with one shared stamp.

        Habitat returns color+depth from one observation. Publishing them async
        via the sensor worker let Atlas see RGB_N with depth_N-1 (QoS/order),
        which looks like desynced timestamps. Keep encode+publish atomic.
        """
        if not self.camera.get("enabled", False) or self.sensors is None:
            return
        if self.camera_elapsed < 1.0 / self.camera["rate_hz"]:
            return
        self.camera_elapsed = 0.0
        with_semantic = bool(
            self.camera.get("semantic_enabled", False)
            and "semantic" in self.bridge.writers
        )
        sampled = self.sensors.sample_camera(
            self.simulator, with_semantic=with_semantic
        )
        if with_semantic:
            color, depth, semantic = sampled
            self.last_rgb = np.asarray(color)
            self.encode_publish_camera(color, depth, stamp, semantic=semantic)
        else:
            color, depth = sampled
            self.last_rgb = np.asarray(color)
            self.encode_publish_camera(color, depth, stamp)

    def submit_depth_points_only(self, stamp: Tuple[int, int]) -> None:
        """Publish depth cloud when RGB is disabled (e.g. fake FoV raycast)."""
        if self.camera.get("enabled", False):
            return
        channel = str(self.camera.get("depth_points_channel") or "").strip()
        if not channel or "depth_points" not in self.bridge.writers:
            return
        rate = float(self.camera.get("rate_hz", 10.0))
        if self.depth_points_elapsed < 1.0 / max(rate, 1e-3):
            return
        sample = getattr(self.simulator, "sample_depth_points", None)
        if not callable(sample):
            return
        self.depth_points_elapsed = 0.0
        frame = str(self.camera.get("frame") or "camera_link")

        def encode_and_publish() -> None:
            points = sample()
            self.bridge.publish(
                "depth_points",
                Messages.encode_point_cloud2(points, stamp, frame),
            )
            self.publish_camera_fov_marker(stamp, frame)

        self.sensor_worker.submit(encode_and_publish)

    def publish_camera_fov_marker(
        self, stamp: Tuple[int, int], frame: str | None = None
    ) -> None:
        """Publish the RGBD pinhole FoV wireframe on ``fov_marker``."""
        if "fov_marker" not in self.bridge.writers:
            return
        frame_id = str(frame or self.camera.get("frame") or "camera_link")
        vfov = self.camera.get("vfov_deg")
        if vfov is None:
            half_h = math.radians(float(self.camera.get("hfov_deg", 90.0))) * 0.5
            aspect = float(self.camera.get("height", 480)) / max(
                float(self.camera.get("width", 640)), 1.0
            )
            vfov = math.degrees(2.0 * math.atan(math.tan(half_h) * aspect))
        # Visualization length only — keep shorter than sensor range_max.
        marker_far = float(
            self.camera.get(
                "fov_marker_range",
                min(3.0, float(self.camera.get("range_max", 30.0))),
            )
        )
        marker_near = min(
            float(self.camera.get("range_min", 0.1)),
            0.5 * marker_far,
        )
        self.bridge.publish(
            "fov_marker",
            Messages.encode_camera_fov_marker(
                stamp,
                frame_id,
                float(self.camera.get("hfov_deg", 90.0)),
                float(vfov),
                marker_near,
                marker_far,
            ),
        )

    def submit_surround(self, stamp: Tuple[int, int]) -> None:
        """Publish each surround camera (RGB + CameraInfo) at its own ``rate_hz``."""
        due = [
            camera
            for camera in self.surround
            if self.surround_elapsed.get(camera.name, 0.0) >= 1.0 / max(camera.rate_hz, 1e-3)
        ]
        if not due:
            return
        render = getattr(self.simulator, "render_surround", None)
        if not callable(render):
            return
        images = render()
        for camera in due:
            self.surround_elapsed[camera.name] = 0.0
            color = images[camera.uuid]
            rgb_msg = Messages.encode_image(color, stamp, camera.frame, "rgb8")
            matrix = Messages.camera_intrinsics(
                camera.width, camera.height, camera.hfov_deg, camera.vfov_deg
            )
            info = Messages.encode_camera_info(
                stamp, camera.frame, camera.width, camera.height, matrix
            )
            info.header.stamp.sec = int(rgb_msg.header.stamp.sec)
            info.header.stamp.nanosec = int(rgb_msg.header.stamp.nanosec)
            self.bridge.publish(camera.rgb_key, rgb_msg)
            self.bridge.publish(camera.info_key, info)
        self.publish_panorama(images, stamp)

    def publish_panorama(self, images: Dict[str, Any], stamp: Tuple[int, int]) -> None:
        """Publish the equirectangular 360°×180° panorama."""
        if self.panorama is None:
            return
        color = images.get(PANORAMA_UUID)
        if color is None:
            return
        msg = Messages.encode_image(color, stamp, self.panorama["frame"], "rgb8")
        self.bridge.publish("surround_panorama", msg)

    def encode_publish_camera(
        self,
        color: Any,
        depth: Any,
        stamp: Tuple[int, int],
        *,
        semantic: Any | None = None,
    ) -> None:
        """Publish RGB, depth, semantic, and CameraInfo with identical ``header.stamp``."""
        frame = self.camera["frame"]
        rgb_msg = Messages.encode_image(color, stamp, frame, "rgb8")
        # Force HxW float32 metres. A stray HxWxC buffer published as 32FC1
        # makes autoviz stride/colorize jump frame-to-frame.
        height, width = color.shape[:2]
        depth_hw = np.asarray(depth, dtype=np.float32)
        if depth_hw.ndim == 3:
            depth_hw = depth_hw[..., 0]
        if depth_hw.shape == (width, height):
            depth_hw = depth_hw.T
        elif depth_hw.shape != (height, width):
            if depth_hw.size == height * width:
                depth_hw = depth_hw.reshape(height, width)
            else:
                depth_hw = np.full((height, width), np.nan, dtype=np.float32)
        depth_hw = np.ascontiguousarray(depth_hw, dtype=np.float32)
        depth_msg = Messages.encode_image(depth_hw, stamp, frame, "32FC1")
        # Hard-sync stamps so Atlas / Autoviz never see RGB≠depth time.
        depth_msg.header.stamp.sec = int(rgb_msg.header.stamp.sec)
        depth_msg.header.stamp.nanosec = int(rgb_msg.header.stamp.nanosec)
        camera_matrix = Messages.camera_intrinsics(
            width, height, float(self.camera.get("hfov_deg", 90.0))
        )
        info_msg = Messages.encode_camera_info(
            stamp, frame, width, height, camera_matrix
        )
        info_msg.header.stamp.sec = int(rgb_msg.header.stamp.sec)
        info_msg.header.stamp.nanosec = int(rgb_msg.header.stamp.nanosec)
        self.bridge.publish("rgb", rgb_msg)
        self.bridge.publish("depth", depth_msg)
        self.bridge.publish("camera_info", info_msg)
        depth_points_channel = str(
            self.camera.get("depth_points_channel") or ""
        ).strip()
        if depth_points_channel and "depth_points" in self.bridge.writers:
            stride = int(self.camera.get("depth_points_stride", 1))
            points = Messages.project_depth_to_points(
                depth_hw,
                camera_matrix,
                stride=stride,
                min_depth=0.15,
                max_depth=3.5,
            )
            if points.shape[0] > 0:
                self.bridge.publish(
                    "depth_points",
                    Messages.encode_point_cloud2(
                        Messages.optical_to_camera_link(points),
                        stamp,
                        frame,
                    ),
                )
            self.publish_camera_fov_marker(stamp, frame)
        if (
            semantic is not None
            and self.camera.get("semantic_enabled", False)
            and "semantic" in self.bridge.writers
        ):
            colored_fn = getattr(self.simulator, "semantic_ids_to_colored_rgb", None)
            if not callable(colored_fn):
                raise RuntimeError("simulator cannot colorize semantic ids")
            colored = colored_fn(semantic)
            semantic_msg = Messages.encode_image(colored, stamp, frame, "rgb8")
            semantic_msg.header.stamp.sec = int(rgb_msg.header.stamp.sec)
            semantic_msg.header.stamp.nanosec = int(rgb_msg.header.stamp.nanosec)
            self.bridge.publish("semantic", semantic_msg)
        ids_channel = str(self.camera.get("semantic_ids_channel") or "").strip()
        if semantic is not None and ids_channel and "semantic_ids" in self.bridge.writers:
            ids_msg = Messages.encode_image(
                np.asarray(semantic, dtype=np.uint16), stamp, frame, "mono16"
            )
            ids_msg.header.stamp.sec = int(rgb_msg.header.stamp.sec)
            ids_msg.header.stamp.nanosec = int(rgb_msg.header.stamp.nanosec)
            self.bridge.publish("semantic_ids", ids_msg)

    def map_sample_origin(self) -> Tuple[float, float]:
        """Planar hint for map sampling; use robot pose after spawn relocation."""
        if self.simulator is not None:
            x, y, _ = self.simulator.pose()
        else:
            x, y, _ = self.robot.pose()
        return (float(x), float(y))

    def submit_map(self, stamp: Tuple[int, int]) -> None:
        """Publish OccupancyGrid at ``rate_hz``; sample ``/overall/map`` once.

        PointCloud2 is encoded once on the **main thread** (not SensorWorker).
        The worker queue is size-1 and camera frames would drop a multi-second
        encode before Autoviz ever sees ``/overall/map``. Later ticks republish
        the cached protobuf (stamp only) so late subscribers still see it.
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
        cloud, grid, resolution, ox, oy, _, _ = self.map_builder.sample(
            self.simulator, self.map_sample_origin()
        )
        self.encode_publish_map_grid(grid, resolution, ox, oy, stamp)
        self.map_published = True
        ply_cfg = self.map_cfg.get("ply") or {}
        if not str(ply_cfg.get("channel") or "").strip():
            return
        if first:
            cloud_rgb = getattr(self.map_builder, "cloud_rgb", None)
            stride = int(ply_cfg.get("stride", 1))
            if stride > 1:
                cloud = Runner.subsample_points(cloud, stride)
                if cloud_rgb is not None:
                    cloud_rgb = cloud_rgb[::stride]
            self.map_cloud_static = cloud
            self.map_cloud_static_rgb = cloud_rgb
            cloud, cloud_rgb = self.merge_traffic_into_map_cloud(
                cloud, cloud_rgb, float(resolution)
            )
            self.encode_publish_map_cloud(cloud, cloud_rgb, stamp)
            return
        # Cached protobuf — never re-encode hundreds of thousands of points.
        self.republish_map_cloud(stamp)

    def merge_traffic_into_map_cloud(
        self,
        cloud: Any,
        cloud_rgb: Any,
        resolution: float,
    ) -> tuple[Any, Any]:
        """Append dynamic obstacle / pedestrian surface points to the map cloud."""
        traffic = getattr(self, "traffic", None)
        if traffic is None:
            return cloud, cloud_rgb
        spacing = max(0.05, float(resolution))
        dynamic = traffic.cloud(spacing)
        if dynamic.shape[0] == 0:
            return cloud, cloud_rgb
        static = np.asarray(cloud, dtype=np.float32).reshape(-1, 3)
        merged = np.concatenate([static, dynamic], axis=0)
        if cloud_rgb is None:
            return merged, None
        rgb = np.asarray(cloud_rgb)
        if rgb.shape[0] != static.shape[0]:
            return merged, None
        # Warm tint for movers so they stand out on the static map.
        tint = np.tile(np.array([[255, 96, 32]], dtype=np.uint8), (dynamic.shape[0], 1))
        return merged, np.concatenate([rgb, tint], axis=0)

    def encode_publish_map_cloud(
        self,
        cloud: Any,
        cloud_rgb: Any,
        stamp: Tuple[int, int],
    ) -> None:
        """Encode once and publish; cache the protobuf for cheap republish."""
        ply_cfg = self.map_cfg.get("ply") or {}
        frame = self.map_cfg["frame"]
        ply_channel = str(ply_cfg.get("channel") or "").strip()
        if not ply_channel:
            return
        if cloud_rgb is not None and cloud_rgb.shape[0] == cloud.shape[0]:
            msg = Messages.encode_point_cloud2(cloud, stamp, frame, rgb=cloud_rgb)
        else:
            if cloud_rgb is not None and cloud_rgb.shape[0] != cloud.shape[0]:
                warnings.warn(
                    f"map cloud_rgb shape mismatch: xyz={cloud.shape[0]} "
                    f"rgb={cloud_rgb.shape[0]}; falling back to flat white",
                    stacklevel=2,
                )
            msg = Messages.encode_point_cloud2(cloud, stamp, frame)
        self.map_cloud_msg = msg
        self.bridge.publish("map_cloud", msg)

    def republish_map_cloud(self, stamp: Tuple[int, int]) -> None:
        """Re-send the cached ``/overall/map`` with an updated stamp."""
        msg = getattr(self, "map_cloud_msg", None)
        if msg is None or "map_cloud" not in self.bridge.writers:
            return
        msg.header.stamp.sec = int(stamp[0])
        msg.header.stamp.nanosec = int(stamp[1])
        self.bridge.publish("map_cloud", msg)

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
        """Publish PLY cloud and OccupancyGrid (tests / legacy callers)."""
        self.encode_publish_map_cloud(cloud, cloud_rgb, stamp)
        self.encode_publish_map_grid(grid, resolution, ox, oy, stamp)

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

    def poll_command(self, dt: float = 0.02) -> None:
        """Apply teleop ``cmd_vel`` or an optional ONNX RL policy twist."""
        reader = self.bridge.command_reader
        has_external = hasattr(reader, "has_msg") and reader.has_msg()
        policy = self.policy_driver
        if has_external and not (policy is not None and policy.exclusive):
            linear, angular = Messages.parse_command_velocity(reader.get_msg())
            self.last_action = (float(linear), float(angular))
            self.robot.set_twist(linear, angular, t=self.clock.now())
            return
        if policy is None:
            return
        x, y, yaw = self.robot.pose()
        linear, angular = self.robot.velocity()
        cmd = policy.compute_command(
            float(dt),
            pose=(float(x), float(y), float(yaw)),
            velocity=(float(linear), float(angular)),
            scan=self.last_scan,
        )
        if cmd is not None:
            self.last_action = (float(cmd[0]), float(cmd[1]))
            self.robot.set_twist(cmd[0], cmd[1], t=self.clock.now())

    def advance_timers(self, dt: float) -> None:
        """Accumulate per-sensor rate timers.

        Args:
            dt: Elapsed control period (s).
        """
        self.scan_elapsed += dt
        self.points_elapsed += dt
        self.camera_elapsed += dt
        self.depth_points_elapsed += dt
        for name in self.surround_elapsed:
            self.surround_elapsed[name] += dt
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
        if self.simulator is None:
            return str(self.odom.get("child_frame", "base_link"))
        urdf = getattr(self.simulator, "urdf", None)
        if urdf is None:
            return str(self.odom.get("child_frame", "base_link"))
        return urdf.body_frame()

    def footprint_points(self) -> Tuple[Tuple[float, float, float], ...]:
        """Footprint vertices in the published footprint frame (``base_link``).

        URDF polygons are authored in the URDF root (usually ``base_footprint``);
        they are transformed into ``base_link`` when that link sits above the root.
        """
        urdf = getattr(self.simulator, "urdf", None) if self.simulator is not None else None
        if urdf is None:
            urdf = self.footprint_urdf()
        if urdf is not None:
            polygon = tuple(urdf.footprint_polygon())
            if polygon:
                return self.footprint_in_base_link(urdf, polygon)
        points = self.footprint.get("points") or []
        polygon = []
        for point in points:
            if len(point) == 2:
                polygon.append((float(point[0]), float(point[1]), 0.0))
            else:
                polygon.append((float(point[0]), float(point[1]), float(point[2])))
        return tuple(polygon)

    @staticmethod
    def footprint_in_base_link(
        urdf: UrdfModel,
        polygon: Tuple[Tuple[float, float, float], ...],
    ) -> Tuple[Tuple[float, float, float], ...]:
        """Map URDF-root polygon into ``base_link`` when the root is ``base_footprint``."""
        if urdf.root != "base_footprint" or "base_link" not in urdf.parents:
            return polygon
        parent, (px, py, pz) = urdf.parents["base_link"]
        if parent != "base_footprint":
            return polygon
        # parents stores translation of child in parent frame; invert for base_link.
        return tuple(
            (float(x) - float(px), float(y) - float(py), float(z) - float(pz))
            for x, y, z in polygon
        )

    def footprint_urdf(self) -> Optional[UrdfModel]:
        """Load / cache URDF for footprint when Habitat simulator is absent (fake)."""
        cached = getattr(self, "urdf_for_footprint", None)
        if cached is not None:
            return cached
        path = str(self.robot_cfg.get("urdf") or "").strip()
        if not path:
            return None
        try:
            model = UrdfModel.load(path)
        except (FileNotFoundError, ValueError):
            return None
        self.urdf_for_footprint = model
        return model

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
        self.publish_odom_path(ox, oy, oyaw, stamp)

    def publish_odom_path(
        self,
        x: float,
        y: float,
        yaw: float,
        stamp: Tuple[int, int],
    ) -> None:
        """Append wheel-odom pose (distance-gated) and always republish Path.

        ``path_min_distance`` only controls sampling density. The full path is
        re-published every odom tick so late Autoviz subscribers still see data.
        """
        if "odom_path" not in self.bridge.writers:
            return
        min_dist = float(self.odom.get("path_min_distance", 0.05))
        append = self.odom_path_last is None
        if not append:
            dx = float(x) - self.odom_path_last[0]
            dy = float(y) - self.odom_path_last[1]
            append = (dx * dx + dy * dy) >= (min_dist * min_dist)
        if append:
            self.odom_path_poses.append((float(x), float(y), float(yaw), stamp))
            self.odom_path_last = (float(x), float(y))
            max_poses = max(2, int(self.odom.get("path_max_poses", 10000)))
            if len(self.odom_path_poses) > max_poses:
                overflow = len(self.odom_path_poses) - max_poses
                del self.odom_path_poses[:overflow]
        if not self.odom_path_poses:
            return
        self.bridge.publish(
            "odom_path",
            Messages.encode_path(self.odom_path_poses, stamp, self.odom["frame"]),
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
        linear, angular = self.robot.velocity()
        scan_period = 1.0 / max(float(self.lidar_3d.get("rate_hz", 10.0)), 1e-3)
        points, rings = self.sensors.sample_points(
            self.simulator,
            linear=linear,
            angular=angular,
            scan_period=scan_period,
        )
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
                ring=rings if points.size > 0 else None,
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
        """Publish OccupancyGrid when due; sample PointCloud2 once, republish cache.

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
        first = not self.map_published
        if rate <= 0.0:
            if not first:
                return
        elif self.map_elapsed < 1.0 / rate and not first:
            return
        self.map_elapsed = 0.0
        cloud, grid, resolution, ox, oy, _, _ = self.map_builder.sample(
            self.simulator, self.map_sample_origin()
        )
        self.encode_publish_map_grid(grid, resolution, ox, oy, stamp)
        self.map_published = True
        ply_cfg = self.map_cfg.get("ply") or {}
        if not str(ply_cfg.get("channel") or "").strip():
            return
        if first:
            stride = int(ply_cfg.get("stride", 1))
            cloud = self.subsample_points(cloud, stride)
            cloud_rgb = getattr(self.map_builder, "cloud_rgb", None)
            if cloud_rgb is not None and stride > 1:
                cloud_rgb = cloud_rgb[::stride]
            self.map_cloud_static = cloud
            self.map_cloud_static_rgb = cloud_rgb
            cloud, cloud_rgb = self.merge_traffic_into_map_cloud(
                cloud, cloud_rgb, float(resolution)
            )
            self.encode_publish_map_cloud(cloud, cloud_rgb, stamp)
            return
        self.republish_map_cloud(stamp)

    def publish_camera(self, stamp: Tuple[int, int]) -> None:
        """Publish RGB, depth, and ``CameraInfo`` when enabled and due.

        Args:
            stamp: Message timestamp shared by RGB, depth, and CameraInfo.
        """
        if not self.camera.get("enabled", False):
            return
        if self.camera_elapsed < 1.0 / self.camera["rate_hz"]:
            return
        self.camera_elapsed = 0.0
        with_semantic = bool(
            self.camera.get("semantic_enabled", False)
            and "semantic" in self.bridge.writers
        )
        sampled = self.sensors.sample_camera(
            self.simulator, with_semantic=with_semantic
        )
        if with_semantic:
            color, depth, semantic = sampled
            self.last_rgb = np.asarray(color)
            self.encode_publish_camera(color, depth, stamp, semantic=semantic)
        else:
            color, depth = sampled
            self.last_rgb = np.asarray(color)
            self.encode_publish_camera(color, depth, stamp)

    def publish_imu(
        self,
        yaw: float,
        linear: float,
        stamp: Tuple[int, int],
        dt: float = 0.0,
    ) -> None:
        """Publish IMU when enabled.

        Habitat has no physics IMU. Specific force comes from
        :meth:`Robot.update_inertial`. One control tick may span a large sim
        ``dt`` when lidar hitches (~0.1 s); lightning aborts IMU predict when
        successive sample Δt > 0.1 s. Subdivide the interval at ``rate_hz`` so
        stamped IMU stays dense even if the control loop is slow.

        Args:
            yaw: Current heading (rad).
            linear: Current linear speed (m/s).
            stamp: End-of-interval message timestamp.
            dt: Sim time covered by this control cycle (s).
        """
        if not self.imu.get("enabled", False):
            return
        rate = max(1.0, float(self.imu.get("rate_hz", 50.0)))
        period = 1.0 / rate
        span = max(float(dt), 0.0)
        if span <= 0.0:
            span = period
        # Samples across [t_end - span, t_end]; cap to avoid flood on huge hitch.
        n = max(1, int(math.floor(span / period + 1e-9)))
        n = min(n, 50)
        sub_dt = span / float(n)
        t_end = float(stamp[0]) + float(stamp[1]) * 1e-9
        t_start = t_end - span
        yaw0 = float(getattr(self, "imu_yaw", yaw))
        dyaw = math.atan2(math.sin(float(yaw) - yaw0), math.cos(float(yaw) - yaw0))
        imu_noise = self.imu.get("noise") or {}
        gyro_var = float(imu_noise.get("gyro", 0.0)) ** 2
        accel_var = float(imu_noise.get("accel", 0.0)) ** 2
        frame = self.imu["frame"]
        for i in range(1, n + 1):
            alpha = float(i) / float(n)
            yaw_i = yaw0 + alpha * dyaw
            t_i = t_start + float(i) * sub_dt
            gyro_z, accel_x, accel_y, accel_z = self.robot.update_inertial(
                yaw_i, linear, t_i
            )
            half = 0.5 * yaw_i
            orientation = (0.0, 0.0, math.sin(half), math.cos(half))
            stamp_i = Clock(start_sec=t_i).stamp()
            self.bridge.publish(
                "imu",
                Messages.encode_inertial(
                    stamp_i,
                    frame,
                    orientation,
                    (0.0, 0.0, gyro_z),
                    (accel_x, accel_y, accel_z),
                    gyro_variance=gyro_var,
                    accel_variance=accel_var,
                ),
            )
        self.imu_yaw = float(yaw)

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
        urdf = getattr(self.simulator, "urdf", None) if self.simulator is not None else None
        if urdf is None:
            urdf = self.footprint_urdf()
        if capability(self.simulator, "fallback_camera_tf") and urdf is None:
            eye = float(self.camera.get("sensor_height", 0.6))
            frame = str(self.camera.get("frame") or "camera_link")
            mounts.append(
                Messages.encode_transform(
                    stamp, body_frame, frame, (0.0, 0.0, eye), 0.0
                )
            )
        elif urdf is not None:
            for child, (parent, xyz) in urdf.parents.items():
                if child == odom_child:
                    continue
                mounts.append(
                    Messages.encode_transform(
                        stamp,
                        parent,
                        child,
                        xyz,
                        float(urdf.joint_yaw.get(child, 0.0)),
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
            if capability(self.simulator, "fallback_camera_tf"):
                # Prefer URDF camera_link; else place optics at sensor_height.
                cam_frame = str(self.camera.get("frame") or "camera_link")
                if cam_frame not in urdf.parents and cam_frame != urdf.root:
                    eye = float(self.camera.get("sensor_height", 0.6))
                    mounts.append(
                        Messages.encode_transform(
                            stamp, body_frame, cam_frame, (0.0, 0.0, eye), 0.0
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
        urdf_links = set(urdf.parents) if urdf is not None else set()
        for camera in self.surround:
            if camera.frame in urdf_links:
                continue
            mounts.append(
                Messages.encode_transform(
                    stamp,
                    body_frame,
                    camera.frame,
                    (camera.x, camera.y, camera.z),
                    camera.yaw,
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
            type=FilePath,
            default=project_root() / "config" / "default.yaml",
        )
        args = parser.parse_args(argv)
        Runner(Config.load(args.config)).run()
