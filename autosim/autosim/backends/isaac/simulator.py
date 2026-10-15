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

"""Isaac Sim WorldBackend: USD stage + sensors aligned with Habitat contracts."""

from __future__ import annotations

import logging
import math
from typing import Any, Mapping, Tuple

import numpy as np

import atexit

from autosim.backend.capabilities import ISAAC_CAPABILITIES
from autosim.backends.isaac.app import close_app, ensure_simulation_app, update_app
from autosim.backends.isaac.physics import IsaacPhysics, physics_config_from_habitat
from autosim.backends.isaac.sensors import IsaacSensors
from autosim.backends.isaac.traffic_usd import (
    maybe_create_traffic_usd,
    traffic_config_from_isaac,
)
from autosim.backends.isaac.usd_stage import UsdStage, require_isaac
from autosim.map import Map as OccupancyMap
from autosim.robot.profiles import apply_profile, resolve_usd_paths
from autosim.robot.urdf import UrdfModel

atexit_registered = False
logger = logging.getLogger(__name__)


class IsaacSimulator:
    """WorldBackend for Isaac Sim + USD (z-up map frame)."""

    capabilities = ISAAC_CAPABILITIES

    def __init__(
        self,
        *,
        settings: Mapping[str, Any],
        stage: UsdStage,
        sensors: IsaacSensors,
        urdf: UrdfModel | None,
        width: int,
        height: int,
        eye_height: float,
    ) -> None:
        self.settings = settings
        self.stage = stage
        self.engine_sensors = sensors
        self.urdf = urdf
        self.width = int(width)
        self.height = int(height)
        self.sensor_height = float(eye_height)
        self.traffic: Any = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.map_builder: Any = None
        self.physics: IsaacPhysics | None = None
        self.traffic_usd: Any = None
        self.scenario_grid: Any = None
        self.scenario_points: Any = None

    @classmethod
    def create(cls, settings: Mapping[str, Any]) -> "IsaacSimulator":
        """Factory from validated config; requires Isaac Sim Python APIs."""
        habitat = settings["habitat"]
        camera = habitat["sensors"]["camera"]
        # Only load RTX rendering when RGB-D images are actually enabled.
        # A lone depth_points_channel does not require the camera experience.
        want_camera = bool(camera.get("enabled", False))
        isaac_block = habitat.get("isaac") if isinstance(habitat.get("isaac"), Mapping) else {}
        headless_cfg = None
        if isinstance(isaac_block, Mapping) and "headless" in isaac_block:
            headless_cfg = bool(isaac_block["headless"])
        ensure_simulation_app(
            headless=headless_cfg,
            enable_cameras=want_camera,
        )
        require_isaac()
        global atexit_registered
        if not atexit_registered:
            atexit.register(close_app)
            atexit_registered = True
        robot_cfg = dict(habitat.get("robot") or {})
        robot_cfg = apply_profile(robot_cfg)
        width = int(camera["width"])
        height = int(camera["height"])
        eye = float(camera.get("sensor_height", robot_cfg.get("base_link_height", 0.5)))
        world_usd, robot_usd = resolve_usd_paths(habitat, robot_cfg)
        base_z = float(robot_cfg.get("base_link_height", 0.0))
        prim = str(
            (habitat.get("usd") or {}).get("robot_prim")
            or robot_cfg.get("usd_prim")
            or "/World/Robot"
        )
        stage = UsdStage(
            world_usd,
            robot_usd=robot_usd,
            robot_prim_path=prim,
            base_z=base_z,
        )
        sensors = IsaacSensors(width=width, height=height, eye_height=eye)
        try:
            urdf = UrdfModel.load(str(robot_cfg.get("urdf") or ""))
        except (FileNotFoundError, ValueError):
            urdf = None
        sim = cls(
            settings=settings,
            stage=stage,
            sensors=sensors,
            urdf=urdf,
            width=width,
            height=height,
            eye_height=eye,
        )
        sim.attach_floorplan_map(world_usd, habitat.get("map"))
        sim.seed_scenario_geometry(habitat)
        # PhysX first (RB on collision capsule only), then camera under robot
        # so Replicator tracks parent xform like Habitat agent sensors.
        phys_cfg = physics_config_from_habitat(habitat)
        sim.physics = IsaacPhysics.attach(
            stage,
            robot_prim_path=prim,
            config=phys_cfg,
            render=want_camera,
        )
        if want_camera:
            sensors.mount_robot_camera(
                prim,
                hfov_deg=float(camera.get("hfov_deg", 90.0)),
                rate_hz=float(camera.get("rate_hz", 30.0)),
            )
        traffic_cfg = traffic_config_from_isaac(
            isaac_block if isinstance(isaac_block, Mapping) else None
        )
        parent = prim.rsplit("/", 1)[0] if "/" in prim else "/Root"
        if not traffic_cfg.get("usd_root"):
            traffic_cfg["usd_root"] = f"{parent}/AutosimTraffic"
        sim.traffic_usd = maybe_create_traffic_usd(stage, config=traffic_cfg)
        return sim

    def attach_floorplan_map(
        self, world_usd: str, map_cfg: Mapping[str, Any] | None
    ) -> None:
        """Prefer InteriorAgent ``rooms.json`` so nav can publish GT ``/map``.

        Occupancy stays floorplan-based; ``/overall/map`` prefers USD mesh
        vertices so furniture and full walls are not missing.
        """
        if not isinstance(map_cfg, Mapping) or not map_cfg.get("enabled", False):
            return
        from autosim.backends.isaac.floorplan_map import FloorplanMap

        rooms_path = FloorplanMap.find_rooms_json(world_usd)
        if rooms_path is None:
            return
        rooms = FloorplanMap.load_rooms(rooms_path)
        if not rooms:
            return
        built = FloorplanMap(map_cfg).build_from_rooms(rooms)
        grid_cfg = map_cfg.get("grid") or {}
        ply_cfg = map_cfg.get("ply") or {}
        ply_channel = str(ply_cfg.get("channel") or "").strip()
        # Habitat publishes scene geometry on /overall/map; do the same from USD
        # mesh vertices so Autoviz shape matches the Isaac viewport.
        mesh_cloud = np.zeros((0, 3), dtype=np.float32)
        # Occupancy carve: USD vertices only (+ inflate). Viz cloud:
        # - surface_spacing_m > 0 → densified USD (slow; optional)
        # - else keep rooms.json dense_floorplan_cloud (solid walls, fast)
        surface_spacing = float(ply_cfg.get("surface_spacing_m", 0.0))
        mesh_stride = max(1, int(ply_cfg.get("mesh_stride", 1)))
        isaac = (self.settings.get("habitat") or {}).get("isaac") or {}
        phys = isaac.get("physics") if isinstance(isaac, Mapping) else {}
        if not isinstance(phys, Mapping):
            phys = {}
        z_lo, z_hi, inflate = FloorplanMap.robot_body_band(
            map_cfg,
            robot_height_m=float(phys.get("robot_height", 0.28)),
            robot_radius_m=float(phys.get("robot_radius", 0.12)),
        )
        furniture = np.zeros((0, 3), dtype=np.float32)
        try:
            furniture = self.stage.sample_mesh_cloud(
                z_min=z_lo,
                z_max=z_hi,
                stride=mesh_stride,
                max_points=int(ply_cfg.get("max_points", 400_000)),
                surface_spacing_m=0.0,
                skip_path_substrings=("door",),
            )
        except Exception as exc:
            logger.warning("USD furniture sample failed: %s", exc)
        if furniture.shape[0] > 0:
            changed = built.mark_elevated_obstacles(
                furniture,
                z_min=z_lo,
                z_max=z_hi,
                inflate_m=inflate,
            )
        else:
            changed = 0
        opened = 0
        if bool(map_cfg.get("free_doors", True)):
            try:
                door_stride = max(
                    1, min(4, int(ply_cfg.get("mesh_stride", 8)) // 2)
                )
                door_cloud = self.stage.sample_mesh_cloud(
                    z_min=0.0,
                    z_max=max(z_hi, 2.2),
                    stride=door_stride,
                    max_points=200_000,
                    surface_spacing_m=0.0,
                    only_path_substrings=("door",),
                )
                door_inflate = float(
                    map_cfg.get("door_opening_inflate_m", 0.15)
                )
                opened = built.free_openings(
                    door_cloud,
                    z_min=0.0,
                    z_max=max(z_hi, 2.2),
                    inflate_m=door_inflate,
                )
            except Exception as exc:
                logger.warning("door opening free failed: %s", exc)
        logger.info(
            "Isaac /map body-band z=[%.2f, %.2f] inflate=%.2f "
            "(%d furniture → %d occ, %d door cells freed)",
            z_lo,
            z_hi,
            inflate,
            int(furniture.shape[0]),
            int(changed),
            int(opened),
        )
        if ply_channel and surface_spacing > 0.0:
            try:
                mesh_cloud = self.stage.sample_mesh_cloud(
                    z_min=float(grid_cfg.get("z_min", 0.0)),
                    z_max=float(grid_cfg.get("z_max", 2.5)),
                    stride=mesh_stride,
                    max_points=int(ply_cfg.get("max_points", 400_000)),
                    surface_spacing_m=surface_spacing,
                )
                if mesh_cloud.shape[0] > 0:
                    built.set_cloud(mesh_cloud)
            except Exception as exc:
                logger.warning("USD mesh cloud densify failed: %s", exc)
        elif ply_channel and furniture.shape[0] > 0:
            # Append furniture verts onto solid floorplan walls.
            base = np.asarray(built.cached_cloud, dtype=np.float32).reshape(-1, 3)
            built.set_cloud(np.vstack([base, furniture]))
        self.map_builder = built
        self.scenario_grid = built.grid
        self.scenario_points = getattr(built, "points", None)

    def seed_scenario_geometry(self, habitat: Mapping[str, Any]) -> None:
        """Fill ``scenario.x_length`` / center / spawn from the floorplan grid."""
        packed = self.scenario_grid
        if packed is None:
            return
        cells, resolution, ox, oy, width, height = packed
        min_x = float(ox)
        min_y = float(oy)
        max_x = min_x + float(width) * float(resolution)
        max_y = min_y + float(height) * float(resolution)
        cx = 0.5 * (min_x + max_x)
        cy = 0.5 * (min_y + max_y)
        spawn = habitat.get("spawn") or [cx, cy, 0.0]
        scenario = habitat.setdefault("scenario", {})
        if not isinstance(scenario, dict):
            return
        scenario.setdefault("x_length", max(1.0, max_x - min_x))
        scenario.setdefault("y_length", max(1.0, max_y - min_y))
        scenario.setdefault("center_x", cx)
        scenario.setdefault("center_y", cy)
        scenario.setdefault("spawn_x", float(spawn[0]))
        scenario.setdefault("spawn_y", float(spawn[1]))
        scenario.setdefault("spawn_radius", 0.8)

    def reset(self, x: float, y: float, yaw: float) -> None:
        # Spawn must teleport even if the capsule overlaps furniture initially.
        self.set_pose(x, y, yaw, force=True)

    def set_pose(
        self, x: float, y: float, yaw: float, *, force: bool = False
    ) -> None:
        """Write planar pose to the robot Xform.

        When PhysX ``block_motion`` is on, overlapping queries keep the previous
        pose (kinematic capsule vs scene colliders).
        """
        nx, ny, nyaw = float(x), float(y), float(yaw)
        physics = self.physics
        if (
            not force
            and physics is not None
            and physics.block_motion
            and physics.overlaps(nx, ny)
        ):
            return
        self.x = nx
        self.y = ny
        self.yaw = nyaw
        self.stage.set_pose(self.x, self.y, self.yaw)
        # Replicator Camera does not reliably inherit parent Xform under PhysX;
        # push eye pose explicitly so RGB/depth track teleop.
        sync = getattr(self.engine_sensors, "sync_world_pose", None)
        if callable(sync):
            base_z = float(getattr(self.stage, "base_z", 0.0) or 0.0)
            sync(self.x, self.y, self.yaw, base_z=base_z)

    def pose(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.yaw

    def step(self) -> None:
        """Advance PhysX without RTX (Habitat: render only on camera sample)."""
        if self.traffic_usd is not None and self.traffic is not None:
            try:
                self.traffic_usd.sync(self.traffic)
            except Exception as exc:
                if not getattr(self, "_traffic_usd_sync_warned", False):
                    self._traffic_usd_sync_warned = True
                    logger.warning("traffic_usd.sync failed (further errors muted): %s", exc)
        physics = self.physics
        if physics is not None:
            # Never render here — color_depth() owns the single Kit/RTX frame.
            physics.step(render=False)
            return
        update_app()

    def close(self) -> None:
        # Close the USD stage only; Kit app teardown is deferred to atexit so
        # SimulationApp.close() cannot abort the process mid-finally.
        try:
            if self.physics is not None:
                self.physics.close()
        except Exception:
            pass
        self.physics = None
        self.traffic_usd = None
        try:
            self.engine_sensors.close()
        except Exception:
            pass
        self.stage.close()

    def eye_height(self) -> float:
        return self.sensor_height

    def linspace_angles(
        self, angle_min: float, angle_max: float, count: int
    ) -> np.ndarray:
        if count <= 1:
            return np.array([float(angle_min)], dtype=np.float64)
        return np.linspace(
            float(angle_min), float(angle_max), num=int(count), dtype=np.float64
        )

    def laser_ranges(
        self,
        angle_min: float,
        angle_max: float,
        num_beams: int,
        range_max: float,
    ) -> np.ndarray:
        """Floorplan occupancy rays + Traffic cylinders (Fake parity)."""
        angles = self.linspace_angles(angle_min, angle_max, num_beams)
        depths = self.static_laser_ranges(angles, float(range_max))
        # Prefer Isaac lidar annotator when present; take nearer of the two.
        annotated = self.engine_sensors.laser_ranges(
            angle_min,
            angle_max,
            num_beams,
            range_max,
            pose=(self.x, self.y, self.yaw),
        )
        if annotated is not None and np.isfinite(annotated).any():
            depths = self.merge_nearer_ranges(depths, annotated)
        depths = self.merge_traffic_ranges(depths, angles, float(range_max))
        return depths.astype(np.float32)

    def laser_origin_xy(self) -> Tuple[float, float]:
        """Map-frame XY of ``laser_link`` / ``base_scan`` (matches Autoviz TF)."""
        lx, ly = 0.0, 0.0
        urdf = self.urdf
        if urdf is not None:
            mounts = getattr(urdf, "mounts", {}) or {}
            for name in ("laser_link", "base_scan"):
                if name in mounts:
                    lx, ly = float(mounts[name][0]), float(mounts[name][1])
                    break
                if name in urdf.parents:
                    try:
                        mx, my, _ = UrdfModel.link_xyz(
                            name, urdf.parents, urdf.root
                        )
                        lx, ly = float(mx), float(my)
                        break
                    except Exception:
                        continue
        cosine = math.cos(float(self.yaw))
        sine = math.sin(float(self.yaw))
        return (
            float(self.x) + cosine * lx - sine * ly,
            float(self.y) + sine * lx + cosine * ly,
        )

    def static_laser_ranges(
        self, angles: np.ndarray, range_max: float
    ) -> np.ndarray:
        packed = self.scenario_grid
        if packed is None and self.map_builder is not None:
            packed = getattr(self.map_builder, "grid", None)
        out = np.full(angles.shape, np.nan, dtype=np.float64)
        if packed is None:
            return out
        grid, resolution, ox, oy, _, _ = packed
        ox_l, oy_l = self.laser_origin_xy()
        for index, angle in enumerate(np.asarray(angles, dtype=np.float64).reshape(-1)):
            hit = OccupancyMap.occupancy_range(
                ox_l,
                oy_l,
                float(self.yaw) + float(angle),
                grid,
                float(resolution),
                float(ox),
                float(oy),
                float(range_max),
            )
            if hit is not None:
                out[index] = float(hit)
        return out

    def merge_traffic_ranges(
        self, depths: np.ndarray, angles: np.ndarray, range_max: float
    ) -> np.ndarray:
        traffic = self.traffic
        if traffic is None or not hasattr(traffic, "nearest_batch"):
            return depths
        dirs = np.stack(
            [
                np.cos(self.yaw + angles),
                np.sin(self.yaw + angles),
                np.zeros_like(angles),
            ],
            axis=1,
        )
        origin = np.array(
            [self.x, self.y, float(self.sensor_height)], dtype=np.float64
        )
        dynamic = traffic.nearest_batch(origin, dirs, float(range_max))
        return self.merge_nearer_ranges(depths, dynamic)

    @staticmethod
    def merge_nearer_ranges(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        left = np.asarray(a, dtype=np.float64).reshape(-1).copy()
        right = np.asarray(b, dtype=np.float64).reshape(-1)
        if right.shape != left.shape:
            return left
        better = np.isfinite(right) & (np.isnan(left) | (right < left))
        left[better] = right[better]
        return left

    def clip_laser_to_occupancy(
        self, ranges: np.ndarray, angles: np.ndarray
    ) -> np.ndarray:
        """Sensors helper: clip beams to floorplan walls when map exists."""
        mapper = self.map_builder
        clip = getattr(mapper, "clip_laser_ranges", None)
        if not callable(clip):
            return ranges
        lidar = (self.settings.get("habitat") or {}).get("sensors", {}).get(
            "lidar_2d", {}
        )
        range_max = float(lidar.get("range_max", 30.0))
        return clip(
            ranges,
            angles,
            self.x,
            self.y,
            self.yaw,
            range_max,
        )

    def lidar_points(
        self,
        h_min: float,
        h_max: float,
        h_beams: int,
        v_min: float,
        v_max: float,
        v_rings: int,
        range_max: float,
        **kwargs: Any,
    ) -> Tuple[np.ndarray, np.ndarray]:
        del kwargs
        cloud, rings = self.engine_sensors.lidar_points(
            h_min, h_max, h_beams, v_min, v_max, v_rings, range_max
        )
        # When annotator empty, synthesize a planar ring from laser_ranges.
        if cloud.shape[0] == 0:
            ranges = self.laser_ranges(h_min, h_max, h_beams, range_max)
            angles = self.linspace_angles(h_min, h_max, h_beams)
            pts: list[list[float]] = []
            ring_ids: list[float] = []
            for ang, dist in zip(angles, ranges):
                if not np.isfinite(dist):
                    continue
                pts.append(
                    [
                        float(dist * math.cos(ang)),
                        float(dist * math.sin(ang)),
                        0.0,
                    ]
                )
                ring_ids.append(0.0)
            if pts:
                return (
                    np.asarray(pts, dtype=np.float32),
                    np.asarray(ring_ids, dtype=np.float32),
                )
        return cloud, rings

    def color_depth(self) -> Tuple[np.ndarray, np.ndarray]:
        """One RTX frame after eye pose, then sample RGB-D (Habitat timing).

        Control ticks use PhysX-only ``step(render=False)``; this is the only
        place that pays for rendering so publish rate can approach ``rate_hz``.
        """
        # Re-assert camera world pose in case PhysX overwrote the Xform since
        # the last set_pose (kinematic teleport vs Fabric cache).
        sync = getattr(self.engine_sensors, "sync_world_pose", None)
        if callable(sync):
            base_z = float(getattr(self.stage, "base_z", 0.0) or 0.0)
            sync(self.x, self.y, self.yaw, base_z=base_z)
        physics = self.physics
        if physics is not None:
            physics.render_frame()
        else:
            update_app()
        return self.engine_sensors.color_depth()

    def world_cloud(
        self,
        planar_x: float,
        planar_y: float,
        eye_y: float,
        h_angles: np.ndarray,
        v_angles: np.ndarray,
        range_max: float,
    ) -> np.ndarray:
        """Panoramic cloud via lidar annotator when present; else empty."""
        del planar_x, planar_y, eye_y, h_angles, v_angles
        points, unused_rings = self.engine_sensors.lidar_points(
            -math.pi, math.pi, 64, -0.5, 0.5, 8, float(range_max)
        )
        del unused_rings
        if points.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float32)
        # Sensor-frame → map frame (approx: rotate by yaw, translate).
        c = float(np.cos(self.yaw))
        s = float(np.sin(self.yaw))
        x = c * points[:, 0] - s * points[:, 1] + self.x
        y = s * points[:, 0] + c * points[:, 1] + self.y
        z = points[:, 2] + self.sensor_height
        return np.stack([x, y, z], axis=1).astype(np.float32)
