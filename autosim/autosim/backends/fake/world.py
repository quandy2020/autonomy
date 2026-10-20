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

"""Procedural scenario WorldBackend (no Habitat / Isaac)."""

from __future__ import annotations

import math
from typing import Any, Mapping, Optional, Tuple

import numpy as np

from autosim.backends.capabilities import FAKE_CAPABILITIES
from autosim.backends.fake.map import Map as FakeMap
from autosim.backends.fake.sensor import Sensor as FakeSensor
from autosim.scene.gen.volume import Volume
from autosim.robot.urdf import UrdfModel


class FakeWorld:
    """WorldBackend backed by scenario occupancy + pinhole raycast."""

    capabilities = FAKE_CAPABILITIES

    def __init__(
        self,
        *,
        camera: Mapping[str, Any],
        map_cfg: Mapping[str, Any] | None,
        scenario_cfg: Mapping[str, Any] | None,
        urdf_path: str = "",
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.traffic: Any = None
        try:
            self.urdf = UrdfModel.load(str(urdf_path or ""))
        except (FileNotFoundError, ValueError):
            self.urdf = None
        self.ray_sensor = FakeSensor(camera)
        self.width = int(self.ray_sensor.width)
        self.height = int(self.ray_sensor.height)
        self.map_builder: Optional[FakeMap] = None
        if map_cfg and map_cfg.get("enabled", False) and scenario_cfg:
            self.map_builder = FakeMap(map_cfg).build(scenario_cfg)

    @classmethod
    def from_settings(cls, settings: Mapping[str, Any]) -> "FakeWorld":
        """Build from full config ``{"habitat": {...}}``."""
        habitat = settings.get("habitat") if isinstance(settings.get("habitat"), Mapping) else settings
        if not isinstance(habitat, Mapping):
            raise ValueError("settings must contain a habitat mapping")
        sensors = habitat.get("sensors") or {}
        camera = sensors.get("camera") or {}
        spawn = habitat.get("spawn") or [0.0, 0.0, 0.0]
        robot = habitat.get("robot") or {}
        return cls(
            camera=camera,
            map_cfg=habitat.get("map"),
            scenario_cfg=habitat.get("scenario"),
            urdf_path=str(robot.get("urdf") or ""),
            x=float(spawn[0]),
            y=float(spawn[1]),
            yaw=float(spawn[2]),
        )

    def reset(self, x: float, y: float, yaw: float) -> None:
        self.set_pose(x, y, yaw)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)

    def pose(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.yaw

    def step(self) -> None:
        return None

    def close(self) -> None:
        return None

    def eye_height(self) -> float:
        return float(self.ray_sensor.sensor_height)

    def linspace_angles(
        self, angle_min: float, angle_max: float, count: int
    ) -> np.ndarray:
        if count <= 1:
            return np.array([float(angle_min)], dtype=np.float64)
        return np.linspace(
            float(angle_min), float(angle_max), num=int(count), dtype=np.float64
        )

    def occupancy_volume(self) -> tuple[np.ndarray, np.ndarray, float] | None:
        built = self.map_builder
        if built is None or built.occupied is None:
            return None
        return built.occupied, built.origin, float(built.resolution)

    def laser_ranges(
        self,
        angle_min: float,
        angle_max: float,
        num_beams: int,
        range_max: float,
    ) -> np.ndarray:
        angles = self.linspace_angles(angle_min, angle_max, num_beams)
        volume = self.occupancy_volume()
        if volume is None:
            return np.full((int(num_beams),), np.nan, dtype=np.float32)
        occupied, origin, resolution = volume
        eye = self.eye_height()
        yaw0 = self.yaw
        dirs = np.stack(
            [
                np.cos(yaw0 + angles),
                np.sin(yaw0 + angles),
                np.zeros_like(angles),
            ],
            axis=1,
        )
        ray_origin = np.array([self.x, self.y, eye], dtype=np.float64)
        depths = Volume.cast_hits(
            occupied,
            origin,
            resolution,
            ray_origin,
            dirs,
            0.05,
            float(range_max),
            ground_z=0.0,
        )
        if self.traffic is not None and hasattr(self.traffic, "nearest_batch"):
            dynamic = self.traffic.nearest_batch(ray_origin, dirs, float(range_max))
            better = np.isfinite(dynamic) & (
                np.isnan(depths) | (dynamic < depths)
            )
            depths = np.array(depths, dtype=np.float64, copy=True)
            depths[better] = dynamic[better]
        return depths.astype(np.float32)

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
        volume = self.occupancy_volume()
        if volume is None:
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=np.float32)
        occupied, origin, resolution = volume
        h = self.linspace_angles(h_min, h_max, h_beams)
        v = self.linspace_angles(v_min, v_max, v_rings)
        yaw0 = self.yaw
        eye = self.eye_height()
        ray_origin = np.array([self.x, self.y, eye], dtype=np.float64)
        points: list[tuple[float, float, float]] = []
        rings: list[float] = []
        for ring_id, pitch in enumerate(v):
            cp = math.cos(float(pitch))
            sp = math.sin(float(pitch))
            dirs = np.stack(
                [
                    np.cos(yaw0 + h) * cp,
                    np.sin(yaw0 + h) * cp,
                    np.full(h.shape, sp),
                ],
                axis=1,
            )
            depths = Volume.cast_hits(
                occupied,
                origin,
                resolution,
                ray_origin,
                dirs,
                0.05,
                float(range_max),
                ground_z=0.0,
            )
            if self.traffic is not None and hasattr(self.traffic, "nearest_batch"):
                dynamic = self.traffic.nearest_batch(
                    ray_origin, dirs, float(range_max)
                )
                better = np.isfinite(dynamic) & (
                    np.isnan(depths) | (dynamic < depths)
                )
                depths = np.array(depths, dtype=np.float64, copy=True)
                depths[better] = dynamic[better]
            for i, depth in enumerate(depths):
                if not np.isfinite(depth):
                    continue
                # Sensor frame: x forward, y left, z up.
                local_yaw = float(h[i])
                points.append(
                    (
                        float(depth) * math.cos(local_yaw) * cp,
                        float(depth) * math.sin(local_yaw) * cp,
                        float(depth) * sp,
                    )
                )
                rings.append(float(ring_id))
        if not points:
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=np.float32)
        return (
            np.asarray(points, dtype=np.float32),
            np.asarray(rings, dtype=np.float32),
        )

    def color_depth(self) -> Tuple[np.ndarray, np.ndarray]:
        volume = self.occupancy_volume()
        color = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        color[..., 1] = 128
        if volume is None:
            depth = np.full((self.height, self.width), np.nan, dtype=np.float32)
            return color, depth
        occupied, origin, resolution = volume
        depth, _, _ = self.ray_sensor.render_depth(
            occupied,
            origin,
            resolution,
            (self.x, self.y, self.yaw),
            traffic=self.traffic,
        )
        return color, depth.astype(np.float32)

    def sample_depth_points(self) -> np.ndarray:
        """RGBD FoV cloud in ``camera_link`` (fake raycast path)."""
        volume = self.occupancy_volume()
        if volume is None:
            return np.zeros((0, 3), dtype=np.float32)
        occupied, origin, resolution = volume
        return self.ray_sensor.sample_points(
            occupied,
            origin,
            resolution,
            (self.x, self.y, self.yaw),
            traffic=self.traffic,
        )

    def world_cloud(
        self,
        planar_x: float,
        planar_y: float,
        eye_y: float,
        h_angles: np.ndarray,
        v_angles: np.ndarray,
        range_max: float,
    ) -> np.ndarray:
        del planar_x, planar_y, eye_y, h_angles, v_angles, range_max
        if self.map_builder is None:
            return np.zeros((0, 3), dtype=np.float32)
        return np.asarray(self.map_builder.points, dtype=np.float32).reshape(-1, 3)
