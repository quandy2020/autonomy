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

"""RGBD local point cloud via pinhole ray casting (no Habitat)."""

from __future__ import annotations

import math
from typing import Any, Mapping, Tuple

import numpy as np

from autosim.messages import Messages
from autosim.scenario.map.volume import Volume


class Sensor:
    """Pinhole RGBD depth buffer → 3D cloud (one hit per pixel, occluded hidden)."""

    def __init__(self, camera: Mapping[str, Any]) -> None:
        """Bind ``habitat.sensors.camera`` optics.

        Args:
            camera: Camera mapping (width, height, hfov_deg, …).
        """
        self.width = max(1, int(camera.get("width", 640)))
        self.height = max(1, int(camera.get("height", 480)))
        self.hfov_deg = float(camera.get("hfov_deg", 90.0))
        if "vfov_deg" in camera:
            self.vfov_deg = float(camera["vfov_deg"])
        else:
            half_h = math.radians(self.hfov_deg) * 0.5
            aspect = float(self.height) / float(self.width)
            self.vfov_deg = math.degrees(2.0 * math.atan(math.tan(half_h) * aspect))
        self.sensor_height = float(camera.get("sensor_height", 0.6))
        self.range_min = float(camera.get("range_min", 0.1))
        self.range_max = float(camera.get("range_max", 30.0))
        self.stride = max(1, int(camera.get("depth_points_stride", 1)))
        self.matrix = Messages.camera_intrinsics(
            self.width, self.height, self.hfov_deg, self.vfov_deg
        )

    def sample_points(
        self,
        occupied: np.ndarray,
        volume_origin: np.ndarray,
        resolution: float,
        pose: Tuple[float, float, float],
        *,
        traffic: Any | None = None,
    ) -> np.ndarray:
        """Render a depth buffer then back-project to ``camera_link`` XYZ.

        Each (strided) pixel casts one pinhole ray. Only the **nearest** hit
        among voxels / ground / dynamic bodies is kept — surfaces behind that
        hit are discarded (true occlusion). Result is a 3D point cloud.

        Args:
            occupied: Boolean volume ``(nx, ny, nz)`` from the scenario.
            volume_origin: Map-frame minimum corner of voxel ``(0,0,0)``.
            resolution: Voxel edge length (meters).
            pose: Robot ``(x, y, yaw)`` in the map frame.
            traffic: Optional :class:`~autosim.scenario.traffic.Traffic`.

        Returns:
            ``Mx3`` float32 in REP-103 ``camera_link`` (x forward, y left, z up).
        """
        depth, uu, vv = self.render_depth(
            occupied, volume_origin, resolution, pose, traffic=traffic
        )
        return self.back_project(depth, uu, vv)

    def render_depth(
        self,
        occupied: np.ndarray,
        volume_origin: np.ndarray,
        resolution: float,
        pose: Tuple[float, float, float],
        *,
        traffic: Any | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Per-pixel Euclidean range along the optical ray (occlusion buffer)."""
        fx = float(self.matrix[0])
        fy = float(self.matrix[4])
        cx = float(self.matrix[2])
        cy = float(self.matrix[5])
        us = np.arange(0, self.width, self.stride, dtype=np.float64) + 0.5
        vs = np.arange(0, self.height, self.stride, dtype=np.float64) + 0.5
        uu, vv = np.meshgrid(us, vs, indexing="xy")
        if fx <= 0.0 or fy <= 0.0:
            empty = np.full(uu.shape, np.nan, dtype=np.float64)
            return empty, uu, vv

        # Optical unit rays: X right, Y down, Z forward.
        optical = np.stack(
            [(uu - cx) / fx, (vv - cy) / fy, np.ones_like(uu)],
            axis=-1,
        ).reshape(-1, 3)
        norms = np.linalg.norm(optical, axis=1, keepdims=True)
        optical = optical / np.maximum(norms, 1e-12)

        # optical → camera_link → map.
        cam = np.stack([optical[:, 2], -optical[:, 0], -optical[:, 1]], axis=1)
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        map_dirs = np.stack(
            [
                cosine * cam[:, 0] - sine * cam[:, 1],
                sine * cam[:, 0] + cosine * cam[:, 1],
                cam[:, 2],
            ],
            axis=1,
        )
        ray_origin = np.array([x, y, self.sensor_height], dtype=np.float64)

        depths = Volume.cast_hits(
            occupied,
            volume_origin,
            float(resolution),
            ray_origin,
            map_dirs,
            self.range_min,
            self.range_max,
            ground_z=0.0,
        )
        if traffic is not None and hasattr(traffic, "nearest_batch"):
            dynamic = traffic.nearest_batch(ray_origin, map_dirs, self.range_max)
            better = np.isfinite(dynamic) & (dynamic >= self.range_min) & (
                np.isnan(depths) | (dynamic < depths)
            )
            depths = np.array(depths, dtype=np.float64, copy=True)
            depths[better] = dynamic[better]

        depths = depths.reshape(uu.shape)
        invalid = ~(
            np.isfinite(depths) & (depths > self.range_min) & (depths < self.range_max)
        )
        depths = np.array(depths, dtype=np.float64, copy=True)
        depths[invalid] = np.nan
        return depths, uu, vv

    def back_project(
        self, depth: np.ndarray, uu: np.ndarray, vv: np.ndarray
    ) -> np.ndarray:
        """Depth buffer → 3D ``camera_link`` points (invalid pixels omitted)."""
        fx = float(self.matrix[0])
        fy = float(self.matrix[4])
        cx = float(self.matrix[2])
        cy = float(self.matrix[5])
        valid = np.isfinite(depth)
        if not np.any(valid):
            return np.zeros((0, 3), dtype=np.float32)
        # Euclidean range along unit optical ray → optical XYZ, then camera_link.
        u = uu[valid]
        v = vv[valid]
        optical = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=1)
        norms = np.linalg.norm(optical, axis=1, keepdims=True)
        optical = optical / np.maximum(norms, 1e-12)
        optical_pts = optical * depth[valid, None]
        return Messages.optical_to_camera_link(optical_pts.astype(np.float32))
