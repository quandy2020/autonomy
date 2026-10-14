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

"""Isaac camera / lidar annotators → Habitat-shaped numpy arrays."""

from __future__ import annotations

import math
from typing import Any, Optional, Tuple

import numpy as np


class IsaacSensors:
    """Read RGB-D / lidar from Isaac Replicator annotators when available.

    When annotators are not attached (headless pose-only), methods return
    empty / placeholder arrays matching Habitat shapes so Sensors / Runner
    keep working.
    """

    def __init__(
        self,
        *,
        width: int = 640,
        height: int = 480,
        eye_height: float = 0.5,
    ) -> None:
        self.width = max(1, int(width))
        self.height = max(1, int(height))
        self.eye_height = float(eye_height)
        self._rgb_annotator: Any = None
        self._depth_annotator: Any = None
        self._lidar_annotator: Any = None

    def attach(
        self,
        *,
        rgb: Any = None,
        depth: Any = None,
        lidar: Any = None,
    ) -> None:
        """Bind optional Replicator annotators (no-op when ``None``)."""
        self._rgb_annotator = rgb
        self._depth_annotator = depth
        self._lidar_annotator = lidar

    def color_depth(self) -> Tuple[np.ndarray, np.ndarray]:
        color = self._read_rgb()
        depth = self._read_depth()
        return color, depth

    def _read_rgb(self) -> np.ndarray:
        if self._rgb_annotator is not None:
            data = np.asarray(self._rgb_annotator.get_data())
            if data.ndim == 3 and data.shape[-1] >= 3:
                return data[..., :3].astype(np.uint8)
        out = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        out[..., 2] = 64
        return out

    def _read_depth(self) -> np.ndarray:
        if self._depth_annotator is not None:
            data = np.asarray(self._depth_annotator.get_data(), dtype=np.float32)
            if data.ndim == 3:
                data = data[..., 0]
            return data.reshape(self.height, self.width)
        return np.full((self.height, self.width), np.nan, dtype=np.float32)

    def laser_ranges(
        self,
        angle_min: float,
        angle_max: float,
        num_beams: int,
        range_max: float,
        *,
        pose: Tuple[float, float, float],
    ) -> np.ndarray:
        """Prefer lidar annotator; else empty (NaN) planar scan."""
        del pose
        cloud = self._lidar_xy()
        if cloud is None:
            return np.full((int(num_beams),), np.nan, dtype=np.float32)
        angles = np.linspace(
            float(angle_min), float(angle_max), num=int(num_beams), dtype=np.float64
        )
        ranges = np.full(angles.shape, np.nan, dtype=np.float32)
        if cloud.shape[0] == 0:
            return ranges
        pts_angles = np.arctan2(cloud[:, 1], cloud[:, 0])
        pts_ranges = np.linalg.norm(cloud[:, :2], axis=1)
        # Nearest point in each angular bin.
        step = (float(angle_max) - float(angle_min)) / max(int(num_beams) - 1, 1)
        for i, ang in enumerate(angles):
            mask = np.abs(((pts_angles - ang + math.pi) % (2 * math.pi)) - math.pi) <= (
                0.5 * step + 1e-6
            )
            if not np.any(mask):
                continue
            hit = float(np.min(pts_ranges[mask]))
            if hit <= float(range_max):
                ranges[i] = hit
        return ranges

    def lidar_points(
        self,
        h_min: float,
        h_max: float,
        h_beams: int,
        v_min: float,
        v_max: float,
        v_rings: int,
        range_max: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        del h_min, h_max, h_beams, v_min, v_max, v_rings
        cloud = self._lidar_xyz()
        if cloud is None or cloud.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=np.float32)
        dist = np.linalg.norm(cloud, axis=1)
        mask = dist <= float(range_max)
        cloud = cloud[mask]
        rings = np.zeros((cloud.shape[0],), dtype=np.float32)
        return cloud.astype(np.float32), rings

    def _lidar_xy(self) -> Optional[np.ndarray]:
        xyz = self._lidar_xyz()
        if xyz is None:
            return None
        return xyz[:, :2]

    def _lidar_xyz(self) -> Optional[np.ndarray]:
        if self._lidar_annotator is None:
            return None
        data = np.asarray(self._lidar_annotator.get_data(), dtype=np.float32)
        if data.size == 0:
            return np.zeros((0, 3), dtype=np.float32)
        return data.reshape(-1, data.shape[-1])[:, :3]
