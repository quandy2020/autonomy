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

"""Exteroceptive sampling: 2D laser, 3D lidar cloud, and RGB-D."""

from __future__ import annotations

from typing import Any, Mapping, Optional, Tuple

import numpy as np


class Sensors:
    """Sample laser, lidar points, and camera observations from the simulator."""

    def __init__(
        self,
        angle_min: float,
        angle_max: float,
        num_beams: int,
        range_min: float,
        range_max: float,
        noise: float = 0.0,
        depth_noise: float = 0.0,
        lidar_3d: Optional[Mapping[str, Any]] = None,
        seed: int = 0,
    ) -> None:
        """Configure 2D laser geometry and optional 3D lidar parameters.

        Args:
            angle_min: 2D scan start angle (rad).
            angle_max: 2D scan end angle (rad).
            num_beams: 2D beam count.
            range_min: Minimum valid range (m) for 2D clipping.
            range_max: Maximum valid range (m) for 2D.
            noise: Gaussian stddev on ranges / distances; ``0`` disables.
            depth_noise: Gaussian stddev on camera depth (m); ``0`` disables.
            lidar_3d: Optional ``habitat.sensors.lidar_3d`` mapping.
            seed: RNG seed for noise.
        """
        self.angle_min = float(angle_min)
        self.angle_max = float(angle_max)
        self.num_beams = int(num_beams)
        self.range_min = float(range_min)
        self.range_max = float(range_max)
        self.noise = float(noise)
        self.depth_noise = float(depth_noise)
        self.lidar_3d = dict(lidar_3d or {})
        self.rng = np.random.default_rng(seed)
        self.depth_min_valid = 1e-3

    @property
    def angle_increment(self) -> float:
        """Angular resolution between adjacent 2D beams (rad).

        Returns:
            ``(angle_max - angle_min) / (num_beams - 1)``; ``0`` for a single beam.
        """
        if self.num_beams <= 1:
            return 0.0
        return (self.angle_max - self.angle_min) / float(self.num_beams - 1)

    def sample_laser(self, simulator: object) -> np.ndarray:
        """Sample one planar laser frame and clip to the valid range.

        Args:
            simulator: Backend implementing ``laser_ranges(...)``.

        Returns:
            ``float32`` range array of shape ``(num_beams,)`` in meters.
        """
        ranges = simulator.laser_ranges(
            self.angle_min, self.angle_max, self.num_beams, self.range_max
        ).astype(np.float32)
        valid = np.isfinite(ranges) & (ranges >= self.range_min) & (ranges <= self.range_max)
        if self.noise > 0.0 and np.any(valid):
            noisy = ranges.copy()
            noisy[valid] = noisy[valid] + self.rng.normal(
                0.0, self.noise, size=int(valid.sum())
            ).astype(np.float32)
            noisy[valid] = np.clip(noisy[valid], self.range_min, self.range_max)
            ranges = noisy
        out = np.full(ranges.shape, np.inf, dtype=np.float32)
        out[valid] = ranges[valid]
        return out

    def sample_points(self, simulator: object) -> np.ndarray:
        """Sample one 3D lidar point cloud in the sensor frame.

        Args:
            simulator: Backend implementing ``lidar_points(...)``.

        Returns:
            ``Nx3 float32`` points; empty if no hits and not mocking.
        """
        cfg = self.lidar_3d
        horizontal = cfg["horizontal"]
        vertical = cfg["vertical"]
        range_min = float(cfg.get("range_min", self.range_min))
        range_max = float(cfg.get("range_max", self.range_max))
        noise = float(cfg.get("noise", 0.0))
        points = simulator.lidar_points(
            float(horizontal["angle_min"]),
            float(horizontal["angle_max"]),
            int(horizontal["num_beams"]),
            float(vertical["angle_min"]),
            float(vertical["angle_max"]),
            int(vertical["num_rings"]),
            range_max,
        ).astype(np.float32)
        if points.size == 0:
            return points.reshape(0, 3)
        if noise > 0.0:
            norms = np.linalg.norm(points, axis=1, keepdims=True)
            norms = np.maximum(norms, 1e-6)
            directions = points / norms
            delta = self.rng.normal(0.0, noise, size=(points.shape[0], 1)).astype(np.float32)
            points = points + directions * delta
        distances = np.linalg.norm(points, axis=1)
        mask = (distances >= range_min) & (distances <= range_max)
        return points[mask]

    def sample_camera(self, simulator: object) -> Tuple[np.ndarray, np.ndarray]:
        """Sample one color frame and the aligned depth map.

        Args:
            simulator: Backend implementing ``color_depth()``.

        Returns:
            ``(color, depth)`` where color is ``uint8`` HxWx3 and depth is ``float32`` HxW.
        """
        color, depth = simulator.color_depth()
        depth = np.squeeze(depth).astype(np.float32, copy=False)
        if depth.ndim != 2:
            depth = depth.reshape(color.shape[0], color.shape[1])
        if self.depth_noise > 0.0:
            valid = depth > self.depth_min_valid
            if np.any(valid):
                noisy = depth.copy()
                noisy[valid] = noisy[valid] + self.rng.normal(
                    0.0,
                    self.depth_noise,
                    size=int(valid.sum()),
                ).astype(np.float32)
                noisy[valid] = np.maximum(noisy[valid], self.depth_min_valid)
                depth = noisy
        return color, depth
