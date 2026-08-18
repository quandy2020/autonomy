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

"""Ground-truth map: panoramic PLY / point cloud and 2D occupancy grid."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, Optional, Tuple

import numpy as np


class Map:
    """Sample a map-frame panoramic cloud, write PLY, and project a 2D grid."""

    def __init__(self, settings: Mapping[str, Any]) -> None:
        """Bind ``habitat.map`` configuration.

        Args:
            settings: ``habitat.map`` mapping.
        """
        self.settings = dict(settings)
        self.cached_cloud: Optional[np.ndarray] = None
        self.cached_grid: Optional[Tuple[np.ndarray, float, float, float, int, int]] = None

    def sample(
        self, simulator: Any, origin_xy: Tuple[float, float]
    ) -> Tuple[np.ndarray, np.ndarray, float, float, float, int, int]:
        """Cast panoramic rays, optionally write PLY, and build occupancy.

        Args:
            simulator: Backend with ``world_cloud`` / ``linspace_angles`` / ``eye_height``.
            origin_xy: Planar map origin ``(x, y)`` (usually spawn).

        Returns:
            ``(cloud_xyz, grid, resolution, origin_x, origin_y, width, height)``.
        """
        cloud = self.sample_ply_cloud(simulator, origin_xy)
        ply_cfg = self.settings.get("ply") or {}
        file_path = str(ply_cfg.get("file") or "").strip()
        if file_path:
            self.write_ply(self.resolve_ply_path(file_path), cloud)

        grid_cfg = self.settings["grid"]
        eye = float(simulator.eye_height())
        origin = (float(origin_xy[0]), float(origin_xy[1]), eye)
        grid, resolution, ox, oy, width, height = self.project_grid(cloud, grid_cfg, origin)
        self.cached_cloud = cloud
        self.cached_grid = (grid, resolution, ox, oy, width, height)
        return cloud, grid, resolution, ox, oy, width, height

    def sample_ply_cloud(self, simulator: Any, origin_xy: Tuple[float, float]) -> np.ndarray:
        """Panoramic Habitat cloud in map frame (z-up) from ``map.ply`` FOV."""
        ply_cfg = self.settings["ply"]
        horizontal = ply_cfg["horizontal"]
        vertical = ply_cfg["vertical"]
        range_max = float(ply_cfg.get("range_max", self.settings.get("range_max", 30.0)))
        h_angles = simulator.linspace_angles(
            float(horizontal["angle_min"]),
            float(horizontal["angle_max"]),
            int(horizontal["num_beams"]),
        )
        v_angles = simulator.linspace_angles(
            float(vertical["angle_min"]),
            float(vertical["angle_max"]),
            int(vertical["num_rings"]),
        )
        eye = float(simulator.eye_height())
        return simulator.world_cloud(
            float(origin_xy[0]),
            float(origin_xy[1]),
            eye,
            h_angles,
            v_angles,
            range_max,
        )

    @staticmethod
    def resolve_ply_path(path: str) -> Path:
        """Resolve PLY path relative to the autosim project root when not absolute."""
        candidate = Path(path)
        if candidate.is_absolute():
            return candidate
        root = Path(__file__).resolve().parents[1]
        return (root / candidate).resolve()

    @staticmethod
    def write_ply(path: Path, cloud: np.ndarray) -> None:
        """Write an ASCII PLY (xyz only) for the panoramic cloud."""
        points = np.asarray(cloud, dtype=np.float64).reshape(-1, 3)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as handle:
            handle.write("ply\nformat ascii 1.0\n")
            handle.write(f"element vertex {points.shape[0]}\n")
            handle.write("property float x\nproperty float y\nproperty float z\n")
            handle.write("end_header\n")
            for x, y, z in points:
                handle.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

    def project_grid(
        self,
        cloud: np.ndarray,
        grid_cfg: Mapping[str, Any],
        origin: Tuple[float, float, float],
    ) -> Tuple[np.ndarray, float, float, float, int, int]:
        """Height-slice occupancy with free-space ray carving from ``origin``.

        Cells: ``-1`` unknown, ``0`` free (carved), ``100`` occupied.
        """
        resolution = float(grid_cfg["resolution"])
        z_min = float(grid_cfg["z_min"])
        z_max = float(grid_cfg["z_max"])
        if cloud.size == 0:
            return np.full((1, 1), -1, dtype=np.int8), resolution, 0.0, 0.0, 1, 1

        points = np.asarray(cloud, dtype=np.float64).reshape(-1, 3)
        mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        sliced = points[mask]
        if sliced.size == 0:
            return np.full((1, 1), -1, dtype=np.int8), resolution, 0.0, 0.0, 1, 1

        min_x = float(np.floor(min(sliced[:, 0].min(), origin[0]) / resolution) * resolution)
        min_y = float(np.floor(min(sliced[:, 1].min(), origin[1]) / resolution) * resolution)
        max_x = float(np.ceil(max(sliced[:, 0].max(), origin[0]) / resolution) * resolution)
        max_y = float(np.ceil(max(sliced[:, 1].max(), origin[1]) / resolution) * resolution)
        width = max(1, int(round((max_x - min_x) / resolution)))
        height = max(1, int(round((max_y - min_y) / resolution)))
        grid = np.full((height, width), -1, dtype=np.int8)

        origin_c = int(np.floor((origin[0] - min_x) / resolution))
        origin_r = int(np.floor((origin[1] - min_y) / resolution))
        cols = np.floor((sliced[:, 0] - min_x) / resolution).astype(np.int32)
        rows = np.floor((sliced[:, 1] - min_y) / resolution).astype(np.int32)
        valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
        cols = cols[valid]
        rows = rows[valid]

        for row, col in zip(rows.tolist(), cols.tolist()):
            self.carve_bresenham(grid, origin_r, origin_c, row, col)
            if 0 <= row < height and 0 <= col < width:
                grid[row, col] = 100
        return grid, resolution, min_x, min_y, width, height

    @staticmethod
    def carve_bresenham(grid: np.ndarray, r0: int, c0: int, r1: int, c1: int) -> None:
        """Mark free cells along a grid ray; leave the endpoint for the caller."""
        height, width = grid.shape
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc
        row, col = r0, c0
        while row != r1 or col != c1:
            if 0 <= row < height and 0 <= col < width and grid[row, col] != 100:
                grid[row, col] = 0
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                row += sr
            if e2 < dr:
                err += dr
                col += sc
