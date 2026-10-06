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

"""Boolean occupancy volume shared by every map generator."""

from __future__ import annotations

import numpy as np


class Volume:
    """Voxel grid: shape, binning, spawn carve, and the 2D occupancy slice."""

    MAX_CELLS = 4_000_000

    @staticmethod
    def shape(lengths: tuple[float, float, float], resolution: float) -> tuple[int, int, int]:
        """Voxel counts, truncating like mockamap's ``int`` cast.

        Raises:
            ValueError: A length is not positive.
        """
        counts = []
        for length in lengths:
            if length <= 0.0:
                raise ValueError("scenario map length must be > 0")
            counts.append(max(1, int(length / resolution + 1e-9)))
        return counts[0], counts[1], counts[2]

    @staticmethod
    def origin(shape: tuple[int, int, int], resolution: float) -> np.ndarray:
        """Minimum corner of voxel ``(0,0,0)``. X/Y are centered; Z starts at 0."""
        return np.array(
            [-0.5 * shape[0] * resolution, -0.5 * shape[1] * resolution, 0.0],
            dtype=np.float64,
        )

    @staticmethod
    def from_points(
        points: np.ndarray, resolution: float
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """Bin a cloud. Refuses grids above :attr:`MAX_CELLS`.

        Returns:
            ``(occupied[x,y,z], origin_xyz, resolution)``.

        Raises:
            ValueError: The cloud is empty or the grid is too large.
        """
        if points.shape[0] == 0:
            raise ValueError("scenario cloud is empty")
        low = np.floor(points.min(axis=0) / resolution) * resolution
        high = points.max(axis=0)
        counts = np.maximum(1, np.ceil((high - low) / resolution + 1e-9).astype(int))
        if int(np.prod(counts)) > Volume.MAX_CELLS:
            raise ValueError("scenario voxel volume is too large; increase resolution")
        shape = (int(counts[0]), int(counts[1]), int(counts[2]))
        index = np.floor((points - low) / resolution).astype(np.int32)
        index = np.clip(index, 0, np.array(shape, dtype=np.int32) - 1)
        occupied = np.zeros(shape, dtype=bool)
        occupied[index[:, 0], index[:, 1], index[:, 2]] = True
        return occupied, low.astype(np.float64), resolution

    @staticmethod
    def clear_spawn(
        occupied: np.ndarray,
        origin: np.ndarray,
        resolution: float,
        radius: float,
        height: float,
    ) -> None:
        """Free a vertical cylinder at the origin up to ``height`` meters."""
        if radius <= 0.0 or height <= 0.0:
            return
        xs = origin[0] + (np.arange(occupied.shape[0]) + 0.5) * resolution
        ys = origin[1] + (np.arange(occupied.shape[1]) + 0.5) * resolution
        zs = origin[2] + (np.arange(occupied.shape[2]) + 0.5) * resolution
        xx, yy = np.meshgrid(xs, ys, indexing="ij")
        near = (xx * xx + yy * yy) <= radius * radius
        occupied[near[:, :, None] & (zs <= height)[None, None, :]] = False

    @staticmethod
    def corners(occupied: np.ndarray, origin: np.ndarray, resolution: float) -> np.ndarray:
        """Occupied voxel corners, matching mockamap's ``index / scale`` samples."""
        index = np.argwhere(occupied)
        if index.size == 0:
            return np.zeros((0, 3), dtype=np.float32)
        return (origin + index.astype(np.float64) * resolution).astype(np.float32)

    @staticmethod
    def planar(
        occupied: np.ndarray,
        origin: np.ndarray,
        resolution: float,
        z_max: float | None = None,
    ) -> tuple[np.ndarray, float, float, float, int, int]:
        """2D grid from voxel centers in ``[0.1, z_max]`` meters.

        Returns:
            ``(grid[y, x], resolution, origin_x, origin_y, width, height)``
            with 100 occupied and 0 free.
        """
        centers = origin[2] + (np.arange(occupied.shape[2]) + 0.5) * resolution
        use = centers >= 0.1
        if z_max is not None and z_max > 0.1:
            use &= centers <= z_max
        layers = occupied[:, :, use]
        columns = layers.any(axis=2) if layers.size else np.zeros(occupied.shape[:2], dtype=bool)
        grid = np.where(columns.T, 100, 0).astype(np.int8)
        return (
            grid,
            float(resolution),
            float(origin[0]),
            float(origin[1]),
            int(occupied.shape[0]),
            int(occupied.shape[1]),
        )
