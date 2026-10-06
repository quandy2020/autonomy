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

"""Planar occupancy that moving agents must not enter."""

from __future__ import annotations

from typing import Any

import numpy as np

from autosim.scenario.arena import Arena


class Field:
    """2D occupancy grid. Cell value ``100`` is blocked."""

    def __init__(
        self,
        grid: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
    ) -> None:
        """Store the published scenario grid.

        Args:
            grid: ``(rows, cols)`` with row = y and col = x.
            resolution: Cell size in meters.
            origin_x: Map-frame x of column 0.
            origin_y: Map-frame y of row 0.
        """
        self.grid = np.asarray(grid)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)

    @classmethod
    def from_grid(cls, packed: Any) -> "Field | None":
        """Build from ``Scenario.grid``; ``None`` when the map was not built."""
        if packed is None:
            return None
        grid, resolution, origin_x, origin_y, _, _ = packed
        return cls(grid, resolution, origin_x, origin_y)

    def blocked(self, position: np.ndarray, radius: float) -> np.ndarray:
        """True where a disk overlaps an occupied cell.

        Args:
            position: ``(N, 2)`` map-frame centers.
            radius: Disk radius in meters.

        Returns:
            Boolean array of shape ``(N,)``.
        """
        points = np.asarray(position, dtype=np.float64).reshape(-1, 2)
        if points.shape[0] == 0 or self.grid.size == 0 or self.resolution <= 0.0:
            return np.zeros((points.shape[0],), dtype=bool)
        span = int(np.ceil(float(radius) / self.resolution)) + 1
        cols = np.floor((points[:, 0] - self.origin_x) / self.resolution).astype(int)
        rows = np.floor((points[:, 1] - self.origin_y) / self.resolution).astype(int)
        hit = np.zeros((points.shape[0],), dtype=bool)
        for row_offset in range(-span, span + 1):
            for col_offset in range(-span, span + 1):
                hit |= self.cell_hits(points, rows + row_offset, cols + col_offset, radius)
        return hit

    def cell_hits(
        self,
        points: np.ndarray,
        rows: np.ndarray,
        cols: np.ndarray,
        radius: float,
    ) -> np.ndarray:
        """True when each disk overlaps that occupied cell's rectangle."""
        height, width = self.grid.shape
        inside = (rows >= 0) & (cols >= 0) & (rows < height) & (cols < width)
        occupied = np.zeros((points.shape[0],), dtype=bool)
        if np.any(inside):
            occupied[inside] = self.grid[rows[inside], cols[inside]] >= 100
        if not np.any(occupied):
            return occupied
        x0 = self.origin_x + cols * self.resolution
        y0 = self.origin_y + rows * self.resolution
        nearest_x = np.clip(points[:, 0], x0, x0 + self.resolution)
        nearest_y = np.clip(points[:, 1], y0, y0 + self.resolution)
        dist2 = (points[:, 0] - nearest_x) ** 2 + (points[:, 1] - nearest_y) ** 2
        return occupied & (dist2 <= float(radius) * float(radius) + 1e-12)

    def bounce(
        self,
        start: np.ndarray,
        proposed: np.ndarray,
        velocity: np.ndarray,
        radius: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Revert an axis that enters a wall and flip that velocity component."""
        placed = np.array(proposed, dtype=np.float64, copy=True)
        moving = np.array(velocity, dtype=np.float64, copy=True)
        for axis in (0, 1):
            trial = np.array(start, dtype=np.float64, copy=True)
            trial[:, axis] = proposed[:, axis]
            bad = self.blocked(trial, radius)
            placed[bad, axis] = start[bad, axis]
            moving[bad, axis] *= -1.0
        still = self.blocked(placed, radius)
        if np.any(still):
            placed[still] = start[still]
            moving[still] = -np.asarray(velocity, dtype=np.float64)[still]
        return placed, moving

    def slide(self, start: np.ndarray, proposed: np.ndarray, radius: float) -> np.ndarray:
        """Keep the free component of a step; stay put when both axes are blocked."""
        placed = np.array(proposed, dtype=np.float64, copy=True)
        bad = self.blocked(placed, radius)
        if not np.any(bad):
            return placed
        for axis in (0, 1):
            trial = np.array(start, dtype=np.float64, copy=True)
            trial[:, axis] = proposed[:, axis]
            revert = bad & self.blocked(trial, radius)
            placed[revert, axis] = start[revert, axis]
        still = self.blocked(placed, radius)
        placed[still] = np.asarray(start, dtype=np.float64)[still]
        return placed

    def relocate(
        self,
        arena: Arena,
        position: np.ndarray,
        radius: float,
        rng: np.random.Generator,
        tries: int = 12,
    ) -> np.ndarray:
        """Resample disks that overlap an occupied cell."""
        placed = np.array(position, dtype=np.float64, copy=True)
        for _ in range(tries):
            bad = self.blocked(placed, radius)
            if not np.any(bad):
                return placed
            placed[bad] = arena.sample(rng, int(np.count_nonzero(bad)))
        return placed
