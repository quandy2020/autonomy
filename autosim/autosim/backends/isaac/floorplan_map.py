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

"""Occupancy map from InteriorAgent ``rooms.json`` (酷家乐 floorplan)."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence, Tuple

import numpy as np


class FloorplanMap:
    """Build a Map-compatible occupancy grid + wall cloud from room polygons."""

    def __init__(self, settings: Mapping[str, Any] | None = None) -> None:
        self.settings = dict(settings or {})
        self.points = np.zeros((0, 3), dtype=np.float32)
        self.occupied: Optional[np.ndarray] = None
        self.origin = np.zeros(3, dtype=np.float64)
        self.resolution = 0.0
        self.grid: tuple[np.ndarray, float, float, float, int, int] | None = None
        self.cloud_rgb: Optional[np.ndarray] = None
        self.cached_cloud: Optional[np.ndarray] = None
        self.cached_grid: Optional[
            Tuple[np.ndarray, float, float, float, int, int]
        ] = None

    @staticmethod
    def find_rooms_json(usd_path: str | Path) -> Optional[Path]:
        """Return ``rooms.json`` beside the USD/USDA when present."""
        candidate = Path(usd_path).expanduser().resolve().parent / "rooms.json"
        return candidate if candidate.is_file() else None

    @classmethod
    def load_rooms(cls, path: str | Path) -> list[np.ndarray]:
        """Parse InteriorAgent rooms.json → list of ``(N, 2)`` polygons."""
        with Path(path).open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
        rooms: list[np.ndarray] = []
        if not isinstance(payload, list):
            return rooms
        for entry in payload:
            if not isinstance(entry, Mapping):
                continue
            poly = entry.get("polygon")
            if not isinstance(poly, Sequence) or len(poly) < 3:
                continue
            arr = np.asarray(poly, dtype=np.float64).reshape(-1, 2)
            if arr.shape[0] >= 3:
                rooms.append(arr)
        return rooms

    def build_from_rooms(self, rooms: Sequence[np.ndarray]) -> "FloorplanMap":
        """Rasterize room polygons into free/occupied cells and a wall cloud."""
        if not rooms:
            raise ValueError("rooms list is empty")
        grid_cfg = self.settings.get("grid") or {}
        resolution = float(grid_cfg.get("resolution", 0.05))
        padding = float(self.settings.get("padding_m", 0.5))
        wall_thickness = float(self.settings.get("wall_thickness_m", resolution * 2.0))

        all_xy = np.concatenate(list(rooms), axis=0)
        min_x = float(all_xy[:, 0].min()) - padding
        min_y = float(all_xy[:, 1].min()) - padding
        max_x = float(all_xy[:, 0].max()) + padding
        max_y = float(all_xy[:, 1].max()) + padding
        width = max(1, int(math.ceil((max_x - min_x) / resolution)))
        height = max(1, int(math.ceil((max_y - min_y) / resolution)))

        # OccupancyGrid: row 0 = min y (ROS / autoviz convention).
        xs = min_x + (np.arange(width, dtype=np.float64) + 0.5) * resolution
        ys = min_y + (np.arange(height, dtype=np.float64) + 0.5) * resolution
        grid_x, grid_y = np.meshgrid(xs, ys)
        free = np.zeros((height, width), dtype=bool)
        for poly in rooms:
            free |= _points_in_polygon(grid_x, grid_y, poly)

        cells = np.full((height, width), 100, dtype=np.int8)
        cells[free] = 0
        # Thicken room boundaries so walls are visible / laser-clippable.
        if wall_thickness > 0.0 and free.any():
            ring = free ^ _erode_bool(free, radius_cells=max(1, int(round(wall_thickness / resolution))))
            cells[ring] = 100

        z_min = float(grid_cfg.get("z_min", 0.1))
        z_max = float(grid_cfg.get("z_max", 2.5))
        # Dense wall extrusion + floor fill (fallback when USD mesh cloud absent).
        cloud = _dense_floorplan_cloud(
            rooms,
            free_mask=free,
            origin_xy=(min_x, min_y),
            resolution=resolution,
            z_min=z_min,
            z_max=z_max,
        )
        self.points = cloud
        self.occupied = cells == 100
        self.origin = np.array([min_x, min_y, 0.0], dtype=np.float64)
        self.resolution = resolution
        self.grid = (cells, resolution, min_x, min_y, width, height)
        self.cached_cloud = cloud
        self.cached_grid = self.grid
        return self

    def set_cloud(self, cloud: np.ndarray) -> None:
        """Replace ``/overall/map`` XYZ (e.g. USD mesh samples)."""
        pts = np.asarray(cloud, dtype=np.float32).reshape(-1, 3)
        self.points = pts
        self.cached_cloud = pts
        self.cloud_rgb = None

    def sample(
        self, simulator: Any = None, origin_xy: Tuple[float, float] = (0.0, 0.0)
    ) -> Tuple[np.ndarray, np.ndarray, float, float, float, int, int]:
        """Return cached cloud and occupancy grid (API matches ``autosim.map.Map``)."""
        del simulator, origin_xy
        if self.cached_cloud is None or self.cached_grid is None:
            raise RuntimeError("floorplan map not built")
        cells, resolution, ox, oy, width, height = self.cached_grid
        return (
            self.cached_cloud,
            cells,
            float(resolution),
            float(ox),
            float(oy),
            int(width),
            int(height),
        )


def _points_in_polygon(
    xs: np.ndarray, ys: np.ndarray, polygon: np.ndarray
) -> np.ndarray:
    """Vectorized even-odd point-in-polygon for grid cell centres."""
    poly = np.asarray(polygon, dtype=np.float64).reshape(-1, 2)
    n = poly.shape[0]
    inside = np.zeros(xs.shape, dtype=bool)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        intersect = ((yi > ys) != (yj > ys)) & (
            xs < (xj - xi) * (ys - yi) / ((yj - yi) + 1e-30) + xi
        )
        inside ^= intersect
        j = i
    return inside


def _erode_bool(mask: np.ndarray, *, radius_cells: int) -> np.ndarray:
    """Binary erosion via max-pool on inverted mask (no scipy dependency)."""
    if radius_cells <= 0:
        return mask.copy()
    inv = ~mask
    out = inv.copy()
    h, w = mask.shape
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            y0 = max(0, dy)
            y1 = min(h, h + dy)
            x0 = max(0, dx)
            x1 = min(w, w + dx)
            sy0 = max(0, -dy)
            sy1 = sy0 + (y1 - y0)
            sx0 = max(0, -dx)
            sx1 = sx0 + (x1 - x0)
            out[y0:y1, x0:x1] |= inv[sy0:sy1, sx0:sx1]
    return ~out


def _dense_floorplan_cloud(
    rooms: Sequence[np.ndarray],
    *,
    free_mask: np.ndarray,
    origin_xy: Tuple[float, float],
    resolution: float,
    z_min: float,
    z_max: float,
) -> np.ndarray:
    """Walls extruded through height + floor samples inside free cells."""
    step = max(float(resolution), 1e-3)
    z0 = min(float(z_min), float(z_max))
    z1 = max(float(z_min), float(z_max))
    n_z = max(2, int(math.ceil((z1 - z0) / step)) + 1)
    z_levels = np.linspace(z0, z1, n_z, dtype=np.float64)
    chunks: list[np.ndarray] = [_wall_cloud(rooms, z_levels=z_levels, spacing=step)]

    # Floor fill (every other free cell keeps size reasonable).
    rows, cols = np.where(free_mask)
    if rows.size > 0:
        take = slice(None, None, 2)
        fx = origin_xy[0] + (cols[take].astype(np.float64) + 0.5) * step
        fy = origin_xy[1] + (rows[take].astype(np.float64) + 0.5) * step
        floor = np.column_stack(
            [fx, fy, np.full(fx.shape[0], z0, dtype=np.float64)]
        ).astype(np.float32)
        chunks.append(floor)
    return np.vstack([c for c in chunks if c.size > 0])


def _wall_cloud(
    rooms: Sequence[np.ndarray],
    *,
    z_levels: Sequence[float],
    spacing: float,
) -> np.ndarray:
    """Sample XYZ points along room polygon edges at given heights."""
    step = max(float(spacing), 1e-3)
    chunks: list[np.ndarray] = []
    for poly in rooms:
        pts = np.asarray(poly, dtype=np.float64).reshape(-1, 2)
        closed = np.vstack([pts, pts[:1]])
        for i in range(closed.shape[0] - 1):
            a = closed[i]
            b = closed[i + 1]
            length = float(np.linalg.norm(b - a))
            if length < 1e-6:
                continue
            n = max(2, int(math.ceil(length / step)) + 1)
            t = np.linspace(0.0, 1.0, n, dtype=np.float64)
            xy = a[None, :] + t[:, None] * (b - a)[None, :]
            for z in z_levels:
                xyz = np.column_stack(
                    [xy[:, 0], xy[:, 1], np.full(n, float(z), dtype=np.float64)]
                )
                chunks.append(xyz.astype(np.float32))
    if not chunks:
        return np.zeros((0, 3), dtype=np.float32)
    return np.vstack(chunks)
