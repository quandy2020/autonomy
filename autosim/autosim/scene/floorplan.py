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

"""InteriorAgent ``rooms.json`` → OccupancyGrid / wall cloud (engine-agnostic)."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


class FloorplanMap:
    """Build a Map-compatible occupancy grid + wall cloud from room polygons."""

    def __init__(self, settings: Mapping[str, Any] | None = None) -> None:
        self.settings = dict(settings or {})
        self.points = np.zeros((0, 3), dtype=np.float32)
        self.occupied: np.ndarray | None = None
        self.origin = np.zeros(3, dtype=np.float64)
        self.resolution = 0.0
        self.grid: tuple[np.ndarray, float, float, float, int, int] | None = None
        self.cloud_rgb: np.ndarray | None = None
        self.cached_cloud: np.ndarray | None = None
        self.cached_grid: tuple[np.ndarray, float, float, float, int, int] | None = None

    @staticmethod
    def find_rooms_json(usd_path: str | Path) -> Path | None:
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
            free |= points_in_polygon(grid_x, grid_y, poly)

        cells = np.full((height, width), 100, dtype=np.int8)
        cells[free] = 0
        # Thicken room boundaries so walls are visible / laser-clippable.
        if wall_thickness > 0.0 and free.any():
            ring = free ^ erode_boolean_mask(free, radius_cells=max(1, int(round(wall_thickness / resolution))))
            cells[ring] = 100

        z_min = float(grid_cfg.get("z_min", 0.0))
        z_max = float(grid_cfg.get("z_max", 2.5))
        # Dense wall extrusion + floor fill (fallback when USD mesh cloud absent).
        # Floor samples always sit at z=0 so Autoviz matches InteriorAgent ground.
        cloud = dense_floorplan_cloud(
            rooms,
            free_mask=free,
            origin_xy=(min_x, min_y),
            resolution=resolution,
            z_min=z_min,
            z_max=z_max,
            floor_z=0.0,
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

    @staticmethod
    def robot_body_band(
        settings: Mapping[str, Any] | None = None,
        *,
        robot_height_m: float | None = None,
        robot_radius_m: float | None = None,
    ) -> tuple[float, float, float]:
        """Height slab for 2D occupancy of a ground robot (metres).

        Physical rule (Nav2 / costmap style):
        - ``z < clearance``: floor / carpet — keep free (do not carve).
        - ``clearance ≤ z ≤ body_top``: intersects the robot body — occupied.
        - ``z > body_top``: overhang (tabletop, lamp) — ignore so the robot
          may pass underneath when the body clears.

        Returns:
            ``(z_min, z_max, inflate_m)`` used by ``mark_elevated_obstacles``.
        """
        cfg = dict(settings or {})
        # Explicit overrides (legacy z_obstacle_* still accepted).
        clearance = cfg.get("robot_clearance_m", cfg.get("z_obstacle_min"))
        body = cfg.get("robot_height_m", cfg.get("z_obstacle_max"))
        inflate = cfg.get("mesh_inflate_m")
        if clearance is None:
            clearance = 0.05
        if body is None:
            body = float(robot_height_m) if robot_height_m is not None else 0.35
        if inflate is None:
            inflate = float(robot_radius_m) if robot_radius_m is not None else 0.12
        z_min = max(0.0, float(clearance))
        margin = float(cfg.get("height_margin_m", 0.05))
        # Legacy z_obstacle_max is an absolute ceiling (no extra margin).
        if "robot_height_m" not in cfg and "z_obstacle_max" in cfg:
            z_max = max(z_min + 0.05, float(cfg["z_obstacle_max"]))
        else:
            z_max = max(z_min + 0.05, float(body) + margin)
        return z_min, z_max, max(0.0, float(inflate))

    def mark_elevated_obstacles(
        self,
        cloud: np.ndarray,
        *,
        z_min: float = 0.05,
        z_max: float = 0.40,
        inflate_m: float = 0.12,
    ) -> int:
        """Carve mesh hits in the robot body height slab into the grid.

        Only points with ``z_min ≤ z ≤ z_max`` mark a cell occupied (sofa seat,
        chair legs, wall base). Floor verts below ``z_min`` and overhangs above
        ``z_max`` are ignored so under-table free space stays traversable.

        Returns:
            Number of cells newly marked occupied.
        """
        if self.grid is None:
            return 0
        pts = np.asarray(cloud, dtype=np.float64).reshape(-1, 3)
        if pts.shape[0] == 0:
            return 0
        cells, resolution, ox, oy, width, height = self.grid
        res = float(resolution)
        hit = pts[
            (pts[:, 2] >= float(z_min))
            & (pts[:, 2] <= float(z_max))
            & np.isfinite(pts).all(axis=1)
        ]
        if hit.shape[0] == 0:
            return 0
        cols = np.floor((hit[:, 0] - float(ox)) / res).astype(np.int64)
        rows = np.floor((hit[:, 1] - float(oy)) / res).astype(np.int64)
        ok = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
        rows, cols = rows[ok], cols[ok]
        if rows.size == 0:
            return 0
        out = np.asarray(cells, dtype=np.int8).copy()
        before = out.copy()
        out[rows, cols] = 100
        inflate = max(0, int(round(float(inflate_m) / res)))
        if inflate > 0:
            for dy in range(-inflate, inflate + 1):
                for dx in range(-inflate, inflate + 1):
                    if dx * dx + dy * dy > inflate * inflate:
                        continue
                    rr = rows + dy
                    cc = cols + dx
                    inside = (rr >= 0) & (rr < height) & (cc >= 0) & (cc < width)
                    if not np.any(inside):
                        continue
                    out[rr[inside], cc[inside]] = 100
        changed = int(np.count_nonzero((before != 100) & (out == 100)))
        self.grid = (out, resolution, ox, oy, width, height)
        self.cached_grid = self.grid
        self.occupied = out == 100
        return changed

    def free_openings(
        self,
        cloud: np.ndarray,
        *,
        z_min: float = 0.0,
        z_max: float = 2.5,
        inflate_m: float = 0.10,
    ) -> int:
        """Punch door (etc.) footprints to free — ground robots may pass.

        InteriorAgent ``rooms.json`` wall rings seal doorway openings, and door
        leaf meshes would otherwise carve as obstacles. Cells under door verts
        in ``[z_min, z_max]`` become free (value 0).
        """
        if self.grid is None:
            return 0
        pts = np.asarray(cloud, dtype=np.float64).reshape(-1, 3)
        if pts.shape[0] == 0:
            return 0
        cells, resolution, ox, oy, width, height = self.grid
        res = float(resolution)
        hit = pts[
            (pts[:, 2] >= float(z_min))
            & (pts[:, 2] <= float(z_max))
            & np.isfinite(pts).all(axis=1)
        ]
        if hit.shape[0] == 0:
            return 0
        cols = np.floor((hit[:, 0] - float(ox)) / res).astype(np.int64)
        rows = np.floor((hit[:, 1] - float(oy)) / res).astype(np.int64)
        ok = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
        rows, cols = rows[ok], cols[ok]
        if rows.size == 0:
            return 0
        out = np.asarray(cells, dtype=np.int8).copy()
        before = out.copy()
        seeds_r, seeds_c = rows, cols
        inflate = max(0, int(round(float(inflate_m) / res)))
        for dy in range(-inflate, inflate + 1):
            for dx in range(-inflate, inflate + 1):
                if inflate > 0 and dx * dx + dy * dy > inflate * inflate:
                    continue
                rr = seeds_r + dy
                cc = seeds_c + dx
                inside = (rr >= 0) & (rr < height) & (cc >= 0) & (cc < width)
                if not np.any(inside):
                    continue
                out[rr[inside], cc[inside]] = 0
        changed = int(np.count_nonzero((before == 100) & (out == 0)))
        self.grid = (out, resolution, ox, oy, width, height)
        self.cached_grid = self.grid
        self.occupied = out == 100
        return changed

    def sample(
        self, simulator: Any = None, origin_xy: tuple[float, float] = (0.0, 0.0)
    ) -> tuple[np.ndarray, np.ndarray, float, float, float, int, int]:
        """Return cached cloud and occupancy grid (API matches ``autosim.scene.Map``)."""
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


def points_in_polygon(
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


def erode_boolean_mask(mask: np.ndarray, *, radius_cells: int) -> np.ndarray:
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


def dense_floorplan_cloud(
    rooms: Sequence[np.ndarray],
    *,
    free_mask: np.ndarray,
    origin_xy: tuple[float, float],
    resolution: float,
    z_min: float,
    z_max: float,
    floor_z: float = 0.0,
) -> np.ndarray:
    """Walls extruded through height + floor samples inside free cells.

    Floor points use ``floor_z`` (default 0) so the map sits on world ground.
    Wall columns still span ``[z_min, z_max]``.
    """
    step = max(float(resolution), 1e-3)
    z0 = min(float(z_min), float(z_max))
    z1 = max(float(z_min), float(z_max))
    n_z = max(2, int(math.ceil((z1 - z0) / step)) + 1)
    z_levels = np.linspace(z0, z1, n_z, dtype=np.float64)
    chunks: list[np.ndarray] = [wall_point_cloud(rooms, z_levels=z_levels, spacing=step)]

    # Floor fill (every other free cell keeps size reasonable).
    rows, cols = np.where(free_mask)
    if rows.size > 0:
        take = slice(None, None, 2)
        fx = origin_xy[0] + (cols[take].astype(np.float64) + 0.5) * step
        fy = origin_xy[1] + (rows[take].astype(np.float64) + 0.5) * step
        floor = np.column_stack(
            [fx, fy, np.full(fx.shape[0], float(floor_z), dtype=np.float64)]
        ).astype(np.float32)
        chunks.append(floor)
    return np.vstack([c for c in chunks if c.size > 0])


def wall_point_cloud(
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
