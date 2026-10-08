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
    def cast_hits(
        occupied: np.ndarray,
        origin: np.ndarray,
        resolution: float,
        ray_origin: np.ndarray,
        ray_dirs: np.ndarray,
        t_min: float,
        t_max: float,
        *,
        ground_z: float = 0.0,
    ) -> np.ndarray:
        """First hit distance along each map-frame ray (voxels + ground plane).

        Uses Amanatides–Woo DDA so thin shells (e.g. ``posts``) cannot be
        skipped. Occluded surfaces behind the first hit are never returned.
        Also intersects the infinite ground plane at ``ground_z``. Misses are
        ``nan``.

        Args:
            occupied: Boolean volume ``(nx, ny, nz)``.
            origin: Map-frame minimum corner of voxel ``(0,0,0)``.
            resolution: Voxel edge length (meters).
            ray_origin: Shared ray start ``(3,)`` in the map frame.
            ray_dirs: Unit directions ``(N, 3)`` in the map frame.
            t_min: Ignore hits closer than this (meters).
            t_max: Ignore hits farther than this (meters).
            ground_z: Ground plane height (map z-up).

        Returns:
            ``(N,)`` float64 distances; ``nan`` when nothing is hit in range.
        """
        dirs = np.asarray(ray_dirs, dtype=np.float64).reshape(-1, 3)
        count = dirs.shape[0]
        hits = np.full(count, np.nan, dtype=np.float64)
        if count == 0 or resolution <= 0.0 or t_max <= t_min:
            return hits

        vol = np.asarray(occupied, dtype=bool)
        low = np.asarray(origin, dtype=np.float64).reshape(3)
        start = np.asarray(ray_origin, dtype=np.float64).reshape(3)
        nx, ny, nz = vol.shape
        res = float(resolution)

        # Ground plane (z = ground_z), closed form.
        dz = dirs[:, 2]
        toward = dz < -1e-12
        if np.any(toward):
            ground_t = (float(ground_z) - start[2]) / dz
            ok = toward & (ground_t >= t_min) & (ground_t <= t_max)
            hits[ok] = ground_t[ok]

        for index in range(count):
            voxel_t = Volume._dda_hit(
                vol, low, res, start, dirs[index], t_min, t_max, nx, ny, nz
            )
            if voxel_t is None:
                continue
            current = hits[index]
            if np.isnan(current) or voxel_t < current:
                hits[index] = voxel_t
        return hits

    @staticmethod
    def _dda_hit(
        vol: np.ndarray,
        low: np.ndarray,
        res: float,
        start: np.ndarray,
        direction: np.ndarray,
        t_min: float,
        t_max: float,
        nx: int,
        ny: int,
        nz: int,
    ) -> float | None:
        """First occupied voxel distance along one unit ray (Amanatides–Woo)."""
        dx, dy, dz = (float(direction[0]), float(direction[1]), float(direction[2]))
        # Enter the grid at t_min so near-plane clipping is honored.
        ox = float(start[0]) + t_min * dx
        oy = float(start[1]) + t_min * dy
        oz = float(start[2]) + t_min * dz
        t = float(t_min)

        inv = [
            (1.0 / dx) if abs(dx) > 1e-12 else float("inf"),
            (1.0 / dy) if abs(dy) > 1e-12 else float("inf"),
            (1.0 / dz) if abs(dz) > 1e-12 else float("inf"),
        ]
        step = [1 if d > 0.0 else -1 for d in (dx, dy, dz)]

        # If still outside the AABB, advance to the first intersection.
        high = low + np.array([nx, ny, nz], dtype=np.float64) * res
        if not (
            low[0] <= ox < high[0]
            and low[1] <= oy < high[1]
            and low[2] <= oz < high[2]
        ):
            t_enter = Volume._aabb_enter(ox, oy, oz, dx, dy, dz, low, high)
            if t_enter is None or t + t_enter > t_max:
                return None
            t += t_enter
            ox = float(start[0]) + t * dx
            oy = float(start[1]) + t * dy
            oz = float(start[2]) + t * dz

        ix = int(np.floor((ox - low[0]) / res))
        iy = int(np.floor((oy - low[1]) / res))
        iz = int(np.floor((oz - low[2]) / res))
        ix = int(np.clip(ix, 0, nx - 1))
        iy = int(np.clip(iy, 0, ny - 1))
        iz = int(np.clip(iz, 0, nz - 1))

        sx = float(start[0])
        sy = float(start[1])
        sz = float(start[2])
        if step[0] > 0:
            t_max_x = ((ix + 1) * res + low[0] - sx) * inv[0]
        elif step[0] < 0 and np.isfinite(inv[0]):
            t_max_x = (ix * res + low[0] - sx) * inv[0]
        else:
            t_max_x = float("inf")
        if step[1] > 0:
            t_max_y = ((iy + 1) * res + low[1] - sy) * inv[1]
        elif step[1] < 0 and np.isfinite(inv[1]):
            t_max_y = (iy * res + low[1] - sy) * inv[1]
        else:
            t_max_y = float("inf")
        if step[2] > 0:
            t_max_z = ((iz + 1) * res + low[2] - sz) * inv[2]
        elif step[2] < 0 and np.isfinite(inv[2]):
            t_max_z = (iz * res + low[2] - sz) * inv[2]
        else:
            t_max_z = float("inf")

        t_delta_x = res * abs(inv[0]) if np.isfinite(inv[0]) else float("inf")
        t_delta_y = res * abs(inv[1]) if np.isfinite(inv[1]) else float("inf")
        t_delta_z = res * abs(inv[2]) if np.isfinite(inv[2]) else float("inf")

        max_steps = (nx + ny + nz) * 3 + 2
        for _ in range(max_steps):
            if t > t_max + 1e-9:
                return None
            if vol[ix, iy, iz]:
                return float(max(t, t_min))
            if t_max_x < t_max_y:
                if t_max_x < t_max_z:
                    ix += step[0]
                    t = t_max_x
                    t_max_x += t_delta_x
                else:
                    iz += step[2]
                    t = t_max_z
                    t_max_z += t_delta_z
            else:
                if t_max_y < t_max_z:
                    iy += step[1]
                    t = t_max_y
                    t_max_y += t_delta_y
                else:
                    iz += step[2]
                    t = t_max_z
                    t_max_z += t_delta_z
            if ix < 0 or iy < 0 or iz < 0 or ix >= nx or iy >= ny or iz >= nz:
                return None
        return None
    @staticmethod
    def _aabb_enter(
        ox: float,
        oy: float,
        oz: float,
        dx: float,
        dy: float,
        dz: float,
        low: np.ndarray,
        high: np.ndarray,
    ) -> float | None:
        """Ray–AABB entry distance from ``(ox,oy,oz)``, or ``None`` if missed."""
        t0 = -1e30
        t1 = 1e30
        for origin_i, d, lo, hi in (
            (ox, dx, float(low[0]), float(high[0])),
            (oy, dy, float(low[1]), float(high[1])),
            (oz, dz, float(low[2]), float(high[2])),
        ):
            if abs(d) < 1e-12:
                if origin_i < lo or origin_i > hi:
                    return None
                continue
            inv = 1.0 / d
            a = (lo - origin_i) * inv
            b = (hi - origin_i) * inv
            if a > b:
                a, b = b, a
            t0 = max(t0, a)
            t1 = min(t1, b)
            if t0 > t1:
                return None
        if t1 < 0.0:
            return None
        return float(max(t0, 0.0))

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
