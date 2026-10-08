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

"""Ray distances to dynamic cylinders and vertical capsules.

Obstacles are finite vertical cylinders. Pedestrians are capsules. The static
Habitat mesh is left unchanged; the caller keeps the nearer hit.
"""

from __future__ import annotations

import numpy as np

from autosim.scenario.obstacle.obstacle import Obstacle
from autosim.scenario.pedestrian.pedestrian import Pedestrian


class Cast:
    """Nearest positive hit of one map-frame ray against the moving bodies."""

    @staticmethod
    def nearest(
        origin: tuple[float, float, float],
        direction: tuple[float, float, float],
        obstacle: Obstacle,
        pedestrian: Pedestrian,
        t_max: float,
    ) -> float | None:
        """Distance along the ray, or ``None`` when every body is missed.

        Args:
            origin: Ray start in the map frame (z up).
            direction: Ray direction; it is normalized internally.
            obstacle: Cylinders. Empty when disabled.
            pedestrian: Capsules. Empty when disabled.
            t_max: Ignore hits farther than this (meters).
        """
        vector = np.asarray(direction, dtype=np.float64).reshape(3)
        norm = float(np.linalg.norm(vector))
        if norm < 1e-12 or t_max <= 0.0:
            return None
        unit = vector / norm
        start = np.asarray(origin, dtype=np.float64).reshape(3)
        hits = [
            Cast.cylinders(start, unit, obstacle, t_max),
            Cast.capsules(start, unit, pedestrian, t_max),
        ]
        finite = [hit for hit in hits if hit is not None]
        return min(finite) if finite else None

    @staticmethod
    def cylinders(
        origin: np.ndarray, direction: np.ndarray, obstacle: Obstacle, t_max: float
    ) -> float | None:
        """Nearest hit on a vertical cylinder, including flat caps."""
        centers = obstacle.position
        if centers.shape[0] == 0:
            return None
        height = float(obstacle.settings["height"])
        radius = float(obstacle.settings["radius"])
        return Cast.tubes(
            origin, direction, centers, radius, 0.0, height, t_max, caps=True
        )

    @staticmethod
    def capsules(
        origin: np.ndarray, direction: np.ndarray, pedestrian: Pedestrian, t_max: float
    ) -> float | None:
        """Nearest hit on a vertical capsule standing on the ground."""
        centers = pedestrian.position
        if centers.shape[0] == 0:
            return None
        radius = float(pedestrian.settings["radius"])
        height = float(pedestrian.settings["height"])
        if height <= 2.0 * radius:
            mid = np.column_stack((centers, np.full(centers.shape[0], 0.5 * height)))
            return Cast.spheres(origin, direction, mid, min(radius, 0.5 * height), t_max, None)
        return Cast.capsule_body(origin, direction, centers, radius, height, t_max)

    @staticmethod
    def capsule_body(
        origin: np.ndarray,
        direction: np.ndarray,
        centers: np.ndarray,
        radius: float,
        height: float,
        t_max: float,
    ) -> float | None:
        """Tube between the end-sphere centers, plus the outer hemispheres."""
        z0 = radius
        z1 = height - radius
        body = Cast.tubes(origin, direction, centers, radius, z0, z1, t_max, caps=False)
        bottom = np.column_stack((centers, np.full(centers.shape[0], z0)))
        top = np.column_stack((centers, np.full(centers.shape[0], z1)))
        low = Cast.spheres(origin, direction, bottom, radius, t_max, "below")
        high = Cast.spheres(origin, direction, top, radius, t_max, "above")
        finite = [hit for hit in (body, low, high) if hit is not None]
        return min(finite) if finite else None

    @staticmethod
    def tubes(
        origin: np.ndarray,
        direction: np.ndarray,
        centers: np.ndarray,
        radius: float,
        z0: float,
        z1: float,
        t_max: float,
        caps: bool,
    ) -> float | None:
        """Vertical tubes. ``caps`` adds the flat disks at ``z0`` and ``z1``."""
        ox, oy, oz = (float(origin[0]), float(origin[1]), float(origin[2]))
        dx, dy, dz = (float(direction[0]), float(direction[1]), float(direction[2]))
        radial = dx * dx + dy * dy
        best: float | None = None
        if radial > 1e-12:
            fx = ox - centers[:, 0]
            fy = oy - centers[:, 1]
            b = 2.0 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - radius * radius
            disc = b * b - 4.0 * radial * c
            root = np.sqrt(np.maximum(disc, 0.0))
            for sign in (-1.0, 1.0):
                distance = (-b + sign * root) / (2.0 * radial)
                height = oz + distance * dz
                ok = (disc >= 0.0) & (distance > 1e-6) & (distance <= t_max)
                ok &= (height >= z0) & (height <= z1)
                best = Cast.closer(best, distance, ok)
        if caps and abs(dz) > 1e-12:
            best = Cast.disks(origin, direction, centers, radius, z0, z1, t_max, best)
        return best

    @staticmethod
    def disks(
        origin: np.ndarray,
        direction: np.ndarray,
        centers: np.ndarray,
        radius: float,
        z0: float,
        z1: float,
        t_max: float,
        best: float | None,
    ) -> float | None:
        """Flat end caps of a vertical cylinder."""
        ox, oy, oz = (float(origin[0]), float(origin[1]), float(origin[2]))
        dx, dy, dz = (float(direction[0]), float(direction[1]), float(direction[2]))
        for plane in (z0, z1):
            distance = (plane - oz) / dz
            if distance <= 1e-6 or distance > t_max:
                continue
            hx = ox + distance * dx - centers[:, 0]
            hy = oy + distance * dy - centers[:, 1]
            inside = hx * hx + hy * hy <= radius * radius
            best = Cast.closer(best, np.full(centers.shape[0], distance), inside)
        return best

    @staticmethod
    def spheres(
        origin: np.ndarray,
        direction: np.ndarray,
        centers: np.ndarray,
        radius: float,
        t_max: float,
        side: str | None,
    ) -> float | None:
        """Sphere hits. ``side`` keeps the outer hemisphere of a capsule cap."""
        offset = centers - origin.reshape(1, 3)
        b = -2.0 * (offset @ direction.reshape(3))
        c = np.sum(offset * offset, axis=1) - radius * radius
        disc = b * b - 4.0 * c
        root = np.sqrt(np.maximum(disc, 0.0))
        best: float | None = None
        for sign in (-1.0, 1.0):
            distance = (-b + sign * root) / 2.0
            ok = (disc >= 0.0) & (distance > 1e-6) & (distance <= t_max)
            if side == "below":
                ok &= origin[2] + distance * direction[2] <= centers[:, 2] + 1e-8
            elif side == "above":
                ok &= origin[2] + distance * direction[2] >= centers[:, 2] - 1e-8
            best = Cast.closer(best, distance, ok)
        return best

    @staticmethod
    def closer(best: float | None, distance: np.ndarray, ok: np.ndarray) -> float | None:
        """Min of ``best`` and the accepted samples of ``distance``."""
        if not np.any(ok):
            return best
        found = float(np.min(np.asarray(distance)[ok]))
        if best is None or found < best:
            return found
        return best

    @staticmethod
    def nearest_batch(
        origin: tuple[float, float, float] | np.ndarray,
        directions: np.ndarray,
        obstacle: Obstacle,
        pedestrian: Pedestrian,
        t_max: float,
    ) -> np.ndarray:
        """Nearest dynamic hit per ray (shared origin). Misses are ``nan``.

        Args:
            origin: Shared ray start in the map frame (z up).
            directions: Unit directions ``(N, 3)``.
            obstacle: Cylinders. Empty when disabled.
            pedestrian: Capsules. Empty when disabled.
            t_max: Ignore hits farther than this (meters).
        """
        dirs = np.asarray(directions, dtype=np.float64).reshape(-1, 3)
        hits = np.full(dirs.shape[0], np.nan, dtype=np.float64)
        if dirs.shape[0] == 0 or t_max <= 0.0:
            return hits
        start = np.asarray(origin, dtype=np.float64).reshape(3)
        Cast._fold_batch(hits, Cast._cylinders_batch(start, dirs, obstacle, t_max))
        Cast._fold_batch(hits, Cast._capsules_batch(start, dirs, pedestrian, t_max))
        return hits

    @staticmethod
    def _fold_batch(hits: np.ndarray, candidate: np.ndarray | None) -> None:
        if candidate is None:
            return
        better = np.isfinite(candidate) & (
            np.isnan(hits) | (candidate < hits)
        )
        hits[better] = candidate[better]

    @staticmethod
    def _cylinders_batch(
        origin: np.ndarray, directions: np.ndarray, obstacle: Obstacle, t_max: float
    ) -> np.ndarray | None:
        centers = obstacle.position
        if centers.shape[0] == 0:
            return None
        height = float(obstacle.settings["height"])
        radius = float(obstacle.settings["radius"])
        return Cast._tubes_batch(
            origin, directions, centers, radius, 0.0, height, t_max, caps=True
        )

    @staticmethod
    def _capsules_batch(
        origin: np.ndarray, directions: np.ndarray, pedestrian: Pedestrian, t_max: float
    ) -> np.ndarray | None:
        centers = pedestrian.position
        if centers.shape[0] == 0:
            return None
        radius = float(pedestrian.settings["radius"])
        height = float(pedestrian.settings["height"])
        if height <= 2.0 * radius:
            mid = np.column_stack((centers, np.full(centers.shape[0], 0.5 * height)))
            return Cast._spheres_batch(
                origin, directions, mid, min(radius, 0.5 * height), t_max, None
            )
        z0 = radius
        z1 = height - radius
        body = Cast._tubes_batch(
            origin, directions, centers, radius, z0, z1, t_max, caps=False
        )
        bottom = np.column_stack((centers, np.full(centers.shape[0], z0)))
        top = np.column_stack((centers, np.full(centers.shape[0], z1)))
        low = Cast._spheres_batch(origin, directions, bottom, radius, t_max, "below")
        high = Cast._spheres_batch(origin, directions, top, radius, t_max, "above")
        hits = np.full(directions.shape[0], np.nan, dtype=np.float64)
        for part in (body, low, high):
            Cast._fold_batch(hits, part)
        return hits

    @staticmethod
    def _tubes_batch(
        origin: np.ndarray,
        directions: np.ndarray,
        centers: np.ndarray,
        radius: float,
        z0: float,
        z1: float,
        t_max: float,
        caps: bool,
    ) -> np.ndarray:
        """Vertical tubes for many rays × many cylinders → ``(N,)``."""
        ox, oy, oz = (float(origin[0]), float(origin[1]), float(origin[2]))
        dx = directions[:, 0]
        dy = directions[:, 1]
        dz = directions[:, 2]
        radial = dx * dx + dy * dy
        best = np.full(directions.shape[0], np.nan, dtype=np.float64)
        # (N, M)
        fx = ox - centers[None, :, 0]
        fy = oy - centers[None, :, 1]
        active = radial > 1e-12
        if np.any(active):
            b = 2.0 * (fx * dx[:, None] + fy * dy[:, None])
            c = fx * fx + fy * fy - radius * radius
            disc = b * b - 4.0 * radial[:, None] * c
            root = np.sqrt(np.maximum(disc, 0.0))
            for sign in (-1.0, 1.0):
                distance = (-b + sign * root) / (2.0 * radial[:, None])
                height = oz + distance * dz[:, None]
                ok = (
                    active[:, None]
                    & (disc >= 0.0)
                    & (distance > 1e-6)
                    & (distance <= t_max)
                    & (height >= z0)
                    & (height <= z1)
                )
                Cast._fold_rows(best, distance, ok)
        if caps and np.any(np.abs(dz) > 1e-12):
            for plane in (z0, z1):
                distance = np.full(directions.shape[0], np.nan, dtype=np.float64)
                usable = np.abs(dz) > 1e-12
                distance[usable] = (plane - oz) / dz[usable]
                hx = ox + distance[:, None] * dx[:, None] - centers[None, :, 0]
                hy = oy + distance[:, None] * dy[:, None] - centers[None, :, 1]
                inside = (
                    usable[:, None]
                    & np.isfinite(distance)[:, None]
                    & (distance[:, None] > 1e-6)
                    & (distance[:, None] <= t_max)
                    & (hx * hx + hy * hy <= radius * radius)
                )
                Cast._fold_rows(best, np.broadcast_to(distance[:, None], inside.shape), inside)
        return best

    @staticmethod
    def _spheres_batch(
        origin: np.ndarray,
        directions: np.ndarray,
        centers: np.ndarray,
        radius: float,
        t_max: float,
        side: str | None,
    ) -> np.ndarray:
        offset = centers[None, :, :] - origin.reshape(1, 1, 3)
        b = -2.0 * np.sum(offset * directions[:, None, :], axis=2)
        c = np.sum(offset * offset, axis=2) - radius * radius
        disc = b * b - 4.0 * c
        root = np.sqrt(np.maximum(disc, 0.0))
        best = np.full(directions.shape[0], np.nan, dtype=np.float64)
        for sign in (-1.0, 1.0):
            distance = (-b + sign * root) / 2.0
            ok = (disc >= 0.0) & (distance > 1e-6) & (distance <= t_max)
            if side == "below":
                z = origin[2] + distance * directions[:, 2:3]
                ok &= z <= centers[None, :, 2] + 1e-8
            elif side == "above":
                z = origin[2] + distance * directions[:, 2:3]
                ok &= z >= centers[None, :, 2] - 1e-8
            Cast._fold_rows(best, distance, ok)
        return best

    @staticmethod
    def _fold_rows(best: np.ndarray, distance: np.ndarray, ok: np.ndarray) -> None:
        """Per-ray min over cylinder axis into ``best`` ``(N,)``."""
        masked = np.where(ok, distance, np.inf)
        row = np.min(masked, axis=1)
        finite = np.isfinite(row)
        better = finite & (np.isnan(best) | (row < best))
        best[better] = row[better]
