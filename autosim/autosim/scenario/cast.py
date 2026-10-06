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
