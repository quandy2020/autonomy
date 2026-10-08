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

"""Obstacles and pedestrians stepped with the plant, visible to lidar."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from autosim.scenario.cast import Cast
from autosim.scenario.field import Field
from autosim.scenario.obstacle.obstacle import Obstacle
from autosim.scenario.pedestrian.pedestrian import Pedestrian


class Traffic:
    """Dynamic bodies for one scenario block.

    Extents omitted on a child block are copied from the parent map. The
    occupancy grid, when the procedural map was built, keeps them out of walls.
    """

    GEOMETRY = ("x_length", "y_length", "spawn_radius")

    def __init__(
        self,
        obstacle: Obstacle,
        pedestrian: Pedestrian,
        field: Field | None = None,
    ) -> None:
        """Bind an optional occupancy field onto both populations.

        Args:
            obstacle: Cylinders. Disabled populations have no bodies.
            pedestrian: Walkers.
            field: Planar map. ``None`` keeps motion inside the rectangle only.
        """
        self.obstacle = obstacle
        self.pedestrian = pedestrian
        self.field = field
        if field is not None:
            obstacle.bind_field(field)
            pedestrian.bind_field(field)

    @classmethod
    def from_block(cls, block: Mapping[str, Any] | None, grid: Any = None) -> "Traffic":
        """Build both populations from ``habitat.scenario``.

        Args:
            block: Scenario mapping. Missing children stay disabled.
            grid: ``Scenario.grid`` tuple, or ``None`` outside a procedural map.
        """
        parent = dict(block or {})
        return cls(
            Obstacle(cls.inherit(parent, "obstacle")),
            Pedestrian(cls.inherit(parent, "pedestrian")),
            Field.from_grid(grid),
        )

    @classmethod
    def inherit(cls, parent: Mapping[str, Any], name: str) -> dict[str, Any]:
        """Child settings with parent map size filled in when omitted."""
        child = dict(parent.get(name) or {})
        for key in cls.GEOMETRY:
            if key not in child and key in parent:
                child[key] = parent[key]
        return child

    def step(self, dt: float) -> None:
        """Advance obstacles, then walkers that stay clear of those cylinders."""
        self.obstacle.step(dt)
        self.pedestrian.step(
            dt,
            bodies=self.obstacle.position,
            body_radius=float(self.obstacle.settings["radius"]),
        )

    def nearest(
        self,
        origin: tuple[float, float, float],
        direction: tuple[float, float, float],
        t_max: float,
    ) -> float | None:
        """Closer dynamic hit along one map-frame ray, or ``None``."""
        return Cast.nearest(origin, direction, self.obstacle, self.pedestrian, t_max)

    def nearest_batch(
        self,
        origin: tuple[float, float, float] | np.ndarray,
        directions: np.ndarray,
        t_max: float,
    ) -> np.ndarray:
        """Closer dynamic hit per ray (shared origin). Misses are ``nan``."""
        return Cast.nearest_batch(
            origin, directions, self.obstacle, self.pedestrian, t_max
        )

    def cloud(self, resolution: float = 0.15) -> np.ndarray:
        """Surface points of moving obstacles and pedestrians for ``/overall/map``.

        Args:
            resolution: Approximate spacing of surface samples (meters).

        Returns:
            ``(M, 3)`` float32 map-frame XYZ. Empty when both populations are idle.
        """
        parts = [
            self.obstacle.surface_points(resolution),
            self.pedestrian.surface_points(resolution),
        ]
        nonempty = [part for part in parts if part.shape[0] > 0]
        if not nonempty:
            return np.zeros((0, 3), dtype=np.float32)
        return np.concatenate(nonempty, axis=0)
