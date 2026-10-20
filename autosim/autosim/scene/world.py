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

"""Scenario environment: bounds, occupancy, and dynamic traffic.

Owns the planar play area and steps obstacles / pedestrians each control
cycle. No ROS — backends (Fake / Habitat / Isaac) only supply a grid and
mirror poses (e.g. USD characters / capsules).
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from autosim.scene.field import Field
from autosim.scene.traffic import Traffic


class World:
    """Environment manager for one ``habitat.scenario`` block."""

    def __init__(self, traffic: Traffic, grid: Any = None) -> None:
        """Bind traffic to an optional occupancy grid.

        Args:
            traffic: Dynamic obstacles and pedestrians.
            grid: Packed ``(cells, res, ox, oy, w, h)`` or ``None``.
        """
        self.traffic = traffic
        self.grid = grid
        self.field = traffic.field

    @classmethod
    def from_block(cls, block: Mapping[str, Any] | None, grid: Any = None) -> "World":
        """Build from ``habitat.scenario`` and an optional floorplan grid.

        When ``grid`` is set and the block omits extents, fill ``x_length`` /
        ``y_length`` / ``center_*`` / ``spawn_*`` from the occupancy map.
        """
        parent = cls.with_extents(dict(block or {}), grid)
        return cls(Traffic.from_block(parent, grid), grid)

    @staticmethod
    def with_extents(block: dict[str, Any], grid: Any) -> dict[str, Any]:
        """Copy map size from a packed grid when the YAML left it blank."""
        settings = dict(block)
        if grid is None:
            return settings
        cells, resolution, origin_x, origin_y, width, height = grid
        del cells
        if "x_length" not in settings:
            settings["x_length"] = float(width) * float(resolution)
        if "y_length" not in settings:
            settings["y_length"] = float(height) * float(resolution)
        if "center_x" not in settings:
            settings["center_x"] = float(origin_x) + 0.5 * settings["x_length"]
        if "center_y" not in settings:
            settings["center_y"] = float(origin_y) + 0.5 * settings["y_length"]
        if "spawn_x" not in settings:
            settings["spawn_x"] = float(settings["center_x"])
        if "spawn_y" not in settings:
            settings["spawn_y"] = float(settings["center_y"])
        return settings

    def step(
        self,
        dt: float,
        robot: tuple[float, float, float] | None = None,
    ) -> None:
        """Advance dynamic agents; pass robot pose for social reactions."""
        self.traffic.step(dt, robot=robot)

    def nearest_batch(
        self,
        origin: tuple[float, float, float] | np.ndarray,
        directions: np.ndarray,
        t_max: float,
    ) -> np.ndarray:
        """Closer dynamic hit per ray (shared origin). Misses are ``nan``."""
        return self.traffic.nearest_batch(origin, directions, t_max)

    def cloud(self, resolution: float = 0.15) -> np.ndarray:
        """Surface points of moving bodies for map fusion."""
        return self.traffic.cloud(resolution)

    @property
    def obstacle(self):
        """Dynamic cylinders."""
        return self.traffic.obstacle

    @property
    def pedestrian(self):
        """Dynamic walkers."""
        return self.traffic.pedestrian

    def rebind(self, grid: Any) -> None:
        """Replace the occupancy field (e.g. after floorplan warmup)."""
        self.grid = grid
        field = Field.from_grid(grid)
        self.field = field
        self.traffic.field = field
        if field is not None:
            self.traffic.obstacle.bind_field(field)
            self.traffic.pedestrian.bind_field(field)
