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

"""Moving cylindrical obstacles inside the scenario bounds.

Obstacles bounce off the map rectangle and off occupied cells. They are not
baked into the static Habitat mesh; call :meth:`step` each control cycle.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from autosim.scenario.arena import Arena
from autosim.scenario.field import Field


class Obstacle:
    """Kinematic cylinders with constant speed and wall reflections."""

    DEFAULTS: dict[str, Any] = {
        "enabled": False,
        "count": 5,
        "radius": 0.4,
        "height": 1.0,
        "speed_min": 0.2,
        "speed_max": 0.8,
        "seed": 1,
        "x_length": 10.0,
        "y_length": 10.0,
        "spawn_radius": 0.8,
    }

    def __init__(self, settings: Mapping[str, Any] | None = None) -> None:
        """Copy settings and spawn when ``enabled`` is true.

        Args:
            settings: ``habitat.scenario.obstacle`` mapping.
        """
        merged = dict(self.DEFAULTS)
        merged.update(dict(settings or {}))
        self.settings = merged
        self.position = np.zeros((0, 2), dtype=np.float64)
        self.velocity = np.zeros((0, 2), dtype=np.float64)
        self.arena: Arena | None = None
        self.field: Field | None = None
        if self.settings.get("enabled", False):
            self.spawn()

    def spawn(self) -> None:
        """Place ``count`` cylinders outside the robot disk with random headings."""
        count = int(self.settings["count"])
        if count < 0:
            raise ValueError("scenario.obstacle.count must be >= 0")
        self.arena = self.make_arena()
        rng = np.random.default_rng(int(self.settings["seed"]))
        self.position = self.arena.sample(rng, count)
        low = float(self.settings["speed_min"])
        high = float(self.settings["speed_max"])
        if low > high:
            raise ValueError("scenario.obstacle.speed_min must be <= speed_max")
        if float(self.settings["height"]) <= 0.0:
            raise ValueError("scenario.obstacle.height must be > 0")
        heading = rng.uniform(0.0, 2.0 * np.pi, size=count)
        speed = rng.uniform(low, high, size=count)
        self.velocity = np.stack((np.cos(heading), np.sin(heading)), axis=1) * speed[:, None]

    def step(self, dt: float) -> None:
        """Advance one step, then bounce off the rectangle and any occupied cell.

        Args:
            dt: Time step in seconds.
        """
        if dt < 0.0:
            raise ValueError("dt must be >= 0")
        if self.arena is None or self.position.shape[0] == 0:
            return
        start = self.position
        proposed = start + self.velocity * float(dt)
        proposed, self.velocity = self.arena.reflect(proposed, self.velocity)
        if self.field is not None:
            proposed, self.velocity = self.field.bounce(
                start, proposed, self.velocity, float(self.settings["radius"])
            )
        self.position = proposed

    def points(self) -> np.ndarray:
        """Cylinder centers in the map frame, z at half the obstacle height.

        Returns:
            ``(N, 3)`` float64. Empty when disabled or ``count`` is 0.
        """
        if self.position.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float64)
        height = np.full((self.position.shape[0], 1), 0.5 * float(self.settings["height"]))
        return np.concatenate((self.position, height), axis=1)

    def bind_field(self, field: Field) -> None:
        """Keep later steps out of occupied cells, and resample anyone already inside."""
        self.field = field
        if self.arena is None or self.position.shape[0] == 0:
            return
        rng = np.random.default_rng(int(self.settings["seed"]) + 17)
        self.position = field.relocate(
            self.arena, self.position, float(self.settings["radius"]), rng
        )

    def make_arena(self) -> Arena:
        """Rectangle this obstacle set is allowed to move in."""
        radius = float(self.settings["radius"])
        if radius <= 0.0:
            raise ValueError("scenario.obstacle.radius must be > 0")
        return Arena(
            float(self.settings["x_length"]),
            float(self.settings["y_length"]),
            radius,
            float(self.settings["spawn_radius"]),
            "scenario map is smaller than an obstacle",
        )
