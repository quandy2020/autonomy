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

"""Walking pedestrians inside the scenario bounds.

Each person tracks a planar goal at constant preferred speed, slides along
occupied cells, and is pushed away from neighbors closer than two radii.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from autosim.scenario.arena import Arena
from autosim.scenario.field import Field


class Pedestrian:
    """Goal-directed walkers with a short-range separation force."""

    DEFAULTS: dict[str, Any] = {
        "enabled": False,
        "count": 8,
        "radius": 0.3,
        "height": 1.7,
        "speed": 1.2,
        "seed": 1,
        "x_length": 10.0,
        "y_length": 10.0,
        "spawn_radius": 1.0,
        "goal_tolerance": 0.4,
    }

    def __init__(self, settings: Mapping[str, Any] | None = None) -> None:
        """Copy settings and spawn when ``enabled`` is true.

        Args:
            settings: ``habitat.scenario.pedestrian`` mapping.
        """
        merged = dict(self.DEFAULTS)
        merged.update(dict(settings or {}))
        self.settings = merged
        self.position = np.zeros((0, 2), dtype=np.float64)
        self.goal = np.zeros((0, 2), dtype=np.float64)
        self.yaw = np.zeros((0,), dtype=np.float64)
        self.arena: Arena | None = None
        self.field: Field | None = None
        self.rng = np.random.default_rng(int(self.settings["seed"]))
        if self.settings.get("enabled", False):
            self.spawn()

    def spawn(self) -> None:
        """Place walkers outside the robot disk and assign a first goal."""
        count = int(self.settings["count"])
        if count < 0:
            raise ValueError("scenario.pedestrian.count must be >= 0")
        if float(self.settings["height"]) <= 0.0:
            raise ValueError("scenario.pedestrian.height must be > 0")
        self.arena = self.make_arena()
        self.position = self.arena.sample(self.rng, count)
        self.goal = self.arena.sample(self.rng, count)
        delta = self.goal - self.position
        self.yaw = np.arctan2(delta[:, 1], delta[:, 0])

    def step(self, dt: float) -> None:
        """Walk toward goals, separate from neighbors, and stay inside the map.

        Args:
            dt: Time step in seconds.
        """
        if dt < 0.0:
            raise ValueError("dt must be >= 0")
        if self.arena is None or self.position.shape[0] == 0:
            return
        velocity = self.limit_speed(self.desired_velocity() + self.separation())
        start = self.position
        proposed = self.arena.clamp(start + velocity * float(dt))
        if self.field is not None:
            proposed = self.field.slide(start, proposed, float(self.settings["radius"]))
        self.position = proposed
        moving = np.linalg.norm(velocity, axis=1) > 1e-6
        self.yaw[moving] = np.arctan2(velocity[moving, 1], velocity[moving, 0])
        self.refresh_goals()

    def poses(self) -> np.ndarray:
        """Planar poses ``(x, y, yaw)``.

        Returns:
            ``(N, 3)`` float64.
        """
        if self.position.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float64)
        return np.column_stack((self.position, self.yaw))

    def desired_velocity(self) -> np.ndarray:
        """Preferred velocity of length ``speed`` toward each goal."""
        delta = self.goal - self.position
        distance = np.linalg.norm(delta, axis=1, keepdims=True)
        direction = np.divide(delta, np.maximum(distance, 1e-6))
        return direction * float(self.settings["speed"])

    def separation(self) -> np.ndarray:
        """Repulsion from other pedestrians inside two radii."""
        delta = self.position[:, None, :] - self.position[None, :, :]
        distance = np.linalg.norm(delta, axis=-1)
        np.fill_diagonal(distance, np.inf)
        overlap = np.clip(2.0 * float(self.settings["radius"]) - distance, 0.0, None)
        direction = delta / np.maximum(distance, 1e-6)[..., None]
        return (direction * overlap[..., None]).sum(axis=1)

    def limit_speed(self, velocity: np.ndarray) -> np.ndarray:
        """Cap each walker at ``speed``."""
        speed = float(self.settings["speed"])
        norm = np.linalg.norm(velocity, axis=1, keepdims=True)
        scale = np.ones_like(norm)
        fast = norm[:, 0] > speed
        scale[fast] = speed / np.maximum(norm[fast], 1e-6)
        return velocity * scale

    def refresh_goals(self) -> None:
        """Assign a new goal to anyone within ``goal_tolerance`` of the current one."""
        if self.arena is None:
            return
        reached = np.linalg.norm(self.goal - self.position, axis=1) <= float(
            self.settings["goal_tolerance"]
        )
        if np.any(reached):
            self.goal[reached] = self.arena.sample(self.rng, int(reached.sum()))

    def bind_field(self, field: Field) -> None:
        """Slide along this grid, and move anyone who spawned on a wall."""
        self.field = field
        if self.arena is None or self.position.shape[0] == 0:
            return
        rng = np.random.default_rng(int(self.settings["seed"]) + 17)
        radius = float(self.settings["radius"])
        self.position = field.relocate(self.arena, self.position, radius, rng)
        self.goal = field.relocate(self.arena, self.goal, radius, rng)

    def make_arena(self) -> Arena:
        """Rectangle these pedestrians are allowed to walk in."""
        radius = float(self.settings["radius"])
        if radius <= 0.0:
            raise ValueError("scenario.pedestrian.radius must be > 0")
        return Arena(
            float(self.settings["x_length"]),
            float(self.settings["y_length"]),
            radius,
            float(self.settings["spawn_radius"]),
            "scenario map is smaller than a pedestrian",
        )
