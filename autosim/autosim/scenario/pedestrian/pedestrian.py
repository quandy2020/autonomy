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

Each person tracks a free-space goal. A blocked step continues along the wall
instead of stopping, and a walker who stays put picks a new goal. Neighbors,
cylinders, and the robot disk all push them apart.
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
        "radius": 0.2,
        "height": 1.7,
        "speed": 1.2,
        "seed": 1,
        "x_length": 10.0,
        "y_length": 10.0,
        "spawn_radius": 1.0,
        "goal_tolerance": 0.4,
        "stuck_steps": 25,
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
        self.stuck = np.zeros((0,), dtype=np.int32)
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
        self.goal = self.sample_goals(count)
        self.stuck = np.zeros((count,), dtype=np.int32)
        delta = self.goal - self.position
        self.yaw = np.arctan2(delta[:, 1], delta[:, 0])

    def step(
        self,
        dt: float,
        bodies: np.ndarray | None = None,
        body_radius: float = 0.0,
    ) -> None:
        """Walk toward goals, skirt walls, and stay clear of other bodies.

        Args:
            dt: Time step in seconds.
            bodies: ``(M, 2)`` centers to avoid, usually obstacle cylinders.
            body_radius: Radius of every center in ``bodies``.
        """
        if dt < 0.0:
            raise ValueError("dt must be >= 0")
        if self.arena is None or self.position.shape[0] == 0:
            return
        velocity = self.limit_speed(self.desired_velocity() + self.separation())
        start = np.array(self.position, dtype=np.float64, copy=True)
        proposed = self.choose_step(start, velocity, float(dt))
        proposed = self.arena.leave_spawn(proposed)
        proposed = self.clear_bodies(proposed, bodies, float(body_radius))
        self.position = self.keep_free(start, proposed)
        self.face(start)
        self.note_stuck(start)
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

    def choose_step(self, start: np.ndarray, velocity: np.ndarray, dt: float) -> np.ndarray:
        """Step forward, or along the wall when that way is blocked."""
        best = start
        best_cost = np.full(start.shape[0], np.inf)
        for option in self.step_options(velocity):
            proposed = self.place_step(start, option, dt)
            cost = self.step_cost(start, proposed)
            take = cost < best_cost
            best = np.where(take[:, None], proposed, best)
            best_cost = np.minimum(best_cost, cost)
        return best

    def step_options(self, velocity: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Forward, then the two wall tangents of the same speed."""
        left = np.stack((-velocity[:, 1], velocity[:, 0]), axis=1)
        return velocity, left, -left

    def place_step(self, start: np.ndarray, velocity: np.ndarray, dt: float) -> np.ndarray:
        """Clamp one candidate and drop any part that enters an occupied cell."""
        proposed = self.arena.clamp(start + velocity * dt)
        if self.field is None:
            return proposed
        radius = float(self.settings["radius"])
        proposed = self.field.slide(start, proposed, radius)
        blocked = self.field.blocked(proposed, radius)
        proposed[blocked] = start[blocked]
        return proposed

    def step_cost(self, start: np.ndarray, proposed: np.ndarray) -> np.ndarray:
        """Distance left to the goal. A step that does not move sorts last."""
        distance = np.linalg.norm(self.goal - proposed, axis=1)
        stayed = np.linalg.norm(proposed - start, axis=1) <= 1e-6
        return np.where(stayed, distance + 1.0e3, distance)

    def clear_bodies(
        self,
        proposed: np.ndarray,
        bodies: np.ndarray | None,
        body_radius: float,
    ) -> np.ndarray:
        """Push centers out to ``radius + body_radius`` from each other body."""
        if bodies is None:
            return proposed
        centers = np.asarray(bodies, dtype=np.float64).reshape(-1, 2)
        gap = float(self.settings["radius"]) + float(body_radius)
        if centers.shape[0] == 0 or gap <= 0.0:
            return proposed
        placed = np.array(proposed, dtype=np.float64, copy=True)
        delta = placed[:, None, :] - centers[None, :, :]
        distance = np.linalg.norm(delta, axis=-1)
        overlap = np.clip(gap - distance, 0.0, None)
        direction = delta / np.maximum(distance, 1e-6)[..., None]
        direction[distance < 1e-6] = np.array([1.0, 0.0])
        placed += (direction * overlap[..., None]).sum(axis=1)
        return self.arena.clamp(placed)

    def keep_free(self, start: np.ndarray, proposed: np.ndarray) -> np.ndarray:
        """Reject a clearance step that lands back inside an occupied cell."""
        if self.field is None:
            return proposed
        placed = np.array(proposed, dtype=np.float64, copy=True)
        blocked = self.field.blocked(placed, float(self.settings["radius"]))
        placed[blocked] = start[blocked]
        return placed

    def face(self, start: np.ndarray) -> None:
        """Aim yaw along the step that was actually taken."""
        delta = self.position - start
        moving = np.linalg.norm(delta, axis=1) > 1e-6
        self.yaw[moving] = np.arctan2(delta[moving, 1], delta[moving, 0])

    def note_stuck(self, start: np.ndarray) -> None:
        """Count consecutive steps that do not move the walker."""
        moved = np.linalg.norm(self.position - start, axis=1) > 1e-3
        self.stuck[moved] = 0
        self.stuck[~moved] += 1

    def sample_goals(self, count: int) -> np.ndarray:
        """Goals inside the rectangle and, when a map exists, off the walls."""
        if self.arena is None or count == 0:
            return np.zeros((0, 2), dtype=np.float64)
        goals = self.arena.sample(self.rng, count)
        if self.field is None:
            return goals
        return self.field.relocate(
            self.arena, goals, float(self.settings["radius"]), self.rng
        )

    def refresh_goals(self) -> None:
        """Replace goals that were reached, or held while the walker did not move."""
        if self.arena is None:
            return
        reached = np.linalg.norm(self.goal - self.position, axis=1) <= float(
            self.settings["goal_tolerance"]
        )
        stalled = self.stuck >= int(self.settings["stuck_steps"])
        pick = reached | stalled
        if np.any(pick):
            self.goal[pick] = self.sample_goals(int(pick.sum()))
            self.stuck[pick] = 0

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
