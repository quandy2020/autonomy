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

"""Planar rectangle shared by moving obstacles and pedestrians."""

from __future__ import annotations

import numpy as np


class Arena:
    """Axis-aligned map rectangle inset by an agent radius, plus a spawn disk."""

    def __init__(
        self,
        x_length: float,
        y_length: float,
        radius: float,
        spawn_radius: float,
        too_small: str,
    ) -> None:
        """Store the rectangle.

        Args:
            x_length: Full map width in meters.
            y_length: Full map height in meters.
            radius: Agent radius; the walkable area is inset by this amount.
            spawn_radius: Robot disk that agents are pushed out of.
            too_small: Error text when the inset rectangle is empty.

        Raises:
            ValueError: ``too_small`` when an agent cannot fit.
        """
        self.x_length = float(x_length)
        self.y_length = float(y_length)
        self.radius = float(radius)
        self.spawn_radius = float(spawn_radius)
        self.limit = 0.5 * np.array([self.x_length, self.y_length]) - self.radius
        if np.any(self.limit <= 0.0):
            raise ValueError(too_small)

    def sample(self, rng: np.random.Generator, count: int) -> np.ndarray:
        """Uniform points inside the inset rectangle, outside the spawn disk.

        Returns:
            ``(count, 2)`` positions.
        """
        if count == 0:
            return np.zeros((0, 2), dtype=np.float64)
        position = rng.uniform(-self.limit, self.limit, size=(count, 2))
        return self.leave_spawn(position)

    def leave_spawn(self, position: np.ndarray) -> np.ndarray:
        """Push samples inside the robot disk out to its rim."""
        margin = self.spawn_radius + self.radius
        updated = np.array(position, dtype=np.float64, copy=True)
        if margin <= 0.0 or updated.shape[0] == 0:
            return updated
        distance = np.linalg.norm(updated, axis=1)
        close = distance < margin
        if not np.any(close):
            return updated
        direction = updated[close]
        norm = np.linalg.norm(direction, axis=1, keepdims=True)
        direction = np.divide(direction, np.maximum(norm, 1e-6))
        direction[norm[:, 0] < 1e-6] = np.array([1.0, 0.0])
        updated[close] = direction * margin
        return updated

    def reflect(self, position: np.ndarray, velocity: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Clamp to the inset rectangle and flip the outward velocity component."""
        placed = np.array(position, dtype=np.float64, copy=True)
        moving = np.array(velocity, dtype=np.float64, copy=True)
        for axis in (0, 1):
            over = placed[:, axis] > self.limit[axis]
            under = placed[:, axis] < -self.limit[axis]
            placed[over, axis] = self.limit[axis]
            placed[under, axis] = -self.limit[axis]
            moving[over | under, axis] *= -1.0
        return placed, moving

    def clamp(self, position: np.ndarray) -> np.ndarray:
        """Clamp XY into the inset rectangle."""
        placed = np.array(position, dtype=np.float64, copy=True)
        placed[:, 0] = np.clip(placed[:, 0], -self.limit[0], self.limit[0])
        placed[:, 1] = np.clip(placed[:, 1], -self.limit[1], self.limit[1])
        return placed
