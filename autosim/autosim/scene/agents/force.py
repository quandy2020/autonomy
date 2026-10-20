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

"""Social force model for planar pedestrians (pure NumPy)."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np


class Force:
    """Goal attraction, social repulsion, and body / wall repulsion."""

    DEFAULTS: dict[str, float] = {
        "tau": 0.5,
        "A": 2.1,
        "B": 0.3,
        "A_obs": 10.0,
        "B_obs": 0.2,
        "goal_force_factor": 1.0,
        "social_force_factor": 1.0,
        "obstacle_force_factor": 1.0,
    }

    def __init__(self, settings: Mapping[str, Any] | None = None) -> None:
        """Copy SFM gains from a pedestrian / agent block.

        Args:
            settings: Optional keys matching :attr:`DEFAULTS`.
        """
        merged = dict(self.DEFAULTS)
        if settings:
            for key in self.DEFAULTS:
                if key in settings:
                    merged[key] = float(settings[key])
        self.settings = merged

    def acceleration(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        preferred: np.ndarray,
        radius: float,
        bodies: np.ndarray | None = None,
        body_radius: float = 0.0,
        *,
        goal_force_factor: np.ndarray | float | None = None,
        social_force_factor: np.ndarray | float | None = None,
        obstacle_force_factor: np.ndarray | float | None = None,
    ) -> np.ndarray:
        """Net planar acceleration from desired, social, and body forces.

        Optional per-agent gain arrays (HuNav YAML) broadcast as ``(N, 1)``.
        """
        gain = self.settings
        n = int(np.asarray(position).reshape(-1, 2).shape[0])

        def _gain(value: np.ndarray | float | None, key: str) -> np.ndarray | float:
            if value is None:
                return float(gain[key])
            arr = np.asarray(value, dtype=np.float64).reshape(-1)
            if arr.size == 1:
                return float(arr[0])
            if arr.size != n:
                return float(gain[key])
            return arr.reshape(n, 1)

        goal_g = _gain(goal_force_factor, "goal_force_factor")
        social_g = _gain(social_force_factor, "social_force_factor")
        obstacle_g = _gain(obstacle_force_factor, "obstacle_force_factor")
        accel = goal_g * self.desired(velocity, preferred)
        accel = accel + social_g * self.social(position, radius)
        if bodies is not None and np.asarray(bodies).size:
            accel = accel + obstacle_g * self.bodies(
                position, radius, bodies, body_radius
            )
        return accel

    def desired(self, velocity: np.ndarray, preferred: np.ndarray) -> np.ndarray:
        """Relaxation toward the preferred velocity: ``(v_pref - v) / tau``."""
        tau = max(float(self.settings["tau"]), 1e-3)
        return (np.asarray(preferred, dtype=np.float64) - np.asarray(velocity)) / tau

    def social(self, position: np.ndarray, radius: float) -> np.ndarray:
        """Exponential pairwise repulsion between pedestrians."""
        points = np.asarray(position, dtype=np.float64).reshape(-1, 2)
        if points.shape[0] < 2:
            return np.zeros_like(points)
        delta = points[:, None, :] - points[None, :, :]
        distance = np.linalg.norm(delta, axis=-1)
        np.fill_diagonal(distance, np.inf)
        strength = float(self.settings["A"]) * np.exp(
            (2.0 * float(radius) - distance) / max(float(self.settings["B"]), 1e-3)
        )
        direction = delta / np.maximum(distance, 1e-6)[..., None]
        return (direction * strength[..., None]).sum(axis=1)

    def bodies(
        self,
        position: np.ndarray,
        radius: float,
        centers: np.ndarray,
        body_radius: float,
    ) -> np.ndarray:
        """Repulsion from cylinders (obstacles or an impassive robot)."""
        points = np.asarray(position, dtype=np.float64).reshape(-1, 2)
        others = np.asarray(centers, dtype=np.float64).reshape(-1, 2)
        if points.shape[0] == 0 or others.shape[0] == 0:
            return np.zeros_like(points)
        delta = points[:, None, :] - others[None, :, :]
        distance = np.linalg.norm(delta, axis=-1)
        gap = float(radius) + float(body_radius)
        strength = float(self.settings["A_obs"]) * np.exp(
            (gap - distance) / max(float(self.settings["B_obs"]), 1e-3)
        )
        direction = delta / np.maximum(distance, 1e-6)[..., None]
        direction[distance < 1e-6] = np.array([1.0, 0.0])
        return (direction * strength[..., None]).sum(axis=1)
