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

"""Scene clouds from SensorSimulator's RandomScene maps.

Real ``forest`` / ``building`` load a PLY (the upstream examples are external).
``random_forest`` stamps trees on a jittered grid (type 5). ``random_room``
builds a windowed wall grid (type 6). See
https://github.com/TJU-Aerial-Robotics/SensorSimulator
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from autosim.map import Map


class Scenes:
    """Point clouds for PLY scenes, random forests, and random rooms."""

    def __init__(self, settings: Mapping[str, Any]) -> None:
        """Bind ``habitat.scenario`` settings.

        Args:
            settings: Scenario mapping (seed, lengths, tree file, room knobs).
        """
        self.settings = settings

    def points(self, kind: str) -> np.ndarray:
        """Build an ``Nx3`` map-frame cloud for ``kind``.

        Args:
            kind: ``forest``, ``building``, ``random_forest``, or ``random_room``.

        Returns:
            Finite ``float64`` points, XY centered and Z shifted onto the ground.
        """
        if kind in ("forest", "building"):
            cloud = self.load_ply()
        elif kind == "random_forest":
            cloud = self.forest()
        else:
            cloud = self.room()
        return self.place(cloud)

    def load_ply(self) -> np.ndarray:
        """Load ``scenario.ply`` (SensorSimulator ``ply_file``).

        Raises:
            ValueError: Path missing.
            FileNotFoundError: File missing or empty.
        """
        raw = str(self.settings.get("ply") or self.settings.get("ply_file") or "").strip()
        if not raw:
            raise ValueError("scenario.ply is required for type forest or building")
        loaded = Map.read_ply(Map.resolve_ply_path(raw))
        if loaded is None:
            raise FileNotFoundError(f"scenario.ply does not exist or is empty: {raw}")
        xyz, _ = loaded
        return np.asarray(xyz, dtype=np.float64)

    def forest(self) -> np.ndarray:
        """Type 5: jittered tree instances plus a ground sheet."""
        width = float(self.settings["x_length"])
        height = float(self.settings["y_length"])
        dist = float(self.settings.get("tree_dist", 8.0))
        step = float(self.settings["resolution"])
        positions = self.poisson(width, height, dist)
        trees = self.stamp_trees(positions)
        ground = self.sheet(
            -0.5 * width, 0.5 * width, -0.5 * height, 0.5 * height, step, 0.0
        )
        if trees.size == 0:
            return ground
        return np.concatenate([trees, ground], axis=0)

    def poisson(self, width: float, height: float, dist: float) -> np.ndarray:
        """Jittered grid used by SensorSimulator (not strict Poisson-disk)."""
        if dist <= 0.0:
            raise ValueError("scenario.tree_dist must be > 0")
        rows = int(width / dist)
        cols = int(height / dist)
        if rows < 1 or cols < 1:
            return np.zeros((0, 2), dtype=np.float64)
        rng = np.random.default_rng(int(self.settings["seed"]))
        offset = rng.uniform(-0.5 * dist, 0.5 * dist, size=(rows, cols, 2))
        xs = np.arange(rows, dtype=np.float64) * dist
        ys = np.arange(cols, dtype=np.float64) * dist
        grid = np.stack(np.broadcast_arrays(xs[:, None], ys[None, :]), axis=-1)
        return (grid + offset).reshape(-1, 2)

    def stamp_trees(self, positions: np.ndarray) -> np.ndarray:
        """Scale, tilt, and place one tree cloud at each site."""
        if positions.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float64)
        tree = self.tree_cloud()
        rng = np.random.default_rng(int(self.settings["seed"]))
        placed = []
        for x, y in positions:
            scale = float(rng.uniform(0.5, 1.0))
            roll = float(rng.uniform(0.0, 1.0)) * np.deg2rad(10.0)
            pitch = float(rng.uniform(0.0, 1.0)) * np.deg2rad(10.0)
            yaw = float(rng.uniform(0.0, 1.0)) * np.deg2rad(360.0)
            rotation = self.euler(yaw, pitch, roll) * scale
            placed.append(tree @ rotation.T + np.array([x, y, 0.0]))
        return np.concatenate(placed, axis=0)

    def tree_cloud(self) -> np.ndarray:
        """Load ``tree_file`` or build a trunk-and-canopy substitute."""
        raw = str(self.settings.get("tree_file") or "").strip()
        if not raw:
            return self.procedural_tree(float(self.settings["resolution"]))
        loaded = Map.read_ply(Map.resolve_ply_path(raw))
        if loaded is None:
            raise FileNotFoundError(f"scenario.tree_file does not exist or is empty: {raw}")
        xyz, _ = loaded
        return np.asarray(xyz, dtype=np.float64)

    @staticmethod
    def procedural_tree(step: float) -> np.ndarray:
        """Small tree used when SensorSimulator's ``tree.ply`` is not configured."""
        step = max(float(step), 0.05)
        angles = np.linspace(0.0, 2.0 * np.pi, 8, endpoint=False)
        heights = np.arange(0.0, 2.0, step)
        z, angle = np.meshgrid(heights, angles, indexing="ij")
        trunk = np.stack(
            [0.15 * np.cos(angle), 0.15 * np.sin(angle), z], axis=-1
        ).reshape(-1, 3)
        phi = np.linspace(0.0, np.pi, 6)
        theta = np.linspace(0.0, 2.0 * np.pi, 10, endpoint=False)
        pp, tt = np.meshgrid(phi, theta, indexing="ij")
        canopy = np.stack(
            [
                np.sin(pp) * np.cos(tt),
                np.sin(pp) * np.sin(tt),
                2.4 + np.cos(pp),
            ],
            axis=-1,
        ).reshape(-1, 3)
        return np.concatenate([trunk, canopy], axis=0)

    def room(self) -> np.ndarray:
        """Type 6: grid of walls with random windows, optionally a ceiling."""
        count = int(self.settings.get("room_number", 4))
        if count < 1:
            raise ValueError("scenario.room_number must be >= 1")
        length = float(self.settings["x_length"]) / count
        height = float(self.settings["z_length"])
        step = float(self.settings["resolution"])
        rng = np.random.default_rng(int(self.settings["seed"]))
        walls = [
            self.transformed_wall(length, height, step, rng, horizontal, i, j)
            for i in range(count + 1)
            for j in range(count + 1)
            for horizontal in (True, False)
            if (i < count if horizontal else j < count)
        ]
        cloud = np.concatenate(walls, axis=0)
        cloud[:, 0] -= 0.5 * count * length
        cloud[:, 1] -= 0.5 * count * length
        if int(self.settings.get("add_ceiling", 0)):
            half = 0.5 * count * length
            ceiling = self.sheet(-half, half, -half, half, step, height - step)
            cloud = np.concatenate([cloud, ceiling], axis=0)
        return cloud

    def transformed_wall(
        self,
        length: float,
        height: float,
        step: float,
        rng: np.random.Generator,
        horizontal: bool,
        i: int,
        j: int,
    ) -> np.ndarray:
        """One wall in the room grid, rotated 90° when it runs along y."""
        windows = max(int(self.settings.get("max_windows", 2)), 1)
        count = int(rng.integers(0, 101)) % windows + 1
        local = self.wall(length, 0.2, height, step, self.openings(length, height, count, rng))
        if not horizontal:
            rotated = np.empty_like(local)
            rotated[:, 0] = -local[:, 1]
            rotated[:, 1] = local[:, 0]
            rotated[:, 2] = local[:, 2]
            local = rotated
        local[:, 0] += i * length
        local[:, 1] += j * length
        return local

    def openings(
        self, length: float, height: float, count: int, rng: np.random.Generator
    ) -> list[tuple[float, float, float, float]]:
        """Window rectangles ``(left, bottom, width, height)`` in wall coordinates."""
        low = float(self.settings.get("window_size_min", 2.0))
        high = float(self.settings.get("window_size_max", 2.8))
        holes = []
        for _ in range(count):
            center_x = float(rng.uniform(0.1, 0.9)) * max(length - 0.5, 0.1)
            center_z = float(rng.uniform(0.1, 0.9)) * max(height - 0.5, 0.1)
            width = min(float(rng.uniform(low, high)), max(length - center_x, 0.0))
            hole_h = min(float(rng.uniform(low, high)), max(height - center_z, 0.0))
            holes.append((center_x - 0.5 * width, center_z - 0.5 * hole_h, width, hole_h))
        return holes

    @staticmethod
    def wall(
        length: float,
        thickness: float,
        height: float,
        step: float,
        holes: list[tuple[float, float, float, float]],
    ) -> np.ndarray:
        """Solid wall samples with window holes removed."""
        xs = np.arange(0.0, length + 1e-9, step)
        ys = np.arange(0.0, thickness + 1e-9, step)
        zs = np.arange(0.0, height + 1e-9, step)
        x, y, z = np.meshgrid(xs, ys, zs, indexing="ij")
        points = np.stack([x.ravel(), y.ravel(), z.ravel()], axis=1)
        keep = np.ones(points.shape[0], dtype=bool)
        for left, bottom, width, hole_h in holes:
            inside = (
                (points[:, 0] >= left)
                & (points[:, 0] <= left + width)
                & (points[:, 2] >= bottom)
                & (points[:, 2] <= bottom + hole_h)
            )
            keep &= ~inside
        return points[keep]

    @staticmethod
    def sheet(
        x0: float, x1: float, y0: float, y1: float, step: float, height: float
    ) -> np.ndarray:
        """Axis-aligned horizontal point sheet at ``height``."""
        xs = np.arange(x0, x1 + 1e-9, step)
        ys = np.arange(y0, y1 + 1e-9, step)
        if xs.size == 0 or ys.size == 0:
            return np.zeros((0, 3), dtype=np.float64)
        x, y = np.meshgrid(xs, ys, indexing="ij")
        z = np.full(x.shape, height, dtype=np.float64)
        return np.stack([x, y, z], axis=-1).reshape(-1, 3)

    @staticmethod
    def euler(yaw: float, pitch: float, roll: float) -> np.ndarray:
        """``Rz(yaw) @ Ry(pitch) @ Rx(roll)``, matching Eigen ``AngleAxis`` order."""
        cz, sz = np.cos(yaw), np.sin(yaw)
        cy, sy = np.cos(pitch), np.sin(pitch)
        cx, sx = np.cos(roll), np.sin(roll)
        rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
        ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
        return rz @ ry @ rx

    @staticmethod
    def place(points: np.ndarray) -> np.ndarray:
        """Center XY and drop the lowest point onto z = 0."""
        cloud = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        cloud = cloud[np.isfinite(cloud).all(axis=1)]
        if cloud.size == 0:
            return np.zeros((0, 3), dtype=np.float64)
        cloud[:, 0] -= 0.5 * (cloud[:, 0].min() + cloud[:, 0].max())
        cloud[:, 1] -= 0.5 * (cloud[:, 1].min() + cloud[:, 1].max())
        cloud[:, 2] -= cloud[:, 2].min()
        return cloud
