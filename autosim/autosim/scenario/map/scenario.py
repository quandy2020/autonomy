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

"""Procedural map and Habitat stage when no GLB is configured.

Map types follow `SensorSimulator <https://github.com/TJU-Aerial-Robotics/SensorSimulator>`_
(RandomScene) and the mockamap generators it builds on: 2D/3D Perlin, posts,
random maze, 3D maze, real PLY forest/building, random forest, and random room.
The result is an occupied point cloud plus a triangle mesh in Habitat Y-up.
This does not build or launch the ROS package.
"""

from __future__ import annotations

import tempfile
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from autosim.scenario.map.maze import Maze
from autosim.scenario.map.perlin import Perlin
from autosim.scenario.map.scenes import Scenes
from autosim.scenario.map.stage import Stage
from autosim.scenario.map.volume import Volume


class Scenario:
    """mockamap-style occupancy, point cloud, and collision mesh."""

    DEFAULTS: dict[str, Any] = {
        "type": "perlin",
        "seed": 511,
        "resolution": 0.1,
        "x_length": 10.0,
        "y_length": 10.0,
        "z_length": 3.0,
        "complexity": 0.03,
        "fill": 0.3,
        "fractal": 1,
        "attenuation": 0.1,
        "width_min": 0.6,
        "width_max": 1.5,
        "obstacle_number": 50,
        "road_width": 0.5,
        "add_wall_x": 0,
        "add_wall_y": 1,
        "maze_type": 1,
        "num_nodes": 40,
        "connectivity": 0.8,
        "node_radius": 1,
        "road_radius": 10,
        "tree_dist": 8.0,
        "tree_file": "",
        "ply": "",
        "room_number": 4,
        "max_windows": 2,
        "window_size_min": 2.0,
        "window_size_max": 2.8,
        "add_ceiling": 0,
        "mesh": "",
        "spawn_radius": 0.8,
        "spawn_height": 1.2,
    }
    CLOUD_KINDS = ("forest", "building", "random_forest", "random_room")
    KINDS = {
        1: "perlin",
        2: "posts",
        3: "maze",
        4: "maze3d",
        5: "random_forest",
        6: "random_room",
        "1": "perlin",
        "2": "posts",
        "3": "maze",
        "4": "maze3d",
        "5": "random_forest",
        "6": "random_room",
        "perlin": "perlin",
        "perlin3d": "perlin",
        "3d_perlin": "perlin",
        "perlin2d": "perlin2d",
        "2d_perlin": "perlin2d",
        "posts": "posts",
        "random": "posts",
        "maze": "maze",
        "maze2d": "maze",
        "random_maze": "maze",
        "maze3d": "maze3d",
        "forest": "forest",
        "building": "building",
        "random_forest": "random_forest",
        "random_room": "random_room",
        "room": "random_room",
    }

    def __init__(self, settings: Mapping[str, Any] | None = None) -> None:
        """Copy ``habitat.scenario`` over mockamap launch defaults.

        Args:
            settings: Partial scenario mapping. Unknown keys are kept.
        """
        merged = dict(self.DEFAULTS)
        merged.update(dict(settings or {}))
        self.settings = merged
        self.kind = self.kind_name(merged.get("type", "perlin"))
        self.points = np.zeros((0, 3), dtype=np.float32)
        self.grid: tuple[np.ndarray, float, float, float, int, int] | None = None
        self.occupied: np.ndarray | None = None
        self.origin = np.zeros(3, dtype=np.float64)
        self.resolution = 0.0
        self.mesh: Path | None = None

    @classmethod
    def kind_name(cls, raw: Any) -> str:
        """Map a mockamap type id or name to a generator key.

        Raises:
            ValueError: Unknown type.
        """
        key: Any = raw.strip().lower() if isinstance(raw, str) else raw
        try:
            return cls.KINDS[key]
        except KeyError as exc:
            raise ValueError(
                "scenario.type must be perlin, perlin2d, posts, maze, maze3d, "
                "forest, building, random_forest, random_room, or 1–6"
            ) from exc

    def build(self, *, write_mesh: bool = True) -> "Scenario":
        """Fill occupancy, carve a spawn pocket, and optionally write the stage mesh.

        Args:
            write_mesh: When False (fake backend), skip Habitat OBJ export.

        Returns:
            This object, with ``points``, ``grid``, ``occupied``, and optionally
            ``mesh`` set.
        """
        occupied, origin, resolution = self.make_occupancy()
        Volume.clear_spawn(
            occupied,
            origin,
            resolution,
            float(self.settings["spawn_radius"]),
            float(self.settings["spawn_height"]),
        )
        self.occupied = occupied
        self.origin = np.asarray(origin, dtype=np.float64).reshape(3)
        self.resolution = float(resolution)
        self.points = Volume.corners(occupied, origin, resolution)
        self.grid = Volume.planar(
            occupied, origin, resolution, z_max=float(self.settings["spawn_height"])
        )
        if write_mesh:
            self.mesh = Stage().write(occupied, origin, resolution, self.mesh_file())
        else:
            self.mesh = None
        return self

    def make_occupancy(self) -> tuple[np.ndarray, np.ndarray, float]:
        """Dispatch on ``type``.

        Returns:
            ``(occupied[x,y,z], origin_xyz, resolution)``. Origin is the
            minimum corner of voxel ``(0,0,0)`` in the map frame (z up).
        """
        resolution = float(self.settings["resolution"])
        if resolution <= 0.0:
            raise ValueError("scenario.resolution must be > 0")
        if self.kind in self.CLOUD_KINDS:
            return Volume.from_points(Scenes(self.settings).points(self.kind), resolution)
        lengths = tuple(float(self.settings[key]) for key in ("x_length", "y_length", "z_length"))
        for key, length in zip(("x_length", "y_length", "z_length"), lengths):
            if length <= 0.0:
                raise ValueError(f"scenario.{key} must be > 0")
        shape = Volume.shape(lengths, resolution)
        origin = Volume.origin(shape, resolution)
        occupied = self.fill_kind(shape, resolution, origin)
        if self.kind == "maze3d":
            origin = origin.copy()
            origin[2] = -shape[2] / (2.0 / resolution)
        return occupied, origin, resolution

    def fill_kind(
        self,
        shape: tuple[int, int, int],
        resolution: float,
        origin: np.ndarray,
    ) -> np.ndarray:
        """Run the generator registered for ``kind``.

        Raises:
            ValueError: The type has no generator.
        """
        fillers = {
            "perlin": lambda: self.fill_perlin(shape),
            "perlin2d": lambda: self.fill_perlin2d(shape),
            "posts": lambda: self.fill_posts(shape, resolution, origin),
            "maze": lambda: self.fill_maze(shape, resolution),
            "maze3d": lambda: self.fill_maze3d(shape, resolution),
        }
        try:
            return fillers[self.kind]()
        except KeyError as exc:
            raise ValueError(f"scenario type {self.kind} has no generator") from exc

    def fill_perlin(self, shape: tuple[int, int, int]) -> np.ndarray:
        """Type 1: fractal Perlin field, occupied above the fill quantile."""
        grid = np.indices(shape, dtype=np.float64)
        field = self.perlin_field(grid[0], grid[1], grid[2])
        return Perlin.occupy(field, float(self.settings["fill"]))

    def fill_perlin2d(self, shape: tuple[int, int, int]) -> np.ndarray:
        """2D Perlin extruded through z (a planar counterpart of type 1)."""
        nx, ny, _nz = shape
        ii, jj = np.meshgrid(np.arange(nx), np.arange(ny), indexing="ij")
        field = self.perlin_field(ii, jj, np.zeros_like(ii))
        mask = Perlin.occupy(field, float(self.settings["fill"]))
        return np.broadcast_to(mask[:, :, None], shape).copy()

    def perlin_field(self, x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
        """Fractal noise on the given coordinates."""
        return Perlin(int(self.settings["seed"])).fractal(
            x,
            y,
            z,
            float(self.settings["complexity"]),
            float(self.settings["attenuation"]),
            max(1, int(self.settings["fractal"])),
        )

    def fill_posts(
        self,
        shape: tuple[int, int, int],
        resolution: float,
        origin: np.ndarray,
    ) -> np.ndarray:
        """Type 2: hollow boxes (mockamap ``randomMapGenerate`` / post2d)."""
        nx, ny, nz = shape
        occupied = np.zeros(shape, dtype=bool)
        rng = np.random.default_rng(int(self.settings["seed"]))
        count = max(0, int(self.settings["obstacle_number"]))
        high = origin + np.array(shape, dtype=np.float64) * resolution
        for _ in range(count):
            center_x = float(rng.uniform(origin[0], high[0]))
            center_y = float(rng.uniform(origin[1], high[1]))
            width = float(rng.uniform(self.settings["width_min"], self.settings["width_max"]))
            height = float(rng.uniform(0.0, high[2]))
            self.paint_shell(
                occupied,
                int(round((center_x - origin[0]) / resolution)),
                int(round((center_y - origin[1]) / resolution)),
                int(np.ceil(width / resolution)),
                int(np.ceil(height / resolution)),
            )
        return occupied

    @staticmethod
    def paint_shell(
        occupied: np.ndarray, ic: int, jc: int, wid: int, hei: int
    ) -> None:
        """Mark the surface of one axis-aligned post, clipped to the volume."""
        half = max(wid, 1) // 2
        xs = slice(max(ic - half, 0), min(ic + half, occupied.shape[0]))
        ys = slice(max(jc - half, 0), min(jc + half, occupied.shape[1]))
        zs = slice(0, min(max(hei, 1), occupied.shape[2]))
        box = occupied[xs, ys, zs]
        if box.size == 0:
            return
        shell = np.zeros(box.shape, dtype=bool)
        shell[0, :, :] = shell[-1, :, :] = True
        shell[:, 0, :] = shell[:, -1, :] = True
        shell[:, :, 0] = shell[:, :, -1] = True
        box |= shell

    def fill_maze(self, shape: tuple[int, int, int], resolution: float) -> np.ndarray:
        """Type 3: recursive-division maze extruded through z."""
        road = float(self.settings["road_width"])
        if road <= 0.0:
            raise ValueError("scenario.road_width must be > 0")
        span = max(1, int(road / resolution + 1e-9))
        cells = Maze(int(self.settings["seed"])).division(
            max(1, shape[0] // span),
            max(1, shape[1] // span),
            int(self.settings["maze_type"]),
        )
        self.seal_borders(cells)
        occupied = np.zeros(shape, dtype=bool)
        walls = np.argwhere(cells > 0)
        for i, j in walls:
            occupied[
                i * span : min((i + 1) * span, shape[0]),
                j * span : min((j + 1) * span, shape[1]),
                :,
            ] = True
        return occupied

    def seal_borders(self, cells: np.ndarray) -> None:
        """Optional outer walls (mockamap ``add_wall_x`` / ``add_wall_y``)."""
        if int(self.settings["add_wall_x"]):
            cells[0, :] = 1
            cells[-1, :] = 1
        if int(self.settings["add_wall_y"]):
            cells[:, 0] = 1
            cells[:, -1] = 1

    def fill_maze3d(self, shape: tuple[int, int, int], resolution: float) -> np.ndarray:
        """Type 4: 3D Voronoi walls. ``node_radius`` is unused, as in mockamap."""
        connectivity = float(self.settings["connectivity"])
        if not 0.0 <= connectivity <= 1.0:
            raise ValueError("scenario.connectivity must be in [0, 1]")
        return Maze(int(self.settings["seed"])).voronoi(
            shape,
            1.0 / resolution,
            int(self.settings["num_nodes"]),
            connectivity,
            int(self.settings["road_radius"]),
        )

    def mesh_file(self) -> Path:
        """OBJ destination from ``mesh``, or a temp file named by type and seed."""
        raw = str(self.settings.get("mesh") or "").strip()
        if raw:
            return Path(raw)
        name = f"autosim-scenario-{self.kind}-{int(self.settings['seed'])}.obj"
        return Path(tempfile.gettempdir()) / name
