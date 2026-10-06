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

"""2D recursive-division and 3D Voronoi mazes from mockamap.

Indexing is ``maze[x, y]`` with ``1`` = wall. The upstream
``recursiveDivision`` call swaps Eigen rows and columns; this port keeps
x along the first axis so the extruded point cloud matches ``x_length`` /
``y_length``.
"""

from __future__ import annotations

import numpy as np


class Maze:
    """Occupancy grids for mockamap maze types 3 and 4."""

    def __init__(self, seed: int) -> None:
        """Create the maze RNG.

        Args:
            seed: Integer seed (mockamap ``seed``).
        """
        self.rng = np.random.default_rng(int(seed))

    def division(self, width: int, height: int, maze_type: int = 1) -> np.ndarray:
        """Recursive-division maze.

        Args:
            width: Cell count along x.
            height: Cell count along y.
            maze_type: Only ``1`` (recursive division) is implemented.

        Returns:
            ``uint8`` array ``(width, height)``, ``1`` = wall.

        Raises:
            ValueError: Unknown ``maze_type``.
        """
        if int(maze_type) != 1:
            raise ValueError("scenario maze_type supports only 1 (recursive division)")
        maze = np.zeros((max(int(width), 1), max(int(height), 1)), dtype=np.uint8)
        if maze.shape[0] >= 3 and maze.shape[1] >= 3:
            self.divide(maze, 0, maze.shape[0] - 1, 0, maze.shape[1] - 1)
        return maze

    def divide(self, maze: np.ndarray, xl: int, xh: int, yl: int, yh: int) -> None:
        """Split one inclusive cell of ``maze`` and recurse when it is large."""
        nx, ny = maze.shape
        if xl < xh - 3 and yl < yh - 3:
            self.split_large(maze, xl, xh, yl, yh, nx, ny)
            return
        if xl < xh - 2 and yl < yh - 2:
            self.split_cross(maze, xl, xh, yl, yh, nx, ny)
            return
        if xl < xh - 1 and yl < yh - 2:
            self.split_vertical(maze, xl, yl, yh, ny)
            return
        if xl < xh - 2 and yl < yh - 1:
            self.split_horizontal(maze, xl, xh, yl, nx)
            return
        if xl < xh - 1 and yl < yh - 1:
            maze[xl + 1, yl + 1] = 1

    def split_large(
        self, maze: np.ndarray, xl: int, xh: int, yl: int, yh: int, nx: int, ny: int
    ) -> None:
        """Draw a cross in a region at least 5×5 and recurse into four rooms."""
        xm, ym = self.wall_center(maze, xl, xh, yl, yh, nx, ny)
        self.paint_cross(maze, xl, xh, yl, yh, xm, ym)
        self.open_doors(maze, xl, xh, yl, yh, xm, ym)
        self.join_openings(maze, xl, xh, yl, yh, xm, ym, nx, ny)
        self.divide(maze, xl, xm - 1, yl, ym - 1)
        self.divide(maze, xm + 1, xh, yl, ym - 1)
        self.divide(maze, xl, xm - 1, ym + 1, yh)
        self.divide(maze, xm + 1, xh, ym + 1, yh)

    def wall_center(
        self, maze: np.ndarray, xl: int, xh: int, yl: int, yh: int, nx: int, ny: int
    ) -> tuple[int, int]:
        """Pick a cross center that does not seal an existing doorway."""
        xm = xl + 1
        ym = yl + 1
        for _ in range(64):
            xm = int(self.rng.integers(xl + 1, xh))
            ym = int(self.rng.integers(yl + 1, yh))
            if self.blocks_door(maze, xl, xh, yl, yh, xm, ym, nx, ny):
                continue
            return xm, ym
        return xm, ym

    @staticmethod
    def blocks_door(
        maze: np.ndarray,
        xl: int,
        xh: int,
        yl: int,
        yh: int,
        xm: int,
        ym: int,
        nx: int,
        ny: int,
    ) -> bool:
        """True when the candidate cross closes a door in a neighboring cell."""
        if xl - 1 >= 0 and maze[xl - 1, ym] == 0:
            return True
        if xh + 1 < nx and maze[xh + 1, ym] == 0:
            return True
        if yl - 1 >= 0 and maze[xm, yl - 1] == 0:
            return True
        if yh + 1 < ny and maze[xm, yh + 1] == 0:
            return True
        return False

    @staticmethod
    def paint_cross(
        maze: np.ndarray, xl: int, xh: int, yl: int, yh: int, xm: int, ym: int
    ) -> None:
        """Fill the horizontal and vertical wall through ``(xm, ym)``."""
        maze[xl : xh + 1, ym] = 1
        maze[xm, yl : yh + 1] = 1

    def open_doors(
        self, maze: np.ndarray, xl: int, xh: int, yl: int, yh: int, xm: int, ym: int
    ) -> None:
        """Punch three of the four possible doorways (mockamap switch)."""
        d1 = int(self.rng.integers(xl, xm))
        d2 = int(self.rng.integers(xm + 1, xh + 1))
        d3 = int(self.rng.integers(yl, ym))
        d4 = int(self.rng.integers(ym + 1, yh + 1))
        choice = int(self.rng.integers(0, 4))
        doors = {
            0: ((d1, ym), (d2, ym), (xm, d3)),
            1: ((d1, ym), (d2, ym), (xm, d4)),
            2: ((d2, ym), (xm, d3), (xm, d4)),
            3: ((d1, ym), (xm, d3), (xm, d4)),
        }
        for x, y in doors[choice]:
            maze[x, y] = 0

    @staticmethod
    def join_openings(
        maze: np.ndarray,
        xl: int,
        xh: int,
        yl: int,
        yh: int,
        xm: int,
        ym: int,
        nx: int,
        ny: int,
    ) -> None:
        """Reopen the wall where it meets a door in an adjacent region."""
        if yl - 1 >= 0 and maze[xm, yl - 1] == 0:
            maze[xm, yl] = 0
        if yh + 1 < ny and maze[xm, yh + 1] == 0:
            maze[xm, yh] = 0
        if xl - 1 >= 0 and maze[xl - 1, ym] == 0:
            maze[xl, ym] = 0
        if xh + 1 < nx and maze[xh + 1, ym] == 0:
            maze[xh, ym] = 0

    def split_cross(
        self, maze: np.ndarray, xl: int, xh: int, yl: int, yh: int, nx: int, ny: int
    ) -> None:
        """One cross and three doors, without further recursion."""
        xm = int(self.rng.integers(xl + 1, xh))
        ym = int(self.rng.integers(yl + 1, yh))
        self.paint_cross(maze, xl, xh, yl, yh, xm, ym)
        self.join_openings(maze, xl, xh, yl, yh, xm, ym, nx, ny)
        self.open_doors(maze, xl, xh, yl, yh, xm, ym)

    def split_vertical(self, maze: np.ndarray, xl: int, yl: int, yh: int, ny: int) -> None:
        """Wall across a 3-wide strip, with a door if none was inherited."""
        maze[xl + 1, yl : yh + 1] = 1
        opened = 0
        if yl - 1 >= 0 and maze[xl + 1, yl - 1] == 0:
            maze[xl + 1, yl] = 0
            opened += 1
        if yh + 1 < ny and maze[xl + 1, yh + 1] == 0:
            maze[xl + 1, yh] = 0
            opened += 1
        if opened == 0:
            maze[xl + 1, int(self.rng.integers(yl, yh + 1))] = 0

    def split_horizontal(self, maze: np.ndarray, xl: int, xh: int, yl: int, nx: int) -> None:
        """Wall across a 3-tall strip, with a door if none was inherited."""
        maze[xl : xh + 1, yl + 1] = 1
        opened = 0
        if xl - 1 >= 0 and maze[xl - 1, yl + 1] == 0:
            maze[xl, yl + 1] = 0
            opened += 1
        if xh + 1 < nx and maze[xh + 1, yl + 1] == 0:
            maze[xh, yl + 1] = 0
            opened += 1
        if opened == 0:
            maze[int(self.rng.integers(xl, xh + 1)), yl + 1] = 0

    def voronoi(
        self,
        shape: tuple[int, int, int],
        scale: float,
        num_nodes: int,
        connectivity: float,
        road_radius: int,
    ) -> np.ndarray:
        """3D maze: voxels on the mid-plane between the two nearest nodes.

        ``nodeRad`` is accepted by mockamap but unused there; it is not an
        argument here. Built in x-slices so the node-distance tensor stays small.

        Args:
            shape: ``(nx, ny, nz)`` voxel counts.
            scale: Voxels per meter (``1 / resolution``).
            num_nodes: Random core count.
            connectivity: Fraction of node-index pairs that leave a gap.
            road_radius: Mockamap ``roadRad`` (voxel units).

        Returns:
            Boolean occupancy ``(nx, ny, nz)``.
        """
        nx, ny, nz = shape
        nodes = self.cores(nx, ny, nz, scale, num_nodes)
        occupied = np.zeros(shape, dtype=bool)
        ys = np.arange(ny, dtype=np.float64) / scale - ny / (2.0 * scale)
        zs = np.arange(nz, dtype=np.float64) / scale - nz / (2.0 * scale)
        yy, zz = np.meshgrid(ys, zs, indexing="ij")
        limit = 1.0 / scale
        gap = float(road_radius) / (scale * 3.0)
        low = int((1.0 - connectivity) * num_nodes)
        high = int((1.0 + connectivity) * num_nodes)
        for index, x in enumerate(np.arange(nx, dtype=np.float64) / scale - nx / (2.0 * scale)):
            occupied[index] = self.voronoi_slice(nodes, x, yy, zz, limit, gap, low, high)
        return occupied

    def cores(
        self, nx: int, ny: int, nz: int, scale: float, num_nodes: int
    ) -> np.ndarray:
        """Random maze cores in meters, matching mockamap's centered box."""
        count = max(int(num_nodes), 2)
        span = np.array([nx, ny, nz], dtype=np.float64) / scale
        jitter = self.rng.random((count, 3))
        cells = self.rng.integers(0, [nx, ny, nz], size=(count, 3))
        return jitter + cells / scale - span / 2.0

    def voronoi_slice(
        self,
        nodes: np.ndarray,
        x: float,
        yy: np.ndarray,
        zz: np.ndarray,
        limit: float,
        gap: float,
        low: int,
        high: int,
    ) -> np.ndarray:
        """Occupied mask for one x-slice of the 3D maze."""
        points = np.stack(
            [np.full(yy.shape, x), yy, zz],
            axis=-1,
        )
        dist = np.linalg.norm(nodes[:, None, None, :] - points[None, ...], axis=-1)
        nearest = np.argpartition(dist, 1, axis=0)[:2]
        d_near = np.take_along_axis(dist, nearest, axis=0)
        order = np.argsort(d_near, axis=0)
        d1 = np.take_along_axis(d_near, order[:1], axis=0)[0]
        d2 = np.take_along_axis(d_near, order[1:], axis=0)[0]
        i1 = np.take_along_axis(nearest, order[:1], axis=0)[0]
        i2 = np.take_along_axis(nearest, order[1:], axis=0)[0]
        on_wall = np.abs(d2 - d1) < limit
        holed = (i1 + i2 > low) & (i1 + i2 < high)
        judge = np.linalg.norm(nodes[i1] - nodes[i2], axis=-1)
        thick = d1 + d2 - judge >= gap
        return on_wall & (~holed | thick)
