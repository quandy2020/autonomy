# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Dynamic maze obstacle layout (Arena dynamic map generation)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from benchmark.simulators.world_state import ObstacleDisk


@dataclass
class MazeSpec:
    width: float = 20.0
    height: float = 20.0
    cell: float = 2.0
    wall_thickness: float = 0.25
    seed: int = 0


def generate_maze_obstacles(spec: MazeSpec) -> List[ObstacleDisk]:
    """Approximate maze walls as overlapping disks (compatible with WorldState)."""
    rng = np.random.default_rng(spec.seed)
    nx = max(3, int(spec.width / spec.cell))
    ny = max(3, int(spec.height / spec.cell))
    obstacles: List[ObstacleDisk] = []
    # Binary walls on grid edges
    for ix in range(nx):
        for iy in range(ny):
            if rng.random() < 0.35:
                x = -spec.width / 2 + (ix + 0.5) * spec.cell
                y = -spec.height / 2 + (iy + 0.5) * spec.cell
                obstacles.append(ObstacleDisk(x=x, y=y, radius=spec.wall_thickness * 2))
    return obstacles


def maze_to_yaml(spec: MazeSpec) -> dict:
    return {
        "name": "maze",
        "scenario_type": "maze",
        "seed": spec.seed,
        "x_length": spec.width,
        "y_length": spec.height,
        "road_width": spec.cell * 0.4,
        "obstacles_approx": len(generate_maze_obstacles(spec)),
    }
