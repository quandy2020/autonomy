# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from benchmark.worlds.generators.dynamic_map import PRESETS, generate_world
from benchmark.worlds.generators.maze import MazeSpec, generate_maze_obstacles

__all__ = ["MazeSpec", "PRESETS", "generate_maze_obstacles", "generate_world"]
