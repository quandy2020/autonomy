# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from benchmark.worlds import PRESETS, generate_world
from benchmark.worlds.generators.maze import MazeSpec, generate_maze_obstacles


def test_presets() -> None:
    for name in PRESETS:
        data = generate_world(name, seed=1)
        assert data["name"] == name or data.get("scenario_type")


def test_maze_obstacles() -> None:
    obs = generate_maze_obstacles(MazeSpec(seed=2, width=10, height=10, cell=2))
    assert len(obs) >= 1
