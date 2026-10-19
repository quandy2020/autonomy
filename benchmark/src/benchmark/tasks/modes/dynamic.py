# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Dynamic map regeneration each episode (Arena: dynamic)."""

from __future__ import annotations

from typing import Any

from benchmark.common.types import Pose2D
from benchmark.simulators.base import SimulatorAdapter
from benchmark.simulators.world_state import WorldState
from benchmark.simulators.worlds.generators.dynamic_map import generate_dynamic_layout
from benchmark.simulators.worlds.generators.maze import MazeSpec, generate_maze_obstacles
from benchmark.tasks.modes.base import EpisodeSetup, TaskMode


class DynamicMode(TaskMode):
    name = "dynamic"

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        super().__init__(config)
        self.map_kind = str(self.config.get("map", "maze")).lower()
        self.pedestrians = int(self.config.get("pedestrians", 4))
        self.moving = int(self.config.get("moving_obstacles", 2))
        self.goal_x = float(self.config.get("goal_x", 4.0))

    def setup(self, episode_id: int, simulator: SimulatorAdapter) -> EpisodeSetup:
        seed = episode_id + int(self.config.get("seed", 0))
        # If Fake/Flatland, inject maze disks into world after configure.
        if hasattr(simulator, "world") and isinstance(
            getattr(simulator, "world", None), WorldState
        ):
            if self.map_kind == "maze":
                disks = generate_maze_obstacles(MazeSpec(seed=seed))
            else:
                disks = generate_dynamic_layout(seed=seed, kind=self.map_kind)
            # Stash for configure_world hook via info; navigate applies via obstacles count.
            # Directly install after EpisodeSetup is applied — store on mode for runner.
            self._pending_disks = disks
            n_obs = len(disks)
        else:
            self._pending_disks = []
            n_obs = int(self.config.get("obstacles", 12))

        return EpisodeSetup(
            start=Pose2D(),
            goal=Pose2D(x=self.goal_x, y=0.0),
            obstacles=n_obs,
            pedestrians=self.pedestrians,
            moving_obstacles=self.moving,
            info={"mode": self.name, "map": self.map_kind, "seed": seed},
        )

    def apply_map(self, simulator: SimulatorAdapter) -> None:
        disks = getattr(self, "_pending_disks", None) or []
        if not disks or not hasattr(simulator, "world"):
            return
        world: WorldState = simulator.world  # type: ignore[attr-defined]
        world.obstacles.clear()
        world.obstacles.extend(disks)
