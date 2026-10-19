# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Coverage / frontier exploration (Arena: explore)."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from benchmark.common.types import Pose2D
from benchmark.simulators.base import SimulatorAdapter
from benchmark.tasks.modes.base import EpisodeSetup, TaskMode


class ExploreMode(TaskMode):
    name = "explore"

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        super().__init__(config)
        self.cell = float(self.config.get("cell_size", 1.0))
        self.coverage_target = float(self.config.get("coverage_target", 0.6))
        self.visited: set[tuple[int, int]] = set()

    def setup(self, episode_id: int, simulator: SimulatorAdapter) -> EpisodeSetup:
        del simulator
        self.visited.clear()
        rng = np.random.default_rng(int(self.config.get("seed", 0)) + episode_id)
        # Exploration "goal" is a far corner that shifts each episode
        angle = float(rng.uniform(0, 2 * math.pi))
        radius = float(self.config.get("explore_radius", 7.0))
        goal = Pose2D(x=radius * math.cos(angle), y=radius * math.sin(angle))
        return EpisodeSetup(
            start=Pose2D(),
            goal=goal,
            obstacles=int(self.config.get("obstacles", 20)),
            pedestrians=int(self.config.get("pedestrians", 0)),
            moving_obstacles=0,
            info={
                "mode": self.name,
                "coverage_target": self.coverage_target,
                "cell_size": self.cell,
            },
        )

    def update_coverage(self, pose: Pose2D) -> float:
        key = (int(pose.x / self.cell), int(pose.y / self.cell))
        self.visited.add(key)
        half = int(float(self.config.get("map_half", 8.0)) / self.cell)
        total = max(1, (2 * half) * (2 * half))
        return len(self.visited) / total
