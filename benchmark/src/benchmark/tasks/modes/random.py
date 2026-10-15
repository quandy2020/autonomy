# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Random start/goal/obstacle placement (Arena: random)."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

import numpy as np

from benchmark.core.types import Pose2D
from benchmark.simulators.base import SimulatorAdapter
from benchmark.tasks.modes.base import EpisodeSetup, TaskMode


class RandomMode(TaskMode):
    name = "random"

    def setup(self, episode_id: int, simulator: SimulatorAdapter) -> EpisodeSetup:
        rng = np.random.default_rng(int(self.config.get("seed", 0)) + episode_id)
        half = float(self.config.get("map_half", 8.0))
        margin = float(self.config.get("margin", 1.5))

        def sample() -> Pose2D:
            return Pose2D(
                x=float(rng.uniform(-half + margin, half - margin)),
                y=float(rng.uniform(-half + margin, half - margin)),
                yaw=float(rng.uniform(-math.pi, math.pi)),
            )

        start = sample()
        goal = sample()
        while math.hypot(start.x - goal.x, start.y - goal.y) < 3.0:
            goal = sample()
        return EpisodeSetup(
            start=start,
            goal=goal,
            obstacles=int(rng.integers(
                int(self.config.get("obstacles_min", 8)),
                int(self.config.get("obstacles_max", 25)) + 1,
            )),
            pedestrians=int(rng.integers(
                int(self.config.get("pedestrians_min", 0)),
                int(self.config.get("pedestrians_max", 10)) + 1,
            )),
            moving_obstacles=int(self.config.get("moving_obstacles", 3)),
            info={"mode": self.name},
        )
