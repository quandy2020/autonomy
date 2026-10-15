# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Spawn start/goal/obstacles inside named rectangular zones (Arena: zones)."""

from __future__ import annotations

import random
from typing import Any, Dict, List, Optional, Tuple

from benchmark.core.types import Pose2D
from benchmark.simulators.base import SimulatorAdapter
from benchmark.tasks.modes.base import EpisodeSetup, TaskMode


def _sample_in_zone(zone: Dict[str, Any], rng: random.Random) -> Tuple[float, float]:
    x0 = float(zone.get("x_min", -2.0))
    x1 = float(zone.get("x_max", 2.0))
    y0 = float(zone.get("y_min", -2.0))
    y1 = float(zone.get("y_max", 2.0))
    return rng.uniform(x0, x1), rng.uniform(y0, y1)


class ZonesMode(TaskMode):
    name = "zones"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(config)
        self.start_zones: List[Dict[str, Any]] = list(self.config.get("start_zones") or [{"x_min": -1, "x_max": 1, "y_min": -1, "y_max": 1}])
        self.goal_zones: List[Dict[str, Any]] = list(self.config.get("goal_zones") or [{"x_min": 3, "x_max": 5, "y_min": -1, "y_max": 1}])
        self.obstacles = int(self.config.get("obstacles", 8))
        self.pedestrians = int(self.config.get("pedestrians", 4))
        self.moving = int(self.config.get("moving_obstacles", 1))

    def setup(self, episode_id: int, simulator: SimulatorAdapter) -> EpisodeSetup:
        del simulator
        rng = random.Random(episode_id + int(self.config.get("seed", 0)))
        sx, sy = _sample_in_zone(rng.choice(self.start_zones), rng)
        gx, gy = _sample_in_zone(rng.choice(self.goal_zones), rng)
        return EpisodeSetup(
            start=Pose2D(x=sx, y=sy),
            goal=Pose2D(x=gx, y=gy),
            obstacles=self.obstacles,
            pedestrians=self.pedestrians,
            moving_obstacles=self.moving,
            info={"mode": self.name, "zone_episode": episode_id},
        )
