# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Load a fixed scenario file / preset layout (Arena: scenario)."""

from __future__ import annotations

from typing import Any, Dict, Optional

from benchmark.core.types import Pose2D
from benchmark.simulators.base import SimulatorAdapter
from benchmark.tasks.modes.base import EpisodeSetup, TaskMode


class ScenarioMode(TaskMode):
    name = "scenario"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(config)
        self.start = Pose2D(
            x=float(self.config.get("start_x", 0.0)),
            y=float(self.config.get("start_y", 0.0)),
            yaw=float(self.config.get("start_yaw", 0.0)),
        )
        self.goal = Pose2D(
            x=float(self.config.get("goal_x", 5.0)),
            y=float(self.config.get("goal_y", 0.0)),
        )
        self.obstacles = int(self.config.get("obstacles", 8))
        self.pedestrians = int(self.config.get("pedestrians", 3))
        self.moving = int(self.config.get("moving_obstacles", 0))

    def setup(self, episode_id: int, simulator: SimulatorAdapter) -> EpisodeSetup:
        del episode_id, simulator
        return EpisodeSetup(
            start=self.start,
            goal=self.goal,
            obstacles=self.obstacles,
            pedestrians=self.pedestrians,
            moving_obstacles=self.moving,
            info={"mode": self.name},
        )
