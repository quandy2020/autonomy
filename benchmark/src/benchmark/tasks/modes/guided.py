# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Waypoint sequence navigation (Arena: guided)."""

from __future__ import annotations

from typing import Any

from benchmark.common.types import Pose2D
from benchmark.simulators.base import SimulatorAdapter
from benchmark.tasks.modes.base import EpisodeSetup, TaskMode


class GuidedMode(TaskMode):
    name = "guided"

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        super().__init__(config)
        raw = self.config.get("waypoints") or [
            {"x": 2.0, "y": 0.0},
            {"x": 4.0, "y": 2.0},
            {"x": 6.0, "y": 0.0},
        ]
        self.waypoints: list[Pose2D] = [
            Pose2D(x=float(w["x"]), y=float(w.get("y", 0.0)), yaw=float(w.get("yaw", 0.0)))
            for w in raw
        ]

    def setup(self, episode_id: int, simulator: SimulatorAdapter) -> EpisodeSetup:
        del episode_id, simulator
        goal = self.waypoints[-1] if self.waypoints else Pose2D(x=5.0)
        return EpisodeSetup(
            start=Pose2D(),
            goal=goal,
            obstacles=int(self.config.get("obstacles", 8)),
            pedestrians=int(self.config.get("pedestrians", 3)),
            moving_obstacles=int(self.config.get("moving_obstacles", 1)),
            waypoints=list(self.waypoints),
            info={"mode": self.name, "n_waypoints": len(self.waypoints)},
        )
