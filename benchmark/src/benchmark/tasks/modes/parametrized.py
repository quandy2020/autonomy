# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Curriculum-style parametrized randomness (Arena: parametrized)."""

from __future__ import annotations

from typing import Any

from benchmark.simulators.base import SimulatorAdapter
from benchmark.tasks.modes.base import EpisodeSetup, TaskMode
from benchmark.tasks.modes.random import RandomMode


class ParametrizedMode(TaskMode):
    name = "parametrized"

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        super().__init__(config)
        # difficulty in [0, 1] scales obstacle/pedestrian density
        self.difficulty = float(self.config.get("difficulty", 0.5))

    def setup(self, episode_id: int, simulator: SimulatorAdapter) -> EpisodeSetup:
        d = max(0.0, min(1.0, self.difficulty))
        cfg = dict(self.config)
        cfg["obstacles_min"] = int(5 + 20 * d)
        cfg["obstacles_max"] = int(10 + 30 * d)
        cfg["pedestrians_min"] = int(10 * d)
        cfg["pedestrians_max"] = int(5 + 20 * d)
        cfg["moving_obstacles"] = int(1 + 5 * d)
        setup = RandomMode(cfg).setup(episode_id, simulator)
        setup.info["mode"] = self.name
        setup.info["difficulty"] = d
        return setup
