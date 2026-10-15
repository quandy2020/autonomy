# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Curriculum stages (Arena: staged / tm_modules)."""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from benchmark.simulators.base import SimulatorAdapter
from benchmark.tasks.modes.base import EpisodeSetup, TaskMode
from benchmark.tasks.modes.parametrized import ParametrizedMode


_DEFAULT_STAGES: List[Dict[str, Any]] = [
    {"difficulty": 0.2, "obstacles": 4, "pedestrians": 1, "episodes": 5},
    {"difficulty": 0.5, "obstacles": 8, "pedestrians": 3, "episodes": 5},
    {"difficulty": 0.8, "obstacles": 14, "pedestrians": 6, "episodes": 5},
]


class StagedMode(TaskMode):
    """Switch obstacle density by curriculum stage over episodes."""

    name = "staged"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(config)
        self.stages: List[Dict[str, Any]] = list(self.config.get("stages") or _DEFAULT_STAGES)
        self._inner = ParametrizedMode({})

    def _stage_for(self, episode_id: int) -> Dict[str, Any]:
        # Walk cumulative episode budgets
        remaining = episode_id
        for stage in self.stages:
            budget = int(stage.get("episodes", 5))
            if remaining < budget:
                return stage
            remaining -= budget
        return self.stages[-1]

    def setup(self, episode_id: int, simulator: SimulatorAdapter) -> EpisodeSetup:
        stage = self._stage_for(episode_id)
        cfg = {
            "difficulty": float(stage.get("difficulty", 0.5)),
            "obstacles": int(stage.get("obstacles", 8)),
            "pedestrians": int(stage.get("pedestrians", 3)),
            "moving_obstacles": int(stage.get("moving_obstacles", 1)),
            "seed": int(self.config.get("seed", 0)),
        }
        self._inner = ParametrizedMode(cfg)
        setup = self._inner.setup(episode_id, simulator)
        setup.info["mode"] = self.name
        setup.info["stage"] = stage
        return setup
