# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Composite challenge mode (Arena: benchmark / tm_modules)."""

from __future__ import annotations

from typing import Any

from benchmark.simulators.base import SimulatorAdapter
from benchmark.tasks.modes.base import EpisodeSetup, TaskMode
from benchmark.tasks.modes.explore import ExploreMode
from benchmark.tasks.modes.guided import GuidedMode
from benchmark.tasks.modes.parametrized import ParametrizedMode
from benchmark.tasks.modes.random import RandomMode
from benchmark.tasks.modes.scenario import ScenarioMode
from benchmark.tasks.modes.staged import StagedMode
from benchmark.tasks.modes.zones import ZonesMode

_INNER = {
    "scenario": ScenarioMode,
    "random": RandomMode,
    "parametrized": ParametrizedMode,
    "guided": GuidedMode,
    "explore": ExploreMode,
    "zones": ZonesMode,
    "staged": StagedMode,
}


class BenchmarkMode(TaskMode):
    """Stage a sequence of modes from a challenge YAML (Arena competition).

    Config shape::

        stages:
          - mode: random
            episodes: 3
            config: {obstacles: 6}
          - mode: guided
            episodes: 2
            config: {}
    """

    name = "benchmark"

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        super().__init__(config)
        self.stages: list[dict[str, Any]] = list(
            self.config.get("stages")
            or [
                {"mode": "random", "episodes": 3, "config": {"obstacles": 6, "pedestrians": 2}},
                {"mode": "parametrized", "episodes": 3, "config": {"difficulty": 0.6}},
                {"mode": "guided", "episodes": 2, "config": {}},
            ]
        )
        self._cache: dict[str, TaskMode] = {}

    def _resolve(self, episode_id: int) -> tuple[TaskMode, dict[str, Any], int]:
        remaining = episode_id
        for stage in self.stages:
            budget = int(stage.get("episodes", 1))
            if remaining < budget:
                mname = str(stage.get("mode", "random")).lower()
                cls = _INNER.get(mname, RandomMode)
                key = f"{mname}:{id(stage)}"
                if key not in self._cache:
                    self._cache[key] = cls(dict(stage.get("config") or {}))
                return self._cache[key], stage, remaining
            remaining -= budget
        last = self.stages[-1]
        mname = str(last.get("mode", "random")).lower()
        cls = _INNER.get(mname, RandomMode)
        return cls(dict(last.get("config") or {})), last, remaining

    def setup(self, episode_id: int, simulator: SimulatorAdapter) -> EpisodeSetup:
        mode, stage, local_ep = self._resolve(episode_id)
        setup = mode.setup(local_ep, simulator)
        setup.info["mode"] = self.name
        setup.info["inner_mode"] = stage.get("mode")
        setup.info["challenge_stage"] = stage
        return setup
