# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Arena task modes (robots / modules)."""

from __future__ import annotations

from typing import Any, Dict, Optional

from benchmark.tasks.modes.base import EpisodeSetup, TaskMode
from benchmark.tasks.modes.benchmark import BenchmarkMode
from benchmark.tasks.modes.dynamic import DynamicMode
from benchmark.tasks.modes.explore import ExploreMode
from benchmark.tasks.modes.guided import GuidedMode
from benchmark.tasks.modes.parametrized import ParametrizedMode
from benchmark.tasks.modes.random import RandomMode
from benchmark.tasks.modes.scenario import ScenarioMode
from benchmark.tasks.modes.staged import StagedMode
from benchmark.tasks.modes.zones import ZonesMode

_MODES = {
    "scenario": ScenarioMode,
    "random": RandomMode,
    "parametrized": ParametrizedMode,
    "guided": GuidedMode,
    "explore": ExploreMode,
    "zones": ZonesMode,
    "staged": StagedMode,
    "dynamic": DynamicMode,
    "benchmark": BenchmarkMode,
}


def create_mode(name: str, config: Optional[Dict[str, Any]] = None) -> TaskMode:
    key = (name or "scenario").strip().lower()
    cls = _MODES.get(key)
    if cls is None:
        known = ", ".join(sorted(_MODES))
        raise ValueError(f"unknown task mode {name!r}; known: {known}")
    return cls(config)


def list_modes() -> list[str]:
    return sorted(_MODES)


__all__ = [
    "BenchmarkMode",
    "DynamicMode",
    "EpisodeSetup",
    "ExploreMode",
    "GuidedMode",
    "ParametrizedMode",
    "RandomMode",
    "ScenarioMode",
    "StagedMode",
    "TaskMode",
    "ZonesMode",
    "create_mode",
    "list_modes",
]
