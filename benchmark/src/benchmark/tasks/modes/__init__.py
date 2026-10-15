# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Arena task modes: scenario / random / parametrized / guided / explore."""

from __future__ import annotations

from typing import Any, Dict, Optional

from benchmark.tasks.modes.base import EpisodeSetup, TaskMode
from benchmark.tasks.modes.explore import ExploreMode
from benchmark.tasks.modes.guided import GuidedMode
from benchmark.tasks.modes.parametrized import ParametrizedMode
from benchmark.tasks.modes.random import RandomMode
from benchmark.tasks.modes.scenario import ScenarioMode

_MODES = {
    "scenario": ScenarioMode,
    "random": RandomMode,
    "parametrized": ParametrizedMode,
    "guided": GuidedMode,
    "explore": ExploreMode,
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
    "EpisodeSetup",
    "ExploreMode",
    "GuidedMode",
    "ParametrizedMode",
    "RandomMode",
    "ScenarioMode",
    "TaskMode",
    "create_mode",
    "list_modes",
]
