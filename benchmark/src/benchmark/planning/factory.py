# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Create planners by name (Arena planner registry)."""

from __future__ import annotations

from typing import Any, Dict

from benchmark.planning.base import Planner
from benchmark.planning.classic.autonomy_mppi import AutonomyMppiPlanner
from benchmark.planning.classic.dwa import DwaPlanner
from benchmark.planning.classic.mpc import MpcPlanner
from benchmark.planning.classic.pure_pursuit import PurePursuitPlanner
from benchmark.planning.classic.teb import TebPlanner
from benchmark.planning.hybrid.switching import SwitchingPlanner
from benchmark.planning.learning.policy import PolicyPlanner, RosnavRlPlanner

_FACTORIES = {
    "pure_pursuit": PurePursuitPlanner,
    "dwa": DwaPlanner,
    "mpc": MpcPlanner,
    "teb": TebPlanner,
    "mppi": AutonomyMppiPlanner,
    "autonomy_mppi": AutonomyMppiPlanner,
    "nmpc": MpcPlanner,
    "policy": PolicyPlanner,
    "rosnav_rl": RosnavRlPlanner,
    "hybrid": SwitchingPlanner,
    "switching": SwitchingPlanner,
}


def create_planner(config: Dict[str, Any]) -> Planner:
    name = str(config.get("name") or "pure_pursuit").strip().lower()
    factory = _FACTORIES.get(name)
    if factory is None:
        known = ", ".join(sorted(_FACTORIES))
        raise ValueError(f"unknown planner {name!r}; known: {known}")
    return factory(config)


def list_planners() -> list[str]:
    return sorted(_FACTORIES)
