# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Create planners by name (Arena planner registry)."""

from __future__ import annotations

from typing import Any

from benchmark.tasks.planning.arena_rl import (
    ApplrPlanner,
    CadrlPlanner,
    CrowdNavPlanner,
    DragonPlanner,
    DrlvoPlanner,
    RlcaPlanner,
    SarlPlanner,
    SarlStarPlanner,
    TrailPlanner,
)
from benchmark.tasks.planning.autonomy_mppi import AutonomyMppiPlanner
from benchmark.tasks.planning.base import Planner
from benchmark.tasks.planning.dwa import DwaPlanner
from benchmark.tasks.planning.mpc import MpcPlanner
from benchmark.tasks.planning.policy import PolicyPlanner
from benchmark.tasks.planning.pure_pursuit import PurePursuitPlanner
from benchmark.tasks.planning.switching import SwitchingPlanner
from benchmark.tasks.planning.teb import TebPlanner

_FACTORIES = {
    # Classic / local
    "pure_pursuit": PurePursuitPlanner,
    "dwa": DwaPlanner,
    "mpc": MpcPlanner,
    "teb": TebPlanner,
    "mppi": AutonomyMppiPlanner,
    "autonomy_mppi": AutonomyMppiPlanner,
    "nmpc": MpcPlanner,
    # Learning / DRL names (ONNX or reactive; no ROS runtime)
    "policy": PolicyPlanner,
    "cadrl": CadrlPlanner,
    "crowdnav": CrowdNavPlanner,
    "rlca": RlcaPlanner,
    "sarl": SarlPlanner,
    "sarl_star": SarlStarPlanner,
    "applr": ApplrPlanner,
    "dragon": DragonPlanner,
    "trail": TrailPlanner,
    "drlvo": DrlvoPlanner,
    # Hybrid switching
    "hybrid": SwitchingPlanner,
    "switching": SwitchingPlanner,
}


def create_planner(config: dict[str, Any]) -> Planner:
    name = str(config.get("name") or "pure_pursuit").strip().lower()
    factory = _FACTORIES.get(name)
    if factory is None:
        known = ", ".join(sorted(_FACTORIES))
        raise ValueError(f"unknown planner {name!r}; known: {known}")
    return factory(config)


def list_planners() -> list[str]:
    return sorted(_FACTORIES)
