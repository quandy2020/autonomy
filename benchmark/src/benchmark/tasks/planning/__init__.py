# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Local / learning / hybrid planners (flat package)."""

from __future__ import annotations

from benchmark.common.registry import get_registry
from benchmark.tasks.planning.autonomy_mppi import AutonomyMppiPlanner
from benchmark.tasks.planning.base import Planner
from benchmark.tasks.planning.dwa import DwaPlanner
from benchmark.tasks.planning.factory import create_planner, list_planners
from benchmark.tasks.planning.mpc import MpcPlanner
from benchmark.tasks.planning.policy import PolicyPlanner
from benchmark.tasks.planning.pure_pursuit import PurePursuitPlanner
from benchmark.tasks.planning.switching import SwitchingPlanner
from benchmark.tasks.planning.teb import TebPlanner

_hub = get_registry()
_hub.planners.register("pure_pursuit", PurePursuitPlanner)
_hub.planners.register("dwa", DwaPlanner)
_hub.planners.register("mpc", MpcPlanner)
_hub.planners.register("teb", TebPlanner)
_hub.planners.register("mppi", AutonomyMppiPlanner)
_hub.planners.register("autonomy_mppi", AutonomyMppiPlanner)
_hub.planners.register("nmpc", MpcPlanner)
_hub.planners.register("policy", PolicyPlanner)
_hub.planners.register("hybrid", SwitchingPlanner)
_hub.planners.register("switching", SwitchingPlanner)

__all__ = [
    "AutonomyMppiPlanner",
    "DwaPlanner",
    "MpcPlanner",
    "Planner",
    "PolicyPlanner",
    "PurePursuitPlanner",
    "SwitchingPlanner",
    "TebPlanner",
    "create_planner",
    "list_planners",
]
