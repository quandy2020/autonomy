# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from benchmark.core.registry import get_registry
from benchmark.planning.base import Planner
from benchmark.planning.classic.autonomy_mppi import AutonomyMppiPlanner
from benchmark.planning.classic.dwa import DwaPlanner
from benchmark.planning.classic.mpc import MpcPlanner
from benchmark.planning.classic.pure_pursuit import PurePursuitPlanner
from benchmark.planning.classic.teb import TebPlanner
from benchmark.planning.factory import create_planner, list_planners
from benchmark.planning.hybrid.switching import SwitchingPlanner
from benchmark.planning.learning.policy import PolicyPlanner, RosnavRlPlanner

_hub = get_registry()
_hub.planners.register("pure_pursuit", PurePursuitPlanner)
_hub.planners.register("dwa", DwaPlanner)
_hub.planners.register("mpc", MpcPlanner)
_hub.planners.register("teb", TebPlanner)
_hub.planners.register("mppi", AutonomyMppiPlanner)
_hub.planners.register("autonomy_mppi", AutonomyMppiPlanner)
_hub.planners.register("nmpc", MpcPlanner)
_hub.planners.register("policy", PolicyPlanner)
_hub.planners.register("rosnav_rl", RosnavRlPlanner)
_hub.planners.register("hybrid", SwitchingPlanner)
_hub.planners.register("switching", SwitchingPlanner)

__all__ = [
    "AutonomyMppiPlanner",
    "DwaPlanner",
    "MpcPlanner",
    "Planner",
    "PolicyPlanner",
    "PurePursuitPlanner",
    "RosnavRlPlanner",
    "SwitchingPlanner",
    "TebPlanner",
    "create_planner",
    "list_planners",
]
