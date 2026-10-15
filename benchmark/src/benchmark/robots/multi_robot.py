# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Multi-robot simultaneous episodes (Arena multi-robot)."""

from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional

from benchmark.core.types import EpisodeResult
from benchmark.planning.factory import create_planner
from benchmark.simulators.fake.simulator import FakeSimulator
from benchmark.tasks.episode_runner import run_episode, trace_to_result
from benchmark.tasks.modes import create_mode

_LOG = logging.getLogger(__name__)


def run_multi_robot(
    *,
    n_robots: int = 2,
    planner_config: Optional[Dict[str, Any]] = None,
    mode_name: str = "scenario",
    mode_config: Optional[Dict[str, Any]] = None,
    timeout_s: float = 30.0,
    seed: int = 0,
) -> List[EpisodeResult]:
    """Run ``n_robots`` independent kinematic worlds in one batch."""
    results: List[EpisodeResult] = []
    planner_cfg = dict(planner_config or {"name": "pure_pursuit"})
    base_mode = dict(mode_config or {})
    for i in range(max(1, n_robots)):
        sim = FakeSimulator({"width": 20, "height": 20, "rate_hz": 10})
        sim.start()
        cfg = {
            **base_mode,
            "seed": seed + i,
            "start_x": float(base_mode.get("start_x", 0.0)) + 0.8 * i,
            "start_y": float(base_mode.get("start_y", 0.0)) + 0.4 * i,
            "goal_x": float(base_mode.get("goal_x", 5.0)),
            "goal_y": float(base_mode.get("goal_y", 0.0)) - 0.3 * i,
            "obstacles": int(base_mode.get("obstacles", 6)),
            "pedestrians": int(base_mode.get("pedestrians", 1)),
            "moving_obstacles": int(base_mode.get("moving_obstacles", 0)),
        }
        mode = create_mode(mode_name if mode_name != "random" else "scenario", cfg)
        planner = create_planner(planner_cfg)
        trace = run_episode(
            sim,
            mode,
            planner,
            episode_id=i,
            timeout_s=timeout_s,
            goal_tolerance=0.4,
        )
        result = trace_to_result(trace)
        result.info["robot_id"] = i
        _LOG.info("robot %s → %s", i, result.status.value)
        results.append(result)
        sim.stop()
    return results
