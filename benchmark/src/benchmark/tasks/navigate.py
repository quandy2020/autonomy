# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Goal-directed navigation with Arena task modes."""

from __future__ import annotations

import logging

from benchmark.common.types import EpisodeResult
from benchmark.simulators.factory import create_simulator
from benchmark.tasks.base import Task
from benchmark.tasks.episode_runner import make_planner, run_episode, trace_to_result
from benchmark.tasks.modes import create_mode

_LOG = logging.getLogger(__name__)


class NavigateTask(Task):
    name = "navigate"

    def run(self) -> list[EpisodeResult]:
        sim = self.simulator or create_simulator(self.config)
        episodes = int(self.task_cfg.get("episodes", 1))
        timeout_s = float(self.task_cfg.get("timeout_s", 60))
        tol = float(self.task_cfg.get("goal_tolerance_m", 0.35))
        mode_name = str(self.task_cfg.get("mode", "scenario"))
        mode_cfg = dict(self.task_cfg.get("mode_config") or {})
        # Allow world/task knobs to flow into mode.
        for key in (
            "obstacles",
            "pedestrians",
            "moving_obstacles",
            "difficulty",
            "waypoints",
            "seed",
            "start_x",
            "start_y",
            "goal_x",
            "goal_y",
            "coverage_target",
            "scenario",
            "scenario_file",
            "stages",
            "map",
            "start_zones",
            "goal_zones",
        ):
            if key in self.task_cfg and key not in mode_cfg:
                mode_cfg[key] = self.task_cfg[key]

        mode = create_mode(mode_name, mode_cfg)
        planner = make_planner(self.config.section("planner"))
        results: list[EpisodeResult] = []

        with sim:
            for ep in range(episodes):
                trace = run_episode(
                    sim,
                    mode,
                    planner,
                    episode_id=ep,
                    timeout_s=timeout_s,
                    goal_tolerance=tol,
                )
                result = trace_to_result(trace)
                _LOG.info(
                    "episode %s mode=%s → %s (steps=%s path=%.2f clear=%.2f)",
                    ep,
                    mode_name,
                    result.status.value,
                    result.steps,
                    result.path_length,
                    result.metrics.get("min_clearance", 0.0),
                )
                results.append(result)
        return results
