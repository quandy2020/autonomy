# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from pathlib import Path

from benchmark.core.config import load_config
from benchmark.evaluation.arena_evaluation import ArenaEvaluation
from benchmark.evaluation.visualizer import traces_to_svg


def test_two_planner_compare(default_config_path: Path, tmp_path: Path) -> None:
    rates = {}
    for name in ("pure_pursuit", "dwa"):
        cfg = load_config(
            default_config_path,
            overrides=[
                f"planner.name={name}",
                "task.mode=scenario",
                "task.episodes=2",
                "task.timeout_s=6",
                "task.obstacles=4",
                "task.pedestrians=0",
                "task.moving_obstacles=0",
                "task.goal_x=3.0",
            ],
        )
        pipe = ArenaEvaluation(cfg)
        summary = pipe.run()
        rates[name] = summary["efficiency"]["success_rate"]
        traces_to_svg(pipe.traces, tmp_path / f"{name}.svg")
        assert (tmp_path / f"{name}.svg").is_file()
    assert set(rates) == {"pure_pursuit", "dwa"}
