# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from pathlib import Path

from benchmark.common.config import load_config
from benchmark.common.types import EpisodeStatus
from benchmark.tasks.manager import run_task


def test_navigate_smoke(default_config_path: Path) -> None:
    cfg = load_config(
        default_config_path,
        overrides=[
            "task.episodes=1",
            "task.timeout_s=8",
            "task.goal_tolerance_m=0.5",
            "task.obstacles=4",
            "task.pedestrians=0",
            "task.moving_obstacles=0",
            "task.mode=scenario",
            "task.goal_x=3.0",
            "task.goal_y=0.0",
        ],
    )
    results = run_task(cfg)
    assert len(results) == 1
    assert results[0].status in (
        EpisodeStatus.GOAL_REACHED,
        EpisodeStatus.TIMEOUT,
        EpisodeStatus.SUCCESS,
        EpisodeStatus.COLLISION,
    )
    assert results[0].steps >= 1
