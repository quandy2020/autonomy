# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from pathlib import Path

from benchmark.core.config import load_config
from benchmark.core.types import EpisodeStatus
from benchmark.tasks.manager import run_task


def test_slam_smoke(default_config_path: Path) -> None:
    cfg = load_config(
        default_config_path,
        overrides=[
            "task.name=slam",
            "task.episodes=1",
            "task.timeout_steps=80",
            "task.obstacles=4",
            "task.pedestrians=0",
            "task.goal_x=3.0",
            "planner.name=pure_pursuit",
        ],
    )
    results = run_task(cfg)
    assert len(results) == 1
    assert "ate" in results[0].metrics
    assert results[0].info.get("stub") is False


def test_perception_smoke(default_config_path: Path) -> None:
    cfg = load_config(
        default_config_path,
        overrides=[
            "task.name=perception",
            "task.episodes=2",
            "task.obstacles=8",
        ],
    )
    results = run_task(cfg)
    assert len(results) == 2
    assert "precision" in results[0].metrics
    assert "recall" in results[0].metrics


def test_hil_twin_smoke(default_config_path: Path) -> None:
    cfg = load_config(
        default_config_path,
        overrides=[
            "task.name=hil",
            "task.timeout_s=5",
            "task.use_sim_twin=true",
            "task.obstacles=0",
            "task.goal_x=2.0",
            "planner.name=pure_pursuit",
        ],
    )
    results = run_task(cfg)
    assert len(results) == 1
    assert results[0].status in (
        EpisodeStatus.GOAL_REACHED,
        EpisodeStatus.TIMEOUT,
        EpisodeStatus.COLLISION,
        EpisodeStatus.ABORTED,
    )
    assert results[0].info.get("stub") is False
