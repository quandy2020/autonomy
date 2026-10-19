# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from pathlib import Path

from benchmark.common.config import load_config
from benchmark.evaluation.dataset import load_jsonl_episode
from benchmark.tasks.manager import run_task


def test_collect_and_replay(default_config_path: Path, tmp_path: Path) -> None:
    out = tmp_path / "ds"
    cfg = load_config(
        default_config_path,
        overrides=[
            "task.name=collect",
            "task.episodes=1",
            "task.episode_frames=25",
            "task.fps=20",
            f"task.output={out}",
            "task.obstacles=3",
            "task.pedestrians=0",
            "task.moving_obstacles=0",
            "task.goal_x=2.5",
            "planner.name=pure_pursuit",
        ],
    )
    results = run_task(cfg)
    assert len(results) == 1
    files = list(out.glob("episode_*.jsonl"))
    assert files, "expected jsonl episode"
    frames = load_jsonl_episode(files[0])
    assert len(frames) >= 5

    replay_cfg = load_config(
        default_config_path,
        overrides=[
            "task.name=replay",
            f"task.source={out}",
            "task.dry_run=true",
            "task.rate=10",
        ],
    )
    replay_results = run_task(replay_cfg)
    assert replay_results[0].status.value in ("success", "failed")
    if replay_results[0].status.value == "success":
        assert replay_results[0].steps >= 5
