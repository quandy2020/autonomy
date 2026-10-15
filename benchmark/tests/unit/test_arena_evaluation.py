# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from pathlib import Path

from benchmark.core.config import load_config
from benchmark.evaluation.arena_evaluation import ArenaEvaluation


def test_arena_eval_smoke(default_config_path: Path, tmp_path: Path) -> None:
    cfg = load_config(
        default_config_path,
        overrides=[
            "task.mode=scenario",
            "task.episodes=2",
            "task.timeout_s=5",
            "task.obstacles=5",
            "task.pedestrians=2",
            "planner.name=dwa",
        ],
    )
    pipe = ArenaEvaluation(cfg)
    summary = pipe.run()
    assert summary["count"] == 2
    assert "safety" in summary and "efficiency" in summary and "smoothness" in summary
    path = pipe.write(tmp_path)
    assert path.exists()
