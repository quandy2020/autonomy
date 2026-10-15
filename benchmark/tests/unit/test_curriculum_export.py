# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from pathlib import Path

from benchmark.rl.deployment.export_onnx import export_linear_policy
from benchmark.rl.env import NavEnv
from benchmark.rl.training.curriculum import Curriculum
from benchmark.rl.training.trainer import SimpleTrainer


def test_curriculum_difficulty() -> None:
    cur = Curriculum(start=0.0, end=1.0, total_episodes=5)
    assert cur.difficulty(0) == 0.0
    assert cur.difficulty(4) == 1.0
    cfg = cur.env_config(2, {"seed": 1})
    assert cfg["mode"] == "parametrized"
    assert 0.0 < cfg["mode_config"]["difficulty"] < 1.0


def test_train_and_export(tmp_path: Path) -> None:
    env = NavEnv({"mode": "scenario", "max_steps": 20, "mode_config": {"obstacles": 2}})
    trainer = SimpleTrainer(env, {"curriculum": True, "episodes": 3, "seed": 0})
    summary = trainer.train(total_episodes=3)
    assert summary["curriculum"] is True
    meta = export_linear_policy(
        trainer.best_weight,
        trainer.best_action_scale,
        tmp_path / "policy",
    )
    assert "npz" in meta or "onnx" in meta
    env.close()
