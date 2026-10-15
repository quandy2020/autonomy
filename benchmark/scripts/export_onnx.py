#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Train a quick policy and export ONNX/npz for PolicyPlanner."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from benchmark.rl.deployment.export_onnx import export_linear_policy  # noqa: E402
from benchmark.rl.env import NavEnv  # noqa: E402
from benchmark.rl.training.trainer import SimpleTrainer  # noqa: E402


def main() -> None:
    out = ROOT / "data" / "checkpoints" / "policy"
    env = NavEnv({"mode": "parametrized", "max_steps": 120, "mode_config": {"difficulty": 0.3}})
    trainer = SimpleTrainer(env, {"curriculum": True, "episodes": 20, "seed": 0})
    summary = trainer.train(total_episodes=20)
    meta = export_linear_policy(trainer.best_weight, trainer.best_action_scale, out)
    env.close()
    print("best_return", summary["best_return"])
    print("export", meta)


if __name__ == "__main__":
    main()
