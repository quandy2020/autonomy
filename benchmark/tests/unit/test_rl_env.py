# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

import numpy as np

from benchmark.rl.env import NavEnv
from benchmark.rl.training.trainer import SimpleTrainer


def test_nav_env_step() -> None:
    env = NavEnv({"mode": "scenario", "max_steps": 20, "mode_config": {"obstacles": 3, "pedestrians": 1}})
    obs, info = env.reset(seed=0)
    assert obs.shape[0] == env.observation_space_shape[0]
    action = np.array([0.2, 0.0], dtype=np.float32)
    obs2, reward, terminated, truncated, step_info = env.step(action)
    assert obs2.shape == obs.shape
    assert isinstance(reward, float)
    assert "reward_parts" in step_info
    env.close()


def test_simple_trainer_smoke() -> None:
    env = NavEnv({"mode": "scenario", "max_steps": 15, "mode_config": {"obstacles": 2, "pedestrians": 0}})
    trainer = SimpleTrainer(env, {"seed": 0})
    summary = trainer.train(total_episodes=3)
    assert summary["episodes"] == 3
    assert "best_return" in summary
    env.close()
