# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Simple on-policy trainer with curriculum + optional SB3."""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

from benchmark.rl.env import NavEnv
from benchmark.rl.training.curriculum import Curriculum

_LOG = logging.getLogger(__name__)


class SimpleTrainer:
    """Random-search linear policy baseline for smoke training."""

    def __init__(self, env: NavEnv, config: Optional[Dict[str, Any]] = None) -> None:
        self.env = env
        self.config = dict(config or {})
        self.best_return = -1e9
        self.best_action_scale = np.array([0.3, 0.8], dtype=np.float32)
        self.best_weight = np.zeros(
            (self.env.observation_space_shape[0], 2), dtype=np.float32
        )
        self.curriculum: Optional[Curriculum] = None
        if self.config.get("curriculum"):
            self.curriculum = Curriculum(
                start=float(self.config.get("curriculum_start", 0.1)),
                end=float(self.config.get("curriculum_end", 0.9)),
                total_episodes=int(self.config.get("episodes", 50)),
            )

    def train(self, total_episodes: int = 50) -> Dict[str, Any]:
        history = []
        rng = np.random.default_rng(int(self.config.get("seed", 0)))
        if self.curriculum:
            self.curriculum.total_episodes = total_episodes
        for ep in range(total_episodes):
            if self.curriculum:
                # Rebuild env mode difficulty each episode
                cfg = self.curriculum.env_config(ep, self.config)
                self.env.mode = __import__(
                    "benchmark.tasks.modes", fromlist=["create_mode"]
                ).create_mode(cfg["mode"], cfg.get("mode_config") or {})
            w = rng.normal(0, 0.05, size=(self.env.observation_space_shape[0], 2)).astype(
                np.float32
            )
            scale = self.best_action_scale * (0.8 + 0.4 * rng.random(2)).astype(np.float32)
            obs, _ = self.env.reset(seed=ep)
            total = 0.0
            for _ in range(self.env.max_steps):
                logits = np.tanh(obs @ w)
                action = scale * logits
                obs, reward, terminated, truncated, _ = self.env.step(action)
                total += reward
                if terminated or truncated:
                    break
            history.append(
                {
                    "episode": ep,
                    "return": total,
                    "difficulty": self.curriculum.difficulty(ep) if self.curriculum else None,
                }
            )
            if total > self.best_return:
                self.best_return = total
                self.best_action_scale = scale
                self.best_weight = w
                _LOG.info("new best return=%.2f @ ep=%s", total, ep)
        return {
            "episodes": total_episodes,
            "best_return": self.best_return,
            "history": history[-20:],
            "curriculum": bool(self.curriculum),
        }

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(
                {
                    "best_return": self.best_return,
                    "scale": self.best_action_scale.tolist(),
                    "weight_shape": list(self.best_weight.shape),
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        np.savez(
            path.with_suffix(".npz"),
            weight=self.best_weight,
            scale=self.best_action_scale,
        )


def train_with_sb3(config: Dict[str, Any], checkpoint_dir: Path) -> Dict[str, Any]:
    """Train PPO via stable-baselines3 when available."""
    try:
        from stable_baselines3 import PPO
        import gymnasium as gym
        from gymnasium import spaces
    except ImportError as exc:
        raise ImportError("Install with: pip install -e '.[train]'") from exc

    class GymNav(gym.Env):
        def __init__(self) -> None:
            super().__init__()
            self._env = NavEnv(config)
            self.observation_space = spaces.Box(
                low=-np.inf, high=np.inf, shape=self._env.observation_space_shape, dtype=np.float32
            )
            self.action_space = spaces.Box(
                low=self._env.action_space_low, high=self._env.action_space_high, dtype=np.float32
            )

        def reset(self, *, seed=None, options=None):  # type: ignore[no-untyped-def]
            del options
            return self._env.reset(seed=seed)

        def step(self, action):  # type: ignore[no-untyped-def]
            return self._env.step(action)

        def close(self) -> None:
            self._env.close()

    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    env = GymNav()
    model = PPO("MlpPolicy", env, verbose=0)
    steps = int(config.get("total_timesteps", 10_000))
    model.learn(total_timesteps=steps)
    out = checkpoint_dir / "ppo_nav"
    model.save(str(out))
    env.close()
    return {"backend": "sb3_ppo", "checkpoint": str(out), "timesteps": steps}
