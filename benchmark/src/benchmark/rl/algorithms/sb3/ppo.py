# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Stable-Baselines3 PPO wrapper (optional dependency)."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Dict, Optional

_LOG = logging.getLogger(__name__)


class Sb3Ppo:
    """Thin PPO facade; falls back gracefully when SB3 is missing."""

    name = "sb3_ppo"

    def __init__(self, env: Any, config: Optional[Dict[str, Any]] = None) -> None:
        self.env = env
        self.config = dict(config or {})
        self.model: Any = None
        self._available = False
        try:
            from stable_baselines3 import PPO  # type: ignore
        except ImportError:
            _LOG.warning("stable_baselines3 not installed; Sb3Ppo is a no-op")
            return
        self._available = True
        self.model = PPO(
            "MlpPolicy",
            env,
            learning_rate=float(self.config.get("learning_rate", 3e-4)),
            n_steps=int(self.config.get("n_steps", 256)),
            batch_size=int(self.config.get("batch_size", 64)),
            verbose=int(self.config.get("verbose", 0)),
        )

    def learn(self, total_timesteps: int = 2048) -> Dict[str, Any]:
        if not self._available or self.model is None:
            return {"ok": False, "reason": "sb3_missing"}
        self.model.learn(total_timesteps=total_timesteps)
        return {"ok": True, "timesteps": total_timesteps}

    def save(self, path: str) -> Optional[str]:
        if self.model is None:
            return None
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        self.model.save(str(out))
        return str(out)
