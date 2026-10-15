# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Difficulty curriculum for Arena-style training."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict


@dataclass
class Curriculum:
    """Maps training progress → parametrized task difficulty."""

    start: float = 0.1
    end: float = 0.9
    total_episodes: int = 100

    def difficulty(self, episode: int) -> float:
        if self.total_episodes <= 1:
            return self.end
        t = max(0.0, min(1.0, episode / float(self.total_episodes - 1)))
        return self.start + t * (self.end - self.start)

    def env_config(self, episode: int, base: Dict[str, Any] | None = None) -> Dict[str, Any]:
        cfg = dict(base or {})
        d = self.difficulty(episode)
        cfg["mode"] = "parametrized"
        mode_cfg = dict(cfg.get("mode_config") or {})
        mode_cfg["difficulty"] = d
        mode_cfg["seed"] = int(cfg.get("seed", 0)) + episode
        cfg["mode_config"] = mode_cfg
        return cfg
