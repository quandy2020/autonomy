# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""RL algorithm backends (SB3 / custom)."""

from __future__ import annotations

from typing import Any, Dict, Optional


def create_algorithm(name: str, env: Any, config: Optional[Dict[str, Any]] = None) -> Any:
    key = (name or "simple").strip().lower()
    if key in ("ppo", "sb3", "sb3_ppo"):
        from benchmark.rl.algorithms.sb3.ppo import Sb3Ppo

        return Sb3Ppo(env, config)
    from benchmark.rl.training.trainer import SimpleTrainer

    return SimpleTrainer(env, config)
