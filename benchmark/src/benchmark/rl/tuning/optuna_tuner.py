# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Optuna hyperparameter search over NavEnv reward / max_steps / difficulty."""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

_LOG = logging.getLogger(__name__)


def tune(
    *,
    n_trials: int = 10,
    episodes_per_trial: int = 5,
    seed: int = 0,
    storage: Optional[str] = None,
) -> Dict[str, Any]:
    try:
        import optuna
    except ImportError as exc:
        raise ImportError("Install with: pip install -e '.[train]' (optuna)") from exc

    from benchmark.rl.env import NavEnv
    from benchmark.rl.training.trainer import SimpleTrainer

    def objective(trial: "optuna.Trial") -> float:  # type: ignore[name-defined]
        max_steps = trial.suggest_int("max_steps", 80, 250)
        difficulty = trial.suggest_float("difficulty", 0.1, 0.8)
        cfg = {
            "mode": "parametrized",
            "mode_config": {"difficulty": difficulty, "seed": seed},
            "max_steps": max_steps,
            "seed": seed + trial.number,
        }
        env = NavEnv(cfg)
        trainer = SimpleTrainer(env, cfg)
        summary = trainer.train(total_episodes=episodes_per_trial)
        env.close()
        return float(summary["best_return"])

    study = optuna.create_study(direction="maximize", storage=storage, study_name="benchmark_nav")
    study.optimize(objective, n_trials=n_trials)
    best = {
        "best_value": study.best_value,
        "best_params": study.best_params,
        "n_trials": n_trials,
    }
    _LOG.info("optuna best=%s params=%s", best["best_value"], best["best_params"])
    return best
