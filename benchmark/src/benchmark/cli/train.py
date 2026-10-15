# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""DRL training / export / Optuna CLI."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import click

from benchmark.core.config import package_root
from benchmark.rl.deployment.export_onnx import export_linear_policy
from benchmark.rl.env import NavEnv
from benchmark.rl.training.trainer import SimpleTrainer, train_with_sb3


@click.command("train")
@click.option("--backend", type=click.Choice(["simple", "sb3"]), default="simple")
@click.option("--episodes", type=int, default=30, show_default=True)
@click.option("--timesteps", type=int, default=10000, show_default=True)
@click.option("--mode", default="random", show_default=True)
@click.option("--curriculum/--no-curriculum", default=False)
@click.option("--export", "export_path", default=None, help="Export best policy (.onnx/.npz)")
@click.option("--checkpoint-dir", default=None)
def train_cmd(
    backend: str,
    episodes: int,
    timesteps: int,
    mode: str,
    curriculum: bool,
    export_path: Optional[str],
    checkpoint_dir: Optional[str],
) -> None:
    """Train a navigational policy."""
    ckpt = Path(checkpoint_dir) if checkpoint_dir else package_root() / "data" / "checkpoints"
    cfg = {
        "mode": mode,
        "max_steps": 200,
        "total_timesteps": timesteps,
        "seed": 0,
        "episodes": episodes,
        "curriculum": curriculum,
    }
    if backend == "sb3":
        summary = train_with_sb3(cfg, ckpt)
    else:
        env = NavEnv(cfg)
        trainer = SimpleTrainer(env, cfg)
        summary = trainer.train(total_episodes=episodes)
        trainer.save(ckpt / "simple_policy.json")
        if export_path:
            meta = export_linear_policy(
                trainer.best_weight,
                trainer.best_action_scale,
                Path(export_path),
            )
            summary["export"] = meta
        env.close()
    click.echo(json.dumps(summary, indent=2))
    click.echo(f"checkpoints → {ckpt}")


@click.command("tune")
@click.option("--trials", type=int, default=8, show_default=True)
@click.option("--episodes", type=int, default=4, show_default=True)
def tune_cmd(trials: int, episodes: int) -> None:
    """Optuna search over NavEnv hyperparameters."""
    from benchmark.rl.tuning.optuna_tuner import tune

    summary = tune(n_trials=trials, episodes_per_trial=episodes)
    click.echo(json.dumps(summary, indent=2))
