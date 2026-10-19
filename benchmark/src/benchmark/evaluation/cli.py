# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Arena Evaluation CLI helpers."""

from __future__ import annotations

import json
from pathlib import Path

import click

from benchmark.common.config import load_config, package_root
from benchmark.evaluation.arena_evaluation import ArenaEvaluation
from benchmark.evaluation.recorder import save_traces


@click.command("eval")
@click.option("-c", "--config", "config_path", default=None)
@click.option("--episodes", type=int, default=None)
@click.option("--set", "overrides", multiple=True)
@click.option("--report-dir", default=None)
def eval_cmd(
    config_path: str | None,
    episodes: int | None,
    overrides: list[str],
    report_dir: str | None,
) -> None:
    """Run Arena Evaluation metric suite."""
    path = config_path or str(package_root() / "configs" / "default.yaml")
    cfg = load_config(path, overrides=list(overrides))
    pipe = ArenaEvaluation(cfg)
    summary = pipe.run(episodes=episodes)
    out = Path(report_dir) if report_dir else package_root() / "data" / "eval_results"
    pipe.write(out)
    if pipe.traces:
        save_traces(pipe.traces, out / "traces.json")
    click.echo(json.dumps(summary, indent=2))
    click.echo(f"report → {out}")
