# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Simulator smoke / world generation CLI."""

from __future__ import annotations

from pathlib import Path

import click
import yaml

from benchmark.simulators.worlds import PRESETS, generate_world
from benchmark.simulators.worlds.generators.arena_gen import generate_from_prompt, parse_prompt
from benchmark.tasks.modes import list_modes


@click.command("generate-world")
@click.argument("name", required=False, default=None)
@click.option("--seed", type=int, default=0)
@click.option("--prompt", default=None, help="Arena-Gen lite text prompt")
@click.option("-o", "--output", default=None, help="Write YAML here")
def generate_world_cmd(
    name: str | None,
    seed: int,
    prompt: str | None,
    output: str | None,
) -> None:
    """Generate a world preset YAML (Arena dynamic map / Arena-Gen lite)."""
    if prompt:
        data = generate_from_prompt(prompt, seed=seed)
        click.echo(f"# parsed {parse_prompt(prompt)}")
    else:
        if not name:
            raise click.UsageError("provide NAME or --prompt")
        data = generate_world(name, seed=seed)
    text = yaml.safe_dump(data, sort_keys=False)
    if output:
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        click.echo(f"wrote {path}")
    else:
        click.echo(text)


@click.command("modes")
def modes_cmd() -> None:
    """List Arena task modes and world presets."""
    click.echo("task modes: " + ", ".join(list_modes()))
    click.echo("worlds:     " + ", ".join(PRESETS))
