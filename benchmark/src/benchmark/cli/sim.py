# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Simulator smoke / world generation CLI."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import click
import yaml

from benchmark.core.config import package_root
from benchmark.tasks.modes import list_modes
from benchmark.worlds import PRESETS, generate_world


@click.command("generate-world")
@click.argument("name")
@click.option("--seed", type=int, default=0)
@click.option("-o", "--output", default=None, help="Write YAML here")
def generate_world_cmd(name: str, seed: int, output: Optional[str]) -> None:
    """Generate a world preset YAML (Arena dynamic map generation)."""
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
