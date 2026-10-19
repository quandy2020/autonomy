# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""CLI entry: ``benchmark`` — Arena-style one-stop task manager."""

from __future__ import annotations

import json
from pathlib import Path

import click

from benchmark import __version__
from benchmark.bridge.autonomy_control import find_controller_binary
from benchmark.common.config import load_config, package_root
from benchmark.common.logging import setup_logging
from benchmark.common.registry import get_registry
from benchmark.dashboard.cli import dashboard_cmd
from benchmark.evaluation.cli import eval_cmd
from benchmark.evaluation.evaluator import Evaluator
from benchmark.evaluation.report import write_report
from benchmark.simulators.cli import generate_world_cmd, modes_cmd
from benchmark.simulators.humans import list_human_models
from benchmark.tasks.manager import run_task
from benchmark.tasks.modes import list_modes
from benchmark.tasks.multi_robot import run_multi_robot
from benchmark.tasks.planning.factory import list_planners


def _default_config() -> str:
    return str(package_root() / "configs" / "default.yaml")


@click.group()
@click.version_option(__version__, prog_name="benchmark")
@click.option("--log-level", default="INFO", show_default=True)
@click.pass_context
def main(ctx: click.Context, log_level: str) -> None:
    """Autonomy benchmark — Arena-like navigation / SLAM / data / HIL manager."""
    setup_logging(log_level)
    ctx.ensure_object(dict)


@main.command("list")
def list_cmd() -> None:
    """List registered simulators / tasks / planners / modes."""
    hub = get_registry()
    hub.ensure_builtins()
    click.echo("simulators: " + ", ".join(hub.simulators.names()))
    click.echo("tasks:      " + ", ".join(hub.tasks.names()))
    click.echo("planners:   " + ", ".join(list_planners()))
    click.echo("modes:      " + ", ".join(list_modes()))
    click.echo("humans:     " + ", ".join(list_human_models()))


@main.command("multi-robot")
@click.option("--robots", type=int, default=2, show_default=True)
@click.option("--planner", default="pure_pursuit", show_default=True)
@click.option("--timeout", type=float, default=20.0, show_default=True)
def multi_robot_cmd(robots: int, planner: str, timeout: float) -> None:
    """Run Arena-style multi-robot batch on kinematic worlds."""
    results = run_multi_robot(
        n_robots=robots,
        planner_config={"name": planner},
        timeout_s=timeout,
    )
    summary = Evaluator().summarize(results)
    click.echo(json.dumps(summary, indent=2))


@main.command("run")
@click.option("-c", "--config", "config_path", default=None, help="Experiment YAML")
@click.option("-t", "--task", "task_name", default=None, help="Override task.name")
@click.option("--mode", "task_mode", default=None, help="Arena task mode")
@click.option("--set", "overrides", multiple=True, help="Dotted override key=value")
@click.option("--report-dir", default=None, help="Write eval report here")
def run_cmd(
    config_path: str | None,
    task_name: str | None,
    task_mode: str | None,
    overrides: list[str],
    report_dir: str | None,
) -> None:
    """Run a configured task (navigate / slam / collect / …)."""
    path = config_path or _default_config()
    ovr = list(overrides)
    if task_name:
        ovr.append(f"task.name={task_name}")
    if task_mode:
        ovr.append(f"task.mode={task_mode}")
    cfg = load_config(path, overrides=ovr)
    click.echo(
        f"config={cfg.path} task={cfg.task_name} sim={cfg.simulator_name} "
        f"mode={cfg.section('task').get('mode', 'scenario')}"
    )
    results = run_task(cfg)
    summary = Evaluator().summarize(results)
    click.echo(json.dumps(summary, indent=2))
    out = Path(report_dir) if report_dir else package_root() / "data" / "eval_results"
    write_report(summary, out, name=f"{cfg.task_name}_summary")
    click.echo(f"report → {out}")


@main.command("doctor")
def doctor_cmd() -> None:
    """Check sibling module importability."""
    mods = {
        "autosim": "autosim",
        "automsgs": "automsgs",
        "autolink": "autolink",
        "numpy": "numpy",
        "yaml": "yaml",
        "onnxruntime": "onnxruntime",
        "flask": "flask",
    }
    for label, mod in mods.items():
        try:
            __import__(mod)
            click.echo(f"[ok] {label}")
        except Exception as exc:
            click.echo(f"[--] {label}: {exc}")
    ctrl = find_controller_binary()
    click.echo(f"[{'ok' if ctrl else '--'}] autonomy.control: {ctrl or 'not found'}")
    from benchmark.common.registry import get_registry
    from benchmark.simulators.humans import list_human_models
    from benchmark.tasks.modes import list_modes
    from benchmark.tasks.planning.factory import list_planners

    hub = get_registry()
    hub.ensure_builtins()
    click.echo("simulators: " + ", ".join(hub.simulators.names()))
    click.echo("modes:      " + ", ".join(list_modes()))
    click.echo("humans:     " + ", ".join(list_human_models()))
    click.echo("planners:   " + ", ".join(list_planners()))


main.add_command(eval_cmd)
main.add_command(dashboard_cmd)
main.add_command(generate_world_cmd)
main.add_command(modes_cmd)
