# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from pathlib import Path

from benchmark.common.config import load_config, package_root
from benchmark.common.registry import get_registry
from benchmark.evaluation.arena_evaluation import ArenaEvaluation
from benchmark.simulators.humans import create_human_model, list_human_models
from benchmark.tasks.planning.factory import create_planner, list_planners
from benchmark.tasks.modes import create_mode, list_modes
from benchmark.tasks.scenario_loader import load_scenario
from benchmark.simulators.worlds.generators.dynamic_map import generate_dynamic_layout


def test_arena_simulators_registered() -> None:
    hub = get_registry()
    hub.ensure_builtins()
    names = set(hub.simulators.names())
    assert {"fake", "flatland", "habitat", "isaac", "hil", "gazebo", "unity"} <= names


def test_arena_task_modes() -> None:
    expected = {
        "scenario",
        "random",
        "parametrized",
        "guided",
        "explore",
        "zones",
        "staged",
        "dynamic",
        "benchmark",
    }
    assert expected <= set(list_modes())


def test_arena_human_models() -> None:
    expected = {
        "social_force",
        "crowd_flow",
        "orca",
        "rvo2",
        "evacuation",
        "spinny",
        "hunav",
        "pysocialforce",
        "deep_social",
        "lightsfm",
        "pedsim",
    }
    assert expected <= set(list_human_models())
    for name in ("orca", "hunav", "evacuation", "spinny", "pysocialforce"):
        model = create_human_model({"name": name})
        assert model.name == name


def test_arena_planners() -> None:
    for name in ("cadrl", "crowdnav", "rlca", "sarl", "applr", "dragon", "trail", "hybrid", "dwa"):
        assert name in list_planners()
        p = create_planner({"name": name})
        assert p is not None


def test_scenario_json_loader() -> None:
    path = package_root() / "configs" / "scenarios" / "demo_challenge.json"
    setup = load_scenario(path)
    assert setup.goal.x == 4.0
    assert setup.obstacles == 6
    assert len(setup.waypoints) == 2


def test_dynamic_layouts() -> None:
    for kind in ("barn", "canteen", "office", "industrial", "maze"):
        disks = generate_dynamic_layout(seed=1, kind=kind, count=12)
        assert len(disks) > 0


def test_benchmark_mode_and_social_metrics() -> None:
    cfg = load_config(
        package_root() / "configs" / "default.yaml",
        overrides=[
            "task.mode=benchmark",
            "task.episodes=3",
            "task.timeout_s=6",
            "task.obstacles=4",
            "task.pedestrians=2",
            "task.moving_obstacles=0",
            "humans.name=orca",
            "planner.name=dwa",
        ],
    )
    # Provide compact challenge stages
    cfg.raw.setdefault("task", {})["mode_config"] = {
        "stages": [
            {"mode": "random", "episodes": 2, "config": {"obstacles": 4, "pedestrians": 2, "moving_obstacles": 0}},
            {"mode": "parametrized", "episodes": 1, "config": {"difficulty": 0.3}},
        ]
    }
    summary = ArenaEvaluation(cfg).run(episodes=3)
    assert summary["count"] == 3
    assert "social" in summary
    assert "time_in_private_zone_s" in summary["social"]


def test_zones_and_staged_setup() -> None:
    from benchmark.simulators.fake.simulator import FakeSimulator

    sim = FakeSimulator({"rate_hz": 10})
    sim.start()
    z = create_mode("zones", {"obstacles": 3, "pedestrians": 1, "moving_obstacles": 0})
    setup = z.setup(0, sim)
    assert setup.info["mode"] == "zones"
    s = create_mode("staged", {})
    setup2 = s.setup(0, sim)
    assert setup2.info["mode"] == "staged"
    sim.stop()
