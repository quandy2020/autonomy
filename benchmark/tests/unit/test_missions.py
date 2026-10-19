# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from benchmark.common.config import load_config, package_root
from benchmark.common.registry import get_registry
from benchmark.tasks.manager import run_task
from benchmark.tasks.missions import MISSION_CATALOG


def test_mission_catalog_ids() -> None:
    ids = {m["id"] for m in MISSION_CATALOG}
    assert ids == {"navigate", "dock", "explore_mapping", "patrol", "teleop"}


def test_missions_registered() -> None:
    hub = get_registry()
    hub.ensure_builtins()
    for m in MISSION_CATALOG:
        assert m["id"] in hub.tasks.names()


def test_dock_and_teleop_smoke() -> None:
    path = package_root() / "configs" / "default.yaml"
    for name in ("dock", "teleop"):
        cfg = load_config(path, overrides=[f"task.name={name}", "task.episodes=1", "task.timeout_s=8"])
        results = run_task(cfg)
        assert len(results) == 1
        assert "control_curves" in results[0].info
        assert results[0].info["control_curves"]["stats"]["samples"] >= 1
