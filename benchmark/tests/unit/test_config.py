# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from pathlib import Path

from benchmark.common.config import load_config


def test_load_default(default_config_path: Path) -> None:
    cfg = load_config(default_config_path)
    assert cfg.task_name == "navigate"
    assert cfg.simulator_name == "fake"


def test_overrides(default_config_path: Path) -> None:
    cfg = load_config(default_config_path, overrides=["task.name=slam", "experiment.seed=7"])
    assert cfg.task_name == "slam"
    assert cfg.experiment["seed"] == 7
