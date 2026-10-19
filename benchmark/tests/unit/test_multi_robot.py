# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from benchmark.tasks.multi_robot import run_multi_robot


def test_multi_robot_batch() -> None:
    results = run_multi_robot(n_robots=2, timeout_s=5.0, planner_config={"name": "dwa"})
    assert len(results) == 2
    assert results[0].info.get("robot_id") == 0
