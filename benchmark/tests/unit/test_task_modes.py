# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from benchmark.simulators.fake.simulator import FakeSimulator
from benchmark.tasks.modes import create_mode, list_modes


def test_list_modes() -> None:
    assert {
        "scenario",
        "random",
        "parametrized",
        "guided",
        "explore",
        "zones",
        "staged",
        "dynamic",
        "benchmark",
    } <= set(list_modes())


def test_random_mode_varies() -> None:
    sim = FakeSimulator({})
    sim.start()
    mode = create_mode("random", {"seed": 1})
    a = mode.setup(0, sim)
    b = mode.setup(1, sim)
    assert (a.start.x, a.start.y) != (b.start.x, b.start.y) or (a.goal.x, a.goal.y) != (b.goal.x, b.goal.y)
    sim.stop()


def test_parametrized_scales_difficulty() -> None:
    sim = FakeSimulator({})
    easy = create_mode("parametrized", {"difficulty": 0.0, "seed": 0}).setup(0, sim)
    hard = create_mode("parametrized", {"difficulty": 1.0, "seed": 0}).setup(0, sim)
    assert hard.obstacles >= easy.obstacles
