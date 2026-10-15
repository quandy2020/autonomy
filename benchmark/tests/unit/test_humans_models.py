# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from benchmark.humans import create_human_model, list_human_models
from benchmark.simulators.fake.simulator import FakeSimulator
from benchmark.core.types import Twist2D


def test_list_human_models() -> None:
    names = set(list_human_models())
    assert {"social_force", "crowd_flow", "orca", "hunav", "pysocialforce"} <= names


def test_social_force_in_fake_sim() -> None:
    sim = FakeSimulator(
        {
            "humans": {"name": "social_force", "behavior": "commuter"},
            "rate_hz": 10,
        }
    )
    sim.start()
    sim.configure_world(obstacles=2, pedestrians=4, moving_obstacles=0, seed=1)
    assert len(sim.world.pedestrians) == 4
    assert sim.world.human_model is not None
    before = [(p.x, p.y) for p in sim.world.pedestrians]
    for _ in range(5):
        sim.step(Twist2D(linear=0.1, angular=0.0))
    after = [(p.x, p.y) for p in sim.world.pedestrians]
    assert before != after
    sim.stop()


def test_crowd_flow_spawn() -> None:
    model = create_human_model({"name": "crowd_flow", "flow_direction": [1.0, 0.0], "count": 5})
    sim = FakeSimulator({"rate_hz": 10})
    sim.start()
    model.spawn(sim.world, 5)
    assert len(sim.world.pedestrians) == 5
    model.step(sim.world)
    sim.stop()
