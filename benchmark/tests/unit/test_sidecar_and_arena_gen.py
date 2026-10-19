# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

import numpy as np

from benchmark.common.types import Twist2D
from benchmark.simulators.habitat.simulator import HabitatSimulator
from benchmark.simulators.isaac.simulator import IsaacSimulator
from benchmark.simulators.sidecar import SidecarSimulator, _ranges_from_scan
from benchmark.simulators.worlds.generators.arena_gen import generate_from_prompt, parse_prompt


def test_ranges_from_scan_resample() -> None:
    raw = list(range(72))
    out = _ranges_from_scan(raw, n_beams=36)
    assert out is not None
    assert out.shape == (36,)


def test_sidecar_twin_only_step() -> None:
    sim = SidecarSimulator(
        {
            "launch_autosim": False,
            "autolink": False,
            "rate_hz": 10,
            "humans": {"name": "social_force", "enabled": True},
        }
    )
    sim.start()
    sim.configure_world(obstacles=3, pedestrians=2, moving_obstacles=0, seed=0)
    result = sim.step(Twist2D(linear=0.2, angular=0.0))
    assert "observation" in result.info
    assert result.info.get("sidecar") is False
    st = sim.status()
    assert st["name"] == "sidecar"
    sim.stop()


def test_habitat_isaac_status_fields() -> None:
    h = HabitatSimulator({"launch_autosim": False, "autolink": False})
    h.start()
    assert "sensors" in h.status()
    r = h.step(Twist2D(0.1, 0.0))
    assert r.info.get("backend") == "habitat"
    h.stop()

    i = IsaacSimulator({"launch_autosim": False, "autolink": False})
    i.start()
    assert "isaaclab_path" in i.status()
    i.stop()


def test_arena_gen_prompt_hospital() -> None:
    spec = parse_prompt("generate a hard hospital level 3 with 12 pedestrians")
    assert spec["kind"] == "hospital"
    assert spec["pedestrians"] == 12
    world = generate_from_prompt("easy canteen with tables", seed=2)
    assert world["arena_gen"] is True
    assert world["kind"] == "canteen"


def test_scan_inject_path_without_bridge() -> None:
    # inject_scan only fires when bridge_ok; ensure twin lidar still present
    sim = HabitatSimulator({"launch_autosim": False, "autolink": False, "inject_scan": True})
    sim.start()
    sim.configure_world(obstacles=2, pedestrians=0, moving_obstacles=0, seed=1)
    obs = sim.step(Twist2D(0.0, 0.0)).info["observation"]
    assert isinstance(obs["lidar"], np.ndarray)
    sim.stop()
