# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Create a simulator from BenchmarkConfig."""

from __future__ import annotations

from typing import Any, Dict

from benchmark.core.config import BenchmarkConfig
from benchmark.core.registry import get_registry
from benchmark.simulators.base import SimulatorAdapter


def create_simulator(config: BenchmarkConfig) -> SimulatorAdapter:
    hub = get_registry()
    hub.ensure_builtins()
    name = config.simulator_name
    sim_cfg: Dict[str, Any] = config.section("simulator")
    # Propagate humans block into simulator so Fake/Isaac twin can use it.
    humans = config.raw.get("humans")
    if isinstance(humans, dict) and "humans" not in sim_cfg:
        sim_cfg = dict(sim_cfg)
        sim_cfg["humans"] = humans
    return hub.simulators.create(name, sim_cfg)
