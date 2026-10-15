# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Simulator adapters (autosim backends + HIL)."""

from __future__ import annotations

from benchmark.core.registry import get_registry
from benchmark.simulators.base import SimulatorAdapter
from benchmark.simulators.factory import create_simulator
from benchmark.simulators.fake import FakeSimulator
from benchmark.simulators.habitat import HabitatSimulator
from benchmark.simulators.hil import HilSimulator
from benchmark.simulators.isaac import IsaacSimulator

_hub = get_registry()
_hub.simulators.register("fake", FakeSimulator)
_hub.simulators.register("habitat", HabitatSimulator)
_hub.simulators.register("isaac", IsaacSimulator)
_hub.simulators.register("hil", HilSimulator)

__all__ = [
    "FakeSimulator",
    "HabitatSimulator",
    "HilSimulator",
    "IsaacSimulator",
    "SimulatorAdapter",
    "create_simulator",
]
