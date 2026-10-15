# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Simulator adapters (autosim backends + HIL)."""

from __future__ import annotations

from benchmark.core.registry import get_registry
from benchmark.simulators.base import SimulatorAdapter
from benchmark.simulators.factory import create_simulator
from benchmark.simulators.fake import FakeSimulator
from benchmark.simulators.flatland import FlatlandSimulator
from benchmark.simulators.gazebo import GazeboSimulator
from benchmark.simulators.habitat import HabitatSimulator
from benchmark.simulators.hil import HilSimulator
from benchmark.simulators.isaac import IsaacSimulator
from benchmark.simulators.unity import UnitySimulator

_hub = get_registry()
_hub.simulators.register("fake", FakeSimulator)
_hub.simulators.register("flatland", FlatlandSimulator)
_hub.simulators.register("habitat", HabitatSimulator)
_hub.simulators.register("isaac", IsaacSimulator)
_hub.simulators.register("hil", HilSimulator)
_hub.simulators.register("gazebo", GazeboSimulator)
_hub.simulators.register("unity", UnitySimulator)

__all__ = [
    "FakeSimulator",
    "FlatlandSimulator",
    "GazeboSimulator",
    "HabitatSimulator",
    "HilSimulator",
    "IsaacSimulator",
    "UnitySimulator",
    "SimulatorAdapter",
    "create_simulator",
]
