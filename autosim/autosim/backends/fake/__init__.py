# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Procedural scenario WorldBackend (no Habitat / Isaac).

Public surface::

    FakeWorld   — WorldBackend (registry name ``fake``)
    Map         — scenario occupancy builder
    Sensor      — pinhole RGB-D raycast
    Robot       — lightweight differential plant (tests)
"""

from __future__ import annotations

from typing import Any, Mapping

from autosim.backend.registry import register


def create(settings: Mapping[str, Any]) -> Any:
    """Build a FakeWorld from ``habitat.scenario`` (+ optional map)."""
    from autosim.backends.fake.world import FakeWorld

    return FakeWorld.from_settings(settings)


register("fake", create)


def __getattr__(name: str) -> Any:
    if name == "FakeWorld":
        from autosim.backends.fake.world import FakeWorld

        return FakeWorld
    if name == "Map":
        from autosim.backends.fake.map import Map

        return Map
    if name == "Sensor":
        from autosim.backends.fake.sensor import Sensor

        return Sensor
    if name == "Robot":
        from autosim.backends.fake.robot import Robot

        return Robot
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = ["FakeWorld", "Map", "Robot", "Sensor", "create"]
