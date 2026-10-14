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

"""WorldBackend protocol + registry tests (no Isaac / Habitat required)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from autosim.backend import WorldBackend, create_backend
from autosim.backend.registry import BACKENDS, _ensure_builtins
from autosim.config import Config
from tests.fake_simulator import FakeSimulator

ROOT = Path(__file__).resolve().parents[1]


def test_fake_simulator_is_world_backend():
    sim = FakeSimulator()
    assert isinstance(sim, WorldBackend)


def test_registry_builtins():
    _ensure_builtins()
    assert set(BACKENDS) >= {"habitat", "fake", "isaac"}


def test_create_fake_backend():
    settings = Config.load(ROOT / "config" / "fake.yaml")
    world = create_backend(settings.data)
    assert isinstance(world, WorldBackend)
    world.set_pose(1.0, 2.0, 0.5)
    assert world.pose() == pytest.approx((1.0, 2.0, 0.5))
    ranges = world.laser_ranges(-3.14, 3.14, 36, 10.0)
    assert ranges.shape == (36,)
    color, depth = world.color_depth()
    assert color.ndim == 3 and depth.ndim == 2
    cloud = world.sample_depth_points()
    assert cloud.ndim == 2 and cloud.shape[1] == 3
    world.close()


def test_isaac_create_raises_without_omni():
    settings = Config.load(ROOT / "config" / "isaac.yaml")
    try:
        import omni.usd  # noqa: F401

        pytest.skip("Isaac Sim present; ImportError path not exercised")
    except ImportError:
        pass
    with pytest.raises(ImportError, match="Isaac"):
        create_backend(settings.data)


def test_coords_roundtrip():
    from autosim.backends.isaac.coords import map_to_world, world_to_map

    t, q = map_to_world(1.0, -2.0, 0.75, z=0.01)
    x, y, yaw = world_to_map(t, q)
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(-2.0)
    assert yaw == pytest.approx(0.75, abs=1e-6)
    assert isinstance(t, np.ndarray)
