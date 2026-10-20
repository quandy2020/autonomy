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

"""Unit tests for Isaac feature slices (no Kit required)."""

from __future__ import annotations

import math

import numpy as np

from autosim.backends.isaac.randomization import randomization_config
from autosim.backends.isaac.robots.wheeled import twist_to_wheel_speeds
from autosim.backends.isaac.sensors import SensorSuite
from autosim.backends.isaac.world import IsaacWorld


def test_twist_to_wheel_speeds_straight():
    left, right = twist_to_wheel_speeds(0.33, 0.0, track_width=0.16, wheel_radius=0.033)
    assert math.isclose(left, right, rel_tol=1e-6)
    assert left > 0


def test_twist_to_wheel_speeds_spin():
    left, right = twist_to_wheel_speeds(0.0, 1.0, track_width=0.16, wheel_radius=0.033)
    assert left < 0 < right or right < 0 < left


def test_domain_random_defaults_off():
    cfg = randomization_config({})
    assert cfg["enabled"] is False
    cfg = randomization_config({"domain_random": {"enabled": True, "seed": 3}})
    assert cfg["enabled"] is True
    assert cfg["seed"] == 3


def test_coerce_point_cloud():
    pts = SensorSuite._coerce_point_cloud(np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float32))
    assert pts.shape == (2, 3)
    flat = SensorSuite._coerce_point_cloud(np.arange(9, dtype=np.float32))
    assert flat.shape == (3, 3)


def test_merge_nearer_ranges():
    a = np.array([1.0, np.nan, 3.0])
    b = np.array([2.0, 0.5, np.nan])
    out = IsaacWorld.merge_nearer_ranges(a, b)
    assert out[0] == 1.0
    assert out[1] == 0.5
    assert out[2] == 3.0


def test_lidar_backend_validation():
    from autosim.runtime.settings import Config

    block = {
        "enabled": True,
        "num_beams": 10,
        "angle_min": -1.0,
        "angle_max": 1.0,
        "range_min": 0.1,
        "range_max": 10.0,
        "backend": "merge",
    }
    Config.validate_lidar_2d(block)
    block["backend"] = "nope"
    try:
        Config.validate_lidar_2d(block)
        assert False, "expected ValueError"
    except ValueError:
        pass
