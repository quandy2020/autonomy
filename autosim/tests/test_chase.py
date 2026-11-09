# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for third-person chase camera config."""

from __future__ import annotations

import math

import pytest

from autosim.backends.habitat.simulator import Simulator
from autosim.bridge.chase import chase_body_mount, chase_spec


def test_chase_spec_disabled_by_default():
    assert chase_spec(None) is None
    assert chase_spec({}) is None
    assert chase_spec({"chase": {"enabled": False}}) is None


def test_chase_body_mount_is_behind_and_above_in_ros():
    xyz, rpy = chase_body_mount(
        {"distance_m": 1.0, "height_m": 0.85, "pitch_deg": -25.0}
    )
    assert xyz == (-1.0, 0.0, 0.85)
    assert rpy[0] == 0.0
    assert rpy[2] == 0.0
    assert rpy[1] == pytest.approx(math.radians(-25.0))


def test_chase_spec_auto_pitch():
    cfg = chase_spec(
        {
            "chase": {
                "enabled": True,
                "distance_m": 2.0,
                "height_m": 1.5,
                "look_at_height_m": 0.5,
            }
        }
    )
    assert cfg is not None
    assert cfg["channel"] == "/camera/chase/image_raw"
    assert cfg["distance_m"] == 2.0
    expected = -math.degrees(math.atan2(1.0, 2.0))
    assert cfg["pitch_deg"] == pytest.approx(expected)


def test_chase_enables_urdf_instance():
    simulator = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={
            "habitat": {
                "path": "",
                "spawn": [0.0, 0.0, 0.0],
                "robot": {"urdf": "robot/turtlebot3_burger/turtlebot3_burger.urdf"},
                "sensors": {"chase": {"enabled": True}},
            }
        },
        open_session=False,
    )
    assert simulator.chase_enabled()
    assert simulator.should_instance_urdf()


def test_sync_articulated_uses_ros_yaw_not_agent_offset():
    """URDF +X already maps to Habitat +X after Rx(−π/2); no yaw−π/2."""
    mn = pytest.importorskip("magnum")

    class FakeAO:
        def __init__(self):
            self.translation = None
            self.rotation = None

    simulator = Simulator(
        backend="minimal",
        width=64,
        height=48,
        settings={
            "habitat": {
                "path": "",
                "spawn": [0.0, 0.0, 0.0],
                "robot": {"urdf": ""},
            }
        },
        open_session=False,
    )
    ao = FakeAO()
    simulator.articulated = ao
    simulator.x, simulator.y, simulator.yaw = 1.0, 2.0, 0.0
    simulator.floor_y = 0.1
    simulator.sync_articulated(mn)
    assert list(ao.translation) == pytest.approx([1.0, 0.1, -2.0])
    # At yaw=0: rotation == Rx(−π/2) only (identity Ry).
    expected = mn.Quaternion.rotation(mn.Rad(-math.pi / 2.0), mn.Vector3(1, 0, 0))
    assert float(ao.rotation.scalar) == pytest.approx(float(expected.scalar), abs=1e-6)
    assert list(ao.rotation.vector) == pytest.approx(
        list(expected.vector), abs=1e-6
    )
