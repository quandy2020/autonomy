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

"""Robot profile resolution tests."""

from __future__ import annotations

from pathlib import Path

import pytest

from autosim.config import Config
from autosim.robot.profiles import PROFILES, apply_profile, get_profile, resolve_path
from autosim.robot.urdf import UrdfModel

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("name", sorted(PROFILES))
def test_profile_urdf_exists(name: str):
    profile = get_profile(name)
    path = UrdfModel.resolve(profile.urdf)
    assert path is not None and path.is_file(), profile.urdf
    usd = resolve_path(profile.usd_robot)
    assert usd.is_file(), profile.usd_robot


def test_apply_profile_fills_defaults():
    merged = apply_profile({"profile": "husky", "cmd_vel": "/cmd_vel"})
    assert merged["urdf"] == "robot/husky/husky.urdf"
    assert "usd_robot" in merged
    assert merged["wheel_separation"] == pytest.approx(0.512)


def test_apply_profile_beni():
    merged = apply_profile({"profile": "beni"})
    assert merged["urdf"] == "robot/beni/beni.urdf"
    assert merged["wheel_separation"] == pytest.approx(0.160)
    assert merged["base_link_height"] == pytest.approx(0.11)
    model = UrdfModel.load(merged["urdf"])
    assert model is not None
    assert model.root == "base_footprint"
    assert "camera_link" in model.parents
    cx, cy, cz = model.camera_xyz()
    assert abs(cx - 0.07) < 1e-6
    assert abs(cy - 0.03) < 1e-6
    assert abs(cz - (-0.016 + 0.11)) < 1e-6  # via base_footprint→base_link


def test_apply_profile_keeps_overrides():
    merged = apply_profile(
        {
            "profile": "turtlebot3_burger",
            "urdf": "robot/husky/husky.urdf",
            "wheel_separation": 0.99,
        }
    )
    assert merged["urdf"] == "robot/husky/husky.urdf"
    assert merged["wheel_separation"] == pytest.approx(0.99)


def test_isaac_yaml_loads():
    settings = Config.load(ROOT / "config" / "isaac.yaml")
    assert settings["habitat"]["backend"] == "isaac"
    assert settings["habitat"]["robot"]["profile"] == "beni"
    assert settings.channel_map().get("robot_description") == "/robot_description"
    camera = settings["habitat"]["sensors"]["camera"]
    assert camera["enabled"] is True
    assert camera["depth_channel"]
    assert camera["rgb_channel"]
    path = resolve_path(settings["habitat"]["path"])
    assert path.is_file()


def test_config_rejects_unknown_backend():
    data = Config.load(ROOT / "config" / "default.yaml").data
    data["habitat"]["backend"] = "unity"
    with pytest.raises(ValueError, match="backend"):
        Config.validate(data)
