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

"""Unit tests for Habitat SMPL-X humanoid helpers (no Kit / no habitat-sim)."""

from __future__ import annotations

import pickle
from pathlib import Path

import numpy as np
import pytest

from autosim.backends.habitat.humanoids.assets import (
    HumanoidAsset,
    list_humanoid_assets,
)
from autosim.backends.habitat.humanoids.sync import (
    humanoid_config_from_habitat,
    maybe_create_humanoid_sync,
    resolve_humanoid_asset,
)
from autosim.runtime.settings import Config
from autosim.scene.agents.roster import Agents


def test_humanoid_config_defaults():
    cfg = humanoid_config_from_habitat({})
    assert cfg["enabled"] is False
    assert cfg["visual"] == "humanoid"
    assert cfg["sync_hz"] == pytest.approx(30.0)
    assert cfg["snap_navmesh"] is True
    assert cfg["skin_map"] == {}

    cfg2 = humanoid_config_from_habitat(
        {
            "humanoids": {
                "enabled": True,
                "visual": "none",
                "names": ["male_46"],
                "sync_hz": 15,
                "skin_map": {1: "male_46", "2": "female_2"},
            }
        }
    )
    assert cfg2["enabled"] is True
    assert cfg2["visual"] == "none"
    assert cfg2["names"] == ["male_46"]
    assert cfg2["sync_hz"] == pytest.approx(15.0)
    assert cfg2["skin_map"] == {1: "male_46", 2: "female_2"}


def test_validate_humanoids_accepts_and_rejects():
    Config.validate_humanoids(None)
    Config.validate_humanoids({"enabled": False})
    Config.validate_humanoids(
        {
            "enabled": True,
            "visual": "humanoid",
            "sync_hz": 30.0,
            "skin_map": {1: "male_46"},
        }
    )
    with pytest.raises(ValueError, match="visual"):
        Config.validate_humanoids({"visual": "capsule"})
    with pytest.raises(ValueError, match="bool"):
        Config.validate_humanoids({"enabled": "yes"})
    with pytest.raises(ValueError, match="skin_map"):
        Config.validate_humanoids({"skin_map": ["male_46"]})


def test_agents_avatar_and_string_skin():
    agents = Agents.from_mapping(
        {
            "agents": [
                {
                    "pose": {"x": 0, "y": 0, "yaw": 0},
                    "avatar": "male_46",
                    "skin": 0,
                    "goals": [{"x": 1, "y": 0}],
                },
                {
                    "pose": {"x": 1, "y": 0, "yaw": 0},
                    "skin": "female_2",
                    "goals": [{"x": 2, "y": 0}],
                },
                {
                    "pose": {"x": 2, "y": 0, "yaw": 0},
                    "skin": 3,
                    "goals": [{"x": 3, "y": 0}],
                },
            ]
        }
    )
    assert agents.records[0]["avatar"] == "male_46"
    assert agents.records[0]["skin"] == 0
    assert agents.records[1]["avatar"] == "female_2"
    assert agents.records[1]["skin"] == 0
    assert agents.records[2]["avatar"] == ""
    assert agents.records[2]["skin"] == 3


def test_resolve_humanoid_asset_priority():
    assets = [
        HumanoidAsset(name="a", urdf=Path("/a.urdf"), motion=Path("/a.pkl")),
        HumanoidAsset(name="b", urdf=Path("/b.urdf"), motion=Path("/b.pkl")),
        HumanoidAsset(name="c", urdf=Path("/c.urdf"), motion=Path("/c.pkl")),
    ]
    assert resolve_humanoid_asset(assets, index=0, avatar="b").name == "b"
    assert (
        resolve_humanoid_asset(
            assets, index=0, skin=1, skin_map={1: "c"}
        ).name
        == "c"
    )
    assert resolve_humanoid_asset(assets, index=0, skin=2).name == "b"
    assert resolve_humanoid_asset(assets, index=5, skin=0).name == "c"


def test_list_humanoid_assets(tmp_path: Path):
    name = "avatar_a"
    folder = tmp_path / name
    folder.mkdir()
    (folder / f"{name}.urdf").write_text("<robot name='a'/>")
    motion = {
        "walk_motion": {
            "joints_array": np.zeros((2, 4), dtype=np.float64),
            "transform_array": np.array(
                [np.eye(4, dtype=np.float64), np.eye(4, dtype=np.float64)]
            ),
            "displacement": np.array([0.0, 0.5], dtype=np.float64),
            "fps": 30.0,
        },
        "stop_pose": {
            "joints": np.zeros(4, dtype=np.float64),
            "transform": np.eye(4, dtype=np.float64),
        },
    }
    with (folder / f"{name}_motion_data_smplx.pkl").open("wb") as handle:
        pickle.dump(motion, handle)

    assets = list_humanoid_assets(tmp_path)
    assert len(assets) == 1
    assert assets[0].name == name
    assert assets[0].urdf.is_file()
    assert assets[0].motion.is_file()

    assert list_humanoid_assets(tmp_path, names=["missing"]) == []
    assert list_humanoid_assets(tmp_path / "nope") == []


def test_walk_gait_controller_advance(tmp_path: Path):
    pytest.importorskip("magnum")
    from autosim.backends.habitat.humanoids.controller import WalkGaitController

    path = tmp_path / "motion.pkl"
    joints = np.zeros((4, 8), dtype=np.float64)
    joints[1, 0] = 1.0
    motion = {
        "walk_motion": {
            "joints_array": joints,
            "transform_array": np.stack([np.eye(4) for _ in range(4)]),
            "displacement": np.linspace(0.0, 1.0, 4),
            "fps": 30.0,
        },
        "stop_pose": {
            "joints": np.zeros(8, dtype=np.float64),
            "transform": np.eye(4, dtype=np.float64),
        },
    }
    with path.open("wb") as handle:
        pickle.dump(motion, handle)

    ctrl = WalkGaitController(path, motion_fps=30.0)
    ctrl.advance(0.0, 0.1)
    joints_stop, _ = ctrl.joints_and_offset()
    assert joints_stop == list(ctrl.stop_pose.joints)

    ctrl.advance(1.0, 0.1)
    joints_walk, offset = ctrl.joints_and_offset()
    assert len(joints_walk) == 8
    assert offset is not None


def test_maybe_create_humanoid_sync_missing_assets():
    sync = maybe_create_humanoid_sync(
        session=None,
        config={
            "enabled": True,
            "visual": "humanoid",
            "root": "/nonexistent/humanoid_data",
            "names": [],
        },
    )
    assert sync is None

    sync2 = maybe_create_humanoid_sync(
        session=None,
        config={"enabled": False, "visual": "humanoid"},
    )
    assert sync2 is None

    sync3 = maybe_create_humanoid_sync(
        session=None,
        config={"enabled": True, "visual": "none"},
    )
    assert sync3 is None


def test_humanoid_asset_dataclass():
    asset = HumanoidAsset(
        name="x",
        urdf=Path("/tmp/x.urdf"),
        motion=Path("/tmp/x.pkl"),
    )
    assert asset.name == "x"
