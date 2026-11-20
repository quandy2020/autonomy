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

"""Every runtime resource must be listed in ``pyproject.toml`` data-files.

A wheel that drops ``config/scenarios`` or the Habitat GLB visuals installs
cleanly and then fails at runtime, so the patterns are checked against the
files that actually exist in the tree.
"""

from pathlib import Path

import pytest

try:  # Python 3.11+
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - 3.10 fallback
    import tomli as tomllib

from autosim.robot.urdf import UrdfModel

ROOT = UrdfModel.package_root()


def _data_files() -> dict[str, list[str]]:
    with (ROOT / "pyproject.toml").open("rb") as handle:
        config = tomllib.load(handle)
    return config["tool"]["setuptools"]["data-files"]


def _covered(relative: str) -> bool:
    """Whether any data-files pattern matches ``relative``."""
    target = ROOT / relative
    for patterns in _data_files().values():
        for pattern in patterns:
            if target.match(pattern):
                return True
    return False


@pytest.mark.parametrize(
    "relative",
    [
        "config/default.yaml",
        "config/mp3d.scene_dataset_config.json",
        "config/robots/google_robot.yaml",
        "config/scenes/17DRP5sb8fy.yaml",
        "config/scenarios/agents_habitat.yaml",
        "config/scenarios/hunav/hospital_agents.yaml",
        "scripts/run.sh",
        "scripts/smoke_habitat_robots.py",
        "robot/google_robot/google_robot.urdf",
        "robot/google_robot/meshes/habitat/link_base.glb.glb",
        "robot/unitree_g1/unitree_g1.urdf",
        "robot/unitree_go2/unitree_go2.urdf",
        "robot/husky/husky.urdf",
        "robot/turtlebot3_burger/turtlebot3_burger.urdf",
    ],
)
def test_runtime_resource_is_packaged(relative: str):
    assert (ROOT / relative).exists(), f"missing resource: {relative}"
    assert _covered(relative), f"{relative} is not listed in pyproject data-files"


def test_every_data_file_pattern_matches_something():
    """A pattern that matches nothing is a silent packaging hole."""
    empty: list[str] = []
    for target, patterns in _data_files().items():
        for pattern in patterns:
            if not any(ROOT.glob(pattern)):
                empty.append(f"{target} <- {pattern}")
    assert empty == []
