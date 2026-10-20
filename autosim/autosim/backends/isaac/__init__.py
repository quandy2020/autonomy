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

"""Isaac Sim WorldBackend — module names mirror isaacsim idioms.

::

    app/                 Kit AppLauncher / SimulationApp (isaaclab.app)
    transforms.py        map-frame ↔ world poses
    stage.py             open stage · robot Xform · mesh cloud
    sensors.py           RGB-D · RTX Lidar · surround cameras
    physics.py           PhysX world · capsule collision
    articulation.py      generic Articulation helpers
    robots/beni.py       Beni floating-base articulation
    robots/wheeled.py    differential-drive wheel joints
    randomization.py     light / character randomization
    people.py            PeopleSync (Traffic → USD)
    animation.py         PeopleAnimator (UsdSkel walk/idle)
    world.py             WorldBackend assembly (IsaacWorld)
"""

from __future__ import annotations

from typing import Any, Mapping

from autosim.backends.registry import register


def create(settings: Mapping[str, Any]) -> Any:
    """Build an Isaac Sim WorldBackend from full config data.

    Raises:
        ImportError: Isaac Sim / Omniverse Python packages are unavailable.
        FileNotFoundError: Configured USD world path does not exist.
    """
    from autosim.backends.isaac.world import IsaacWorld

    return IsaacWorld.create(dict(settings))


register("isaac", create)
