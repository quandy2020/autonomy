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

"""Engine-independent scene: occupancy, procedural gen, dynamic agents.

Layout::

    map.py · floorplan.py     ground-truth OccupancyGrid / PLY
    gen/       procedural static maps (Scenario · Volume · Stage …)
    agents/    Obstacle · Pedestrian · Force · Behavior · Agents
    world.py   environment manager (bounds + field + traffic)
    traffic.py obstacle + pedestrian step / lidar hits
    arena.py · field.py · cast.py
"""

from autosim.scene.agents import Agents, Behavior, Force, Obstacle, Pedestrian
from autosim.scene.gen.scenario import Scenario
from autosim.scene.map import Map
from autosim.scene.traffic import Traffic
from autosim.scene.world import World

__all__ = [
    "Map",
    "Scenario",
    "World",
    "Traffic",
    "Obstacle",
    "Pedestrian",
    "Agents",
    "Behavior",
    "Force",
]
