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

"""Procedural scenario: static map, moving obstacles, and pedestrians."""

from autosim.scenario.map.scenario import Scenario
from autosim.scenario.obstacle.obstacle import Obstacle
from autosim.scenario.pedestrian.pedestrian import Pedestrian
from autosim.scenario.traffic import Traffic

__all__ = ["Scenario", "Obstacle", "Pedestrian", "Traffic"]
