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

"""Global map for the fake backend: scenario occupancy cloud + grid."""

from __future__ import annotations

from typing import Any, Mapping, Optional, Tuple

import numpy as np

from autosim.scenario.map.scenario import Scenario


class Map:
    """Build and cache the scenario world without Habitat."""

    def __init__(self, settings: Mapping[str, Any] | None = None) -> None:
        """Bind ``habitat.map`` settings (publish channels / stride)."""
        self.settings = dict(settings or {})
        self.scenario: Scenario | None = None
        self.points = np.zeros((0, 3), dtype=np.float32)
        self.occupied: Optional[np.ndarray] = None
        self.origin = np.zeros(3, dtype=np.float64)
        self.resolution = 0.0
        self.grid: tuple[np.ndarray, float, float, float, int, int] | None = None
        self.cloud_rgb: Optional[np.ndarray] = None
        self.cached_cloud: Optional[np.ndarray] = None
        self.cached_grid: Optional[
            Tuple[np.ndarray, float, float, float, int, int]
        ] = None

    def build(self, scenario_block: Mapping[str, Any]) -> "Map":
        """Generate occupancy from ``habitat.scenario`` (no mesh file)."""
        self.scenario = Scenario(scenario_block).build(write_mesh=False)
        self.points = np.asarray(self.scenario.points, dtype=np.float32).reshape(-1, 3)
        self.occupied = self.scenario.occupied
        self.origin = np.asarray(self.scenario.origin, dtype=np.float64).reshape(3)
        self.resolution = float(self.scenario.resolution)
        self.grid = self.scenario.grid
        self.cached_cloud = self.points
        self.cached_grid = self.grid
        return self

    def sample(
        self, simulator: Any = None, origin_xy: Tuple[float, float] = (0.0, 0.0)
    ) -> Tuple[np.ndarray, np.ndarray, float, float, float, int, int]:
        """Return cached cloud and occupancy grid (API matches habitat Map)."""
        del simulator, origin_xy
        if self.cached_cloud is None or self.cached_grid is None:
            raise RuntimeError("fake map not built; call Map.build(scenario) first")
        cells, resolution, ox, oy, width, height = self.cached_grid
        return (
            self.cached_cloud,
            cells,
            float(resolution),
            float(ox),
            float(oy),
            int(width),
            int(height),
        )
