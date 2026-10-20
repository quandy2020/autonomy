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

"""WorldBackend protocol: engine-agnostic sensing + pose lifecycle.

Optional behaviour (surround cameras, navmesh map sample, fallback TF, …)
is declared via ``capabilities`` / ``supports_*`` — see
:mod:`autosim.backends.capabilities`. Runner must not switch on backend names.
"""

from __future__ import annotations

from typing import Any, Optional, Protocol, Tuple, runtime_checkable

import numpy as np


@runtime_checkable
class WorldBackend(Protocol):
    """Simulator contract used by Sensors, Map, and Runner.

    Map frame is z-up planar ``(x, y, yaw)``. Engines with other conventions
    convert inside the backend.
    """

    urdf: Any
    traffic: Any

    def reset(self, x: float, y: float, yaw: float) -> None:
        """Teleport and clear transient sensor state."""

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        """Set the agent / robot planar pose in the map frame."""

    def pose(self) -> Tuple[float, float, float]:
        """Return ``(x, y, yaw)`` in the map frame."""

    def step(self) -> None:
        """Advance the engine one control tick (may be a no-op)."""

    def close(self) -> None:
        """Release GPU / stage resources."""

    def eye_height(self) -> float:
        """Sensor / eye height above the ground (meters)."""

    def linspace_angles(
        self, angle_min: float, angle_max: float, count: int
    ) -> np.ndarray:
        """Evenly spaced angles for lidar / panoramic samples."""

    def laser_ranges(
        self,
        angle_min: float,
        angle_max: float,
        num_beams: int,
        range_max: float,
    ) -> np.ndarray:
        """2D lidar ranges ``(N,)`` float32."""

    def lidar_points(
        self,
        h_min: float,
        h_max: float,
        h_beams: int,
        v_min: float,
        v_max: float,
        v_rings: int,
        range_max: float,
        **kwargs: Any,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """3D lidar ``(Nx3, rings)`` in the sensor frame."""

    def color_depth(self) -> Tuple[np.ndarray, np.ndarray]:
        """RGB ``HxWx3`` uint8 and depth ``HxW`` float32 (meters)."""

    def world_cloud(
        self,
        planar_x: float,
        planar_y: float,
        eye_y: float,
        h_angles: np.ndarray,
        v_angles: np.ndarray,
        range_max: float,
    ) -> np.ndarray:
        """Panoramic occupancy cloud in the map frame (z-up)."""


def optional_call(backend: Any, name: str, *args: Any, **kwargs: Any) -> Any:
    """Call ``backend.name`` when present, else ``None``."""
    method = getattr(backend, name, None)
    if not callable(method):
        return None
    return method(*args, **kwargs)
