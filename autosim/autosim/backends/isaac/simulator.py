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

"""Isaac Sim WorldBackend: USD stage + sensors aligned with Habitat contracts."""

from __future__ import annotations

import math
from typing import Any, Mapping, Tuple

import numpy as np

import atexit

from autosim.backends.isaac.app import close_app, ensure_simulation_app, update_app
from autosim.backends.isaac.sensors import IsaacSensors
from autosim.backends.isaac.usd_stage import UsdStage, require_isaac
from autosim.robot.profiles import apply_profile, resolve_usd_paths
from autosim.urdf import UrdfModel

_ATEXIT_REGISTERED = False


class IsaacSimulator:
    """WorldBackend for Isaac Sim + USD (z-up map frame)."""

    def __init__(
        self,
        *,
        settings: Mapping[str, Any],
        stage: UsdStage,
        sensors: IsaacSensors,
        urdf: UrdfModel | None,
        width: int,
        height: int,
        eye_height: float,
    ) -> None:
        self.settings = settings
        self._stage = stage
        self._sensors = sensors
        self.urdf = urdf
        self.width = int(width)
        self.height = int(height)
        self._eye_height = float(eye_height)
        self.traffic: Any = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.map_builder: Any = None

    @classmethod
    def create(cls, settings: Mapping[str, Any]) -> "IsaacSimulator":
        """Factory from validated config; requires Isaac Sim Python APIs."""
        habitat = settings["habitat"]
        camera = habitat["sensors"]["camera"]
        # Only load RTX rendering when RGB-D images are actually enabled.
        # A lone depth_points_channel does not require the camera experience.
        want_camera = bool(camera.get("enabled", False))
        isaac_block = habitat.get("isaac") if isinstance(habitat.get("isaac"), Mapping) else {}
        headless_cfg = None
        if isinstance(isaac_block, Mapping) and "headless" in isaac_block:
            headless_cfg = bool(isaac_block["headless"])
        ensure_simulation_app(headless=headless_cfg, enable_cameras=want_camera)
        require_isaac()
        global _ATEXIT_REGISTERED
        if not _ATEXIT_REGISTERED:
            atexit.register(close_app)
            _ATEXIT_REGISTERED = True
        robot_cfg = dict(habitat.get("robot") or {})
        robot_cfg = apply_profile(robot_cfg)
        width = int(camera["width"])
        height = int(camera["height"])
        eye = float(camera.get("sensor_height", robot_cfg.get("base_link_height", 0.5)))
        world_usd, robot_usd = resolve_usd_paths(habitat, robot_cfg)
        base_z = float(robot_cfg.get("base_link_height", 0.0))
        prim = str(
            (habitat.get("usd") or {}).get("robot_prim")
            or robot_cfg.get("usd_prim")
            or "/World/Robot"
        )
        stage = UsdStage(
            world_usd,
            robot_usd=robot_usd,
            robot_prim_path=prim,
            base_z=base_z,
        )
        sensors = IsaacSensors(width=width, height=height, eye_height=eye)
        try:
            urdf = UrdfModel.load(str(robot_cfg.get("urdf") or ""))
        except (FileNotFoundError, ValueError):
            urdf = None
        return cls(
            settings=settings,
            stage=stage,
            sensors=sensors,
            urdf=urdf,
            width=width,
            height=height,
            eye_height=eye,
        )

    def reset(self, x: float, y: float, yaw: float) -> None:
        self.set_pose(x, y, yaw)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self._stage.set_pose(self.x, self.y, self.yaw)

    def pose(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.yaw

    def step(self) -> None:
        update_app()

    def close(self) -> None:
        # Close the USD stage only; Kit app teardown is deferred to atexit so
        # SimulationApp.close() cannot abort the process mid-finally.
        self._stage.close()

    def eye_height(self) -> float:
        return self._eye_height

    def linspace_angles(
        self, angle_min: float, angle_max: float, count: int
    ) -> np.ndarray:
        if count <= 1:
            return np.array([float(angle_min)], dtype=np.float64)
        return np.linspace(
            float(angle_min), float(angle_max), num=int(count), dtype=np.float64
        )

    def laser_ranges(
        self,
        angle_min: float,
        angle_max: float,
        num_beams: int,
        range_max: float,
    ) -> np.ndarray:
        return self._sensors.laser_ranges(
            angle_min,
            angle_max,
            num_beams,
            range_max,
            pose=(self.x, self.y, self.yaw),
        )

    def lidar_points(
        self,
        h_min: float,
        h_max: float,
        h_beams: int,
        v_min: float,
        v_max: float,
        v_rings: int,
        range_max: float,
        **_kwargs: Any,
    ) -> Tuple[np.ndarray, np.ndarray]:
        return self._sensors.lidar_points(
            h_min, h_max, h_beams, v_min, v_max, v_rings, range_max
        )

    def color_depth(self) -> Tuple[np.ndarray, np.ndarray]:
        return self._sensors.color_depth()

    def world_cloud(
        self,
        planar_x: float,
        planar_y: float,
        eye_y: float,
        h_angles: np.ndarray,
        v_angles: np.ndarray,
        range_max: float,
    ) -> np.ndarray:
        """Panoramic cloud via lidar annotator when present; else empty."""
        del planar_x, planar_y, eye_y, h_angles, v_angles
        points, _ = self._sensors.lidar_points(
            -math.pi, math.pi, 64, -0.5, 0.5, 8, float(range_max)
        )
        if points.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float32)
        # Sensor-frame → map frame (approx: rotate by yaw, translate).
        c = float(np.cos(self.yaw))
        s = float(np.sin(self.yaw))
        x = c * points[:, 0] - s * points[:, 1] + self.x
        y = s * points[:, 0] + c * points[:, 1] + self.y
        z = points[:, 2] + self._eye_height
        return np.stack([x, y, z], axis=1).astype(np.float32)
