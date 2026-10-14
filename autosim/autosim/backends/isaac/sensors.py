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

"""Isaac camera / lidar → Habitat-shaped numpy arrays.

RGB-D uses ``isaacsim.sensors.camera.Camera`` (Replicator render product +
annotators) parented under the robot Xform so poses follow ``UsdStage.set_pose``.
"""

from __future__ import annotations

import logging
import math
from typing import Any, Optional, Tuple

import numpy as np

from autosim.backends.isaac.app import update_app

_LOG = logging.getLogger(__name__)


class IsaacSensors:
    """Read RGB-D / lidar from an Isaac Camera (or raw annotators).

    When no camera is mounted, methods return empty / placeholder arrays so
    Sensors / Runner keep working (pose-only smoke).
    """

    # Keep far clip in the robotics band. Isaac defaults (~1e5 m) produce
    # sparse far hits that make autoviz depth colorization flicker.
    _MIN_DEPTH_M = 0.05
    _MAX_DEPTH_M = 40.0

    def __init__(
        self,
        *,
        width: int = 640,
        height: int = 480,
        eye_height: float = 0.5,
    ) -> None:
        self.width = max(1, int(width))
        self.height = max(1, int(height))
        self.eye_height = float(eye_height)
        self._camera: Any = None
        self._rgb_annotator: Any = None
        self._depth_annotator: Any = None
        self._lidar_annotator: Any = None

    def attach(
        self,
        *,
        rgb: Any = None,
        depth: Any = None,
        lidar: Any = None,
        camera: Any = None,
    ) -> None:
        """Bind optional Camera / Replicator annotators (no-op when ``None``)."""
        if camera is not None:
            self._camera = camera
        self._rgb_annotator = rgb
        self._depth_annotator = depth
        self._lidar_annotator = lidar

    def mount_robot_camera(
        self,
        robot_prim_path: str,
        *,
        hfov_deg: float = 90.0,
        prim_suffix: str = "Camera",
        warmup_frames: int = 8,
    ) -> bool:
        """Spawn a forward-looking Camera under ``robot_prim`` and attach it.

        Args:
            robot_prim_path: Parent Xform (e.g. ``/World/Robot``).
            hfov_deg: Horizontal field of view in degrees.
            prim_suffix: Child prim name under the robot.
            warmup_frames: Kit updates so the first RGB/depth frames are valid.

        Returns:
            ``True`` when the camera was created and initialized.
        """
        try:
            from isaacsim.sensors.camera import Camera
        except ImportError as exc:
            _LOG.warning("isaacsim.sensors.camera unavailable: %s", exc)
            return False

        prim_path = f"{robot_prim_path.rstrip('/')}/{prim_suffix}"
        try:
            cam = Camera(
                prim_path=prim_path,
                name="autosim_rgb_camera",
                resolution=(self.width, self.height),
                translation=np.array([0.0, 0.0, self.eye_height], dtype=np.float64),
                # world axes: +Z up, +X forward (matches autosim map/robot frame).
                orientation=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            )
            # Re-apply local pose with explicit camera_axes (ctor uses usd default).
            cam.set_local_pose(
                translation=[0.0, 0.0, self.eye_height],
                orientation=[1.0, 0.0, 0.0, 0.0],
                camera_axes="world",
            )
            self._apply_hfov(cam, float(hfov_deg))
            try:
                cam.set_clipping_range(
                    near_distance=0.1, far_distance=float(self._MAX_DEPTH_M)
                )
            except Exception as exc:  # noqa: BLE001
                _LOG.debug("could not set camera clipping range: %s", exc)
            cam.initialize(attach_rgb_annotator=True)
            cam.add_distance_to_image_plane_to_frame()
        except Exception as exc:  # noqa: BLE001 — Kit/Replicator failures vary
            _LOG.warning("failed to mount Isaac camera at %s: %s", prim_path, exc)
            return False

        self._camera = cam
        for _ in range(max(0, int(warmup_frames))):
            update_app()
        _LOG.info(
            "Isaac camera mounted at %s (%dx%d, hfov=%.1f°)",
            prim_path,
            self.width,
            self.height,
            float(hfov_deg),
        )
        return True

    @staticmethod
    def _apply_hfov(camera: Any, hfov_deg: float) -> None:
        """Set horizontal aperture from desired FOV (keeps default focal length)."""
        hfov = max(1.0, min(179.0, float(hfov_deg)))
        try:
            focal = float(camera.get_focal_length())
            if focal <= 0.0:
                focal = 24.0
                camera.set_focal_length(focal)
            # horizontal_fov = 2 * atan(aperture / (2 * focal))
            aperture = 2.0 * focal * math.tan(math.radians(hfov) * 0.5)
            camera.set_horizontal_aperture(aperture)
        except Exception as exc:  # noqa: BLE001
            _LOG.debug("could not set camera hfov: %s", exc)

    def _to_hw(self, arr: Any) -> np.ndarray:
        """Normalize Isaac annotator arrays to contiguous ``(H, W)`` float32.

        ``distance_to_image_plane`` is documented as ``(width, height, 1)``; a
        plain ``reshape(H, W)`` on that layout scrambles rows and looks like
        depth flicker under motion.
        """
        data = np.asarray(arr)
        if data.ndim >= 3:
            data = data[..., 0]
        data = np.asarray(data, dtype=np.float32)
        if data.shape == (self.height, self.width):
            return np.ascontiguousarray(data)
        if data.shape == (self.width, self.height):
            return np.ascontiguousarray(data.T)
        if data.size == self.height * self.width:
            # Prefer documented Isaac (W, H) packing when only the size is known.
            return np.ascontiguousarray(
                data.reshape(self.width, self.height).T
            )
        return np.full((self.height, self.width), np.nan, dtype=np.float32)

    def _sanitize_depth(self, arr: Any) -> np.ndarray:
        """Clip invalid / far-plane hits to NaN for stable 32FC1 visualization."""
        depth = self._to_hw(arr)
        valid = (
            np.isfinite(depth)
            & (depth > float(self._MIN_DEPTH_M))
            & (depth < float(self._MAX_DEPTH_M))
        )
        if np.all(valid):
            return depth
        out = depth.copy()
        out[~valid] = np.nan
        return out

    def close(self) -> None:
        """Detach annotators / destroy render product when present."""
        cam = self._camera
        self._camera = None
        self._rgb_annotator = None
        self._depth_annotator = None
        self._lidar_annotator = None
        if cam is None:
            return
        try:
            destroy = getattr(cam, "destroy", None)
            if callable(destroy):
                destroy()
        except Exception:
            pass

    def color_depth(self) -> Tuple[np.ndarray, np.ndarray]:
        color = self._read_rgb()
        depth = self._read_depth()
        return color, depth

    def _read_rgb(self) -> np.ndarray:
        if self._camera is not None:
            try:
                data = self._camera.get_rgb(device="cpu")
            except Exception:
                data = None
            if data is not None:
                rgb = self._to_hw_rgb(data)
                if rgb is not None:
                    return rgb
        if self._rgb_annotator is not None:
            rgb = self._to_hw_rgb(self._rgb_annotator.get_data())
            if rgb is not None:
                return rgb
        out = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        out[..., 2] = 64
        return out

    def _to_hw_rgb(self, arr: Any) -> Optional[np.ndarray]:
        """Normalize RGB annotator output to ``(H, W, 3)`` uint8."""
        data = np.asarray(arr)
        if data.ndim != 3 or data.shape[-1] < 3 or data.size == 0:
            return None
        rgb = data[..., :3]
        if rgb.shape[0] == self.height and rgb.shape[1] == self.width:
            return rgb.astype(np.uint8, copy=False)
        if rgb.shape[0] == self.width and rgb.shape[1] == self.height:
            return np.ascontiguousarray(np.transpose(rgb, (1, 0, 2))).astype(
                np.uint8, copy=False
            )
        return None

    def _read_depth(self) -> np.ndarray:
        if self._camera is not None:
            try:
                data = self._camera.get_depth(device="cpu")
            except Exception:
                data = None
            if data is not None and np.asarray(data).size > 0:
                return self._sanitize_depth(data)
        if self._depth_annotator is not None:
            data = self._depth_annotator.get_data()
            if data is not None and np.asarray(data).size > 0:
                return self._sanitize_depth(data)
        return np.full((self.height, self.width), np.nan, dtype=np.float32)

    def laser_ranges(
        self,
        angle_min: float,
        angle_max: float,
        num_beams: int,
        range_max: float,
        *,
        pose: Tuple[float, float, float],
    ) -> np.ndarray:
        """Prefer lidar annotator; else empty (NaN) planar scan."""
        del pose
        cloud = self._lidar_xy()
        if cloud is None:
            return np.full((int(num_beams),), np.nan, dtype=np.float32)
        angles = np.linspace(
            float(angle_min), float(angle_max), num=int(num_beams), dtype=np.float64
        )
        ranges = np.full(angles.shape, np.nan, dtype=np.float32)
        if cloud.shape[0] == 0:
            return ranges
        pts_angles = np.arctan2(cloud[:, 1], cloud[:, 0])
        pts_ranges = np.linalg.norm(cloud[:, :2], axis=1)
        step = (float(angle_max) - float(angle_min)) / max(int(num_beams) - 1, 1)
        for i, ang in enumerate(angles):
            mask = np.abs(((pts_angles - ang + math.pi) % (2 * math.pi)) - math.pi) <= (
                0.5 * step + 1e-6
            )
            if not np.any(mask):
                continue
            hit = float(np.min(pts_ranges[mask]))
            if hit <= float(range_max):
                ranges[i] = hit
        return ranges

    def lidar_points(
        self,
        h_min: float,
        h_max: float,
        h_beams: int,
        v_min: float,
        v_max: float,
        v_rings: int,
        range_max: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        del h_min, h_max, h_beams, v_min, v_max, v_rings
        cloud = self._lidar_xyz()
        if cloud is None or cloud.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=np.float32)
        dist = np.linalg.norm(cloud, axis=1)
        mask = dist <= float(range_max)
        cloud = cloud[mask]
        rings = np.zeros((cloud.shape[0],), dtype=np.float32)
        return cloud.astype(np.float32), rings

    def _lidar_xy(self) -> Optional[np.ndarray]:
        xyz = self._lidar_xyz()
        if xyz is None:
            return None
        return xyz[:, :2]

    def _lidar_xyz(self) -> Optional[np.ndarray]:
        if self._lidar_annotator is None:
            return None
        data = np.asarray(self._lidar_annotator.get_data(), dtype=np.float32)
        if data.size == 0:
            return np.zeros((0, 3), dtype=np.float32)
        return data.reshape(-1, data.shape[-1])[:, :3]
