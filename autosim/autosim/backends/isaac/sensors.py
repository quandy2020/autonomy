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
annotators) parented under the robot Xform. PhysX/Fabric often leave the
Replicator view matrix stale, so ``sync_world_pose`` re-applies the eye pose
after every robot ``set_pose`` (Habitat agent-sensor parity).
"""

from __future__ import annotations

import logging
import math
from typing import Any, Optional, Tuple

import numpy as np

from autosim.backends.isaac.app import update_app
from autosim.backends.isaac.coords import map_to_world

logger = logging.getLogger(__name__)


class IsaacSensors:
    """Read RGB-D / lidar from an Isaac Camera (or raw annotators).

    When no camera is mounted, methods return empty / placeholder arrays so
    Sensors / Runner keep working (pose-only smoke).
    """

    # Keep far clip in the robotics band. Isaac defaults (~1e5 m) produce
    # sparse far hits that make autoviz depth colorization flicker.
    MIN_DEPTH_M = 0.05
    MAX_DEPTH_M = 40.0

    def __init__(
        self,
        *,
        width: int = 640,
        height: int = 480,
        eye_height: float = 0.5,
        eye_offset: Optional[Tuple[float, float, float]] = None,
    ) -> None:
        self.width = max(1, int(width))
        self.height = max(1, int(height))
        # Local camera pose in base_link (URDF camera_link). Z-only legacy
        # (0,0,eye_height) puts Beni's eye inside the body mesh → blank RGB-D.
        if eye_offset is None:
            self.eye_offset = (0.0, 0.0, float(eye_height))
        else:
            self.eye_offset = (
                float(eye_offset[0]),
                float(eye_offset[1]),
                float(eye_offset[2]),
            )
        self.eye_height = float(self.eye_offset[2])
        self.camera_prim: Any = None
        self.rgb_annotator: Any = None
        self.depth_annotator: Any = None
        self.lidar_annotator: Any = None

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
            self.camera_prim = camera
        self.rgb_annotator = rgb
        self.depth_annotator = depth
        self.lidar_annotator = lidar

    def mount_robot_camera(
        self,
        robot_prim_path: str,
        *,
        hfov_deg: float = 90.0,
        rate_hz: float = 30.0,
        prim_suffix: str = "Camera",
        warmup_frames: int = 8,
    ) -> bool:
        """Spawn a forward-looking Camera under ``robot_prim`` and attach it.

        Args:
            robot_prim_path: Parent Xform (e.g. ``/Root/Robot``).
            hfov_deg: Horizontal field of view in degrees.
            rate_hz: Desired acquire rate (``camera.rate_hz``). Applied to the
                Isaac Camera only when it divides Kit ``rateLimitFrequency``;
                otherwise the sensor updates every render frame and AutoSim
                still publishes at ``rate_hz``.
            prim_suffix: Child prim name under the robot.
            warmup_frames: Kit updates so the first RGB/depth frames are valid.

        Returns:
            ``True`` when the camera was created and initialized.
        """
        try:
            from isaacsim.sensors.camera import Camera
        except ImportError as exc:
            logger.warning("isaacsim.sensors.camera unavailable: %s", exc)
            return False

        prim_path = f"{robot_prim_path.rstrip('/')}/{prim_suffix}"
        ox, oy, oz = self.eye_offset
        local_xyz = np.array([ox, oy, oz], dtype=np.float64)
        # arena-isaac: snap frequency to a divisor of the render rate so the
        # Camera ctor never raises; then FOV via focal_length (not aperture).
        freq = self.snap_camera_frequency(rate_hz)
        try:
            cam_kwargs: dict[str, Any] = {
                "prim_path": prim_path,
                "name": "autosim_rgb_camera",
                "resolution": (self.width, self.height),
                "translation": local_xyz.copy(),
                # world axes: +Z up, +X forward (matches autosim map/robot frame).
                "orientation": np.array(
                    [1.0, 0.0, 0.0, 0.0], dtype=np.float64
                ),
            }
            if freq is not None:
                cam_kwargs["frequency"] = int(freq)
            cam = Camera(**cam_kwargs)
            # Local pose under robot — follows parent like Habitat agent sensors.
            try:
                cam.set_local_pose(
                    translation=[float(ox), float(oy), float(oz)],
                    orientation=[1.0, 0.0, 0.0, 0.0],
                    camera_axes="world",
                )
            except Exception as exc:  # noqa: BLE001
                logger.debug("camera set_local_pose fallback: %s", exc)
            self.apply_horizontal_fov(
                cam, float(hfov_deg), width=self.width, height=self.height
            )
            try:
                # Small platforms (Beni) need a near clip below 0.1 m.
                cam.set_clipping_range(
                    near_distance=float(self.MIN_DEPTH_M),
                    far_distance=float(self.MAX_DEPTH_M),
                )
            except Exception as exc:  # noqa: BLE001
                logger.debug("could not set camera clipping range: %s", exc)
            cam.initialize(attach_rgb_annotator=True)
            cam.add_distance_to_image_plane_to_frame()
        except Exception as exc:  # noqa: BLE001 — Kit/Replicator failures vary
            logger.warning("failed to mount Isaac camera at %s: %s", prim_path, exc)
            return False

        self.camera_prim = cam
        for _ in range(max(0, int(warmup_frames))):
            update_app()
        logger.info(
            "Isaac camera mounted at %s (%dx%d, hfov=%.1f°, eye=(%.3f,%.3f,%.3f), "
            "freq=%s, publish≈%.0fHz)",
            prim_path,
            self.width,
            self.height,
            float(hfov_deg),
            float(ox),
            float(oy),
            float(oz),
            freq,
            float(rate_hz),
        )
        return True

    @staticmethod
    def snap_camera_frequency(
        rate_hz: float, *, default_render_hz: int = 60
    ) -> Optional[int]:
        """Nearest divisor of Kit render rate (arena-isaac ``_snap_rate``).

        Isaac ``Camera(frequency=…)`` raises unless
        ``rateLimitFrequency % frequency == 0``. Prefer Kit's configured rate;
        fall back to 60Hz (arena default) when rate limiting is disabled.
        """
        desired = int(round(float(rate_hz)))
        if desired <= 0:
            return None
        render = int(default_render_hz)
        try:
            import carb
            from isaacsim.core.utils.carb import get_carb_setting

            render_hz = get_carb_setting(
                carb.settings.get_settings(),
                "/app/runLoops/main/rateLimitFrequency",
            )
            if render_hz is not None and float(render_hz) > 0:
                render = int(round(float(render_hz)))
        except Exception:
            pass
        if render <= 0:
            return None
        divisors = [d for d in range(1, render + 1) if render % d == 0]
        if not divisors:
            return None
        snapped = min(divisors, key=lambda d: abs(d - desired))
        if snapped != desired:
            logger.info(
                "camera.rate_hz=%.0f → Camera frequency=%d (divisor of %dHz)",
                float(rate_hz),
                snapped,
                render,
            )
        return int(snapped)

    # Back-compat alias for older call sites / tests.
    safe_camera_frequency = snap_camera_frequency

    def sync_world_pose(
        self, x: float, y: float, yaw: float, *, base_z: float = 0.0
    ) -> None:
        """Drive RGB-D Camera to the robot eye pose in world frame.

        Isaac Camera / Replicator often keeps a stale world matrix when only the
        parent Xform moves under PhysX/Fabric — call after every ``set_pose``.
        """
        cam = self.camera_prim
        if cam is None:
            return
        ox, oy, oz = self.eye_offset
        cy = math.cos(float(yaw))
        sy = math.sin(float(yaw))
        # Rotate base_link offset into map XY; Z = base_z + local z.
        wx = float(x) + float(ox) * cy - float(oy) * sy
        wy = float(y) + float(ox) * sy + float(oy) * cy
        wz = float(base_z) + float(oz)
        translation, quat_wxyz = map_to_world(wx, wy, float(yaw), z=wz)
        try:
            cam.set_world_pose(
                position=translation,
                orientation=quat_wxyz,
                camera_axes="world",
            )
        except TypeError:
            try:
                cam.set_world_pose(translation, quat_wxyz)
            except Exception as exc:  # noqa: BLE001
                logger.debug("camera set_world_pose failed: %s", exc)
        except Exception as exc:  # noqa: BLE001
            logger.debug("camera set_world_pose failed: %s", exc)

    @staticmethod
    def apply_horizontal_fov(
        camera: Any,
        hfov_deg: float,
        *,
        width: int = 0,
        height: int = 0,
    ) -> None:
        """Set FOV by adjusting focal length (arena-isaac Camera path).

        ``hfov = 2 * atan(aperture / (2 * focal))`` ⇒
        ``focal = aperture / (2 * tan(hfov/2))``. Also matches vertical
        aperture to the pixel aspect so pixels stay square.
        """
        hfov = max(1.0, min(179.0, float(hfov_deg)))
        try:
            aperture = float(camera.get_horizontal_aperture())
            if aperture <= 0.0:
                aperture = 20.955
                camera.set_horizontal_aperture(aperture)
            half = math.radians(hfov) * 0.5
            focal = aperture / (2.0 * math.tan(half))
            camera.set_focal_length(float(focal))
            w = int(width) if int(width) > 0 else 0
            h = int(height) if int(height) > 0 else 0
            if w > 0 and h > 0:
                camera.set_vertical_aperture(aperture * float(h) / float(w))
        except Exception as exc:  # noqa: BLE001
            logger.debug("could not set camera hfov: %s", exc)

    def reshape_hw(self, arr: Any) -> np.ndarray:
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

    def sanitize_depth(self, arr: Any) -> np.ndarray:
        """Clip invalid / far-plane hits to NaN for stable 32FC1 visualization."""
        depth = self.reshape_hw(arr)
        valid = (
            np.isfinite(depth)
            & (depth > float(self.MIN_DEPTH_M))
            & (depth < float(self.MAX_DEPTH_M))
        )
        if np.all(valid):
            return depth
        out = depth.copy()
        out[~valid] = np.nan
        return out

    def close(self) -> None:
        """Detach annotators / destroy render product when present."""
        cam = self.camera_prim
        self.camera_prim = None
        self.rgb_annotator = None
        self.depth_annotator = None
        self.lidar_annotator = None
        if cam is None:
            return
        try:
            destroy = getattr(cam, "destroy", None)
            if callable(destroy):
                destroy()
        except Exception:
            pass

    def color_depth(self) -> Tuple[np.ndarray, np.ndarray]:
        color = self.read_rgb()
        depth = self.read_depth()
        return color, depth

    def read_rgb(self) -> np.ndarray:
        if self.camera_prim is not None:
            try:
                data = self.camera_prim.get_rgb(device="cpu")
            except Exception:
                data = None
            if data is not None:
                rgb = self.reshape_rgb(data)
                if rgb is not None:
                    return rgb
        if self.rgb_annotator is not None:
            rgb = self.reshape_rgb(self.rgb_annotator.get_data())
            if rgb is not None:
                return rgb
        out = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        out[..., 2] = 64
        return out

    def reshape_rgb(self, arr: Any) -> Optional[np.ndarray]:
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

    def read_depth(self) -> np.ndarray:
        if self.camera_prim is not None:
            try:
                data = self.camera_prim.get_depth(device="cpu")
            except Exception:
                data = None
            if data is not None and np.asarray(data).size > 0:
                return self.sanitize_depth(data)
        if self.depth_annotator is not None:
            data = self.depth_annotator.get_data()
            if data is not None and np.asarray(data).size > 0:
                return self.sanitize_depth(data)
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
        cloud = self.lidar_points_xy()
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
        cloud = self.lidar_points_xyz()
        if cloud is None or cloud.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=np.float32)
        dist = np.linalg.norm(cloud, axis=1)
        mask = dist <= float(range_max)
        cloud = cloud[mask]
        rings = np.zeros((cloud.shape[0],), dtype=np.float32)
        return cloud.astype(np.float32), rings

    def lidar_points_xy(self) -> Optional[np.ndarray]:
        xyz = self.lidar_points_xyz()
        if xyz is None:
            return None
        return xyz[:, :2]

    def lidar_points_xyz(self) -> Optional[np.ndarray]:
        if self.lidar_annotator is None:
            return None
        data = np.asarray(self.lidar_annotator.get_data(), dtype=np.float32)
        if data.size == 0:
            return np.zeros((0, 3), dtype=np.float32)
        return data.reshape(-1, data.shape[-1])[:, :3]
