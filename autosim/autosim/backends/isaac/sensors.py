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
from typing import Any

import numpy as np

from autosim.backends.isaac.app import update_app
from autosim.backends.isaac.transforms import map_to_world

logger = logging.getLogger(__name__)


class SensorSuite:
    """Read RGB-D / lidar from an Isaac Camera (or raw annotators).

    When no camera is mounted, methods return empty / placeholder arrays so
    SensorSuite / Runner keep working (pose-only smoke).
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
        eye_offset: tuple[float, float, float | None] = None,
    ) -> None:
        self.width = max(1, int(width))
        self.height = max(1, int(height))
        # Local camera pose in base_link (URDF camera_link). Z-only legacy
        # (0,0,eye_height) can put the eye inside the body mesh → blank RGB-D.
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
        self.lidar_sensor: Any = None
        self.lidar_flat_scan: bool = False
        self.imu_sensor: Any = None
        self.surround_cameras: list[Any] = []

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
                # Small platforms need a near clip below 0.1 m.
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
    def is_2d_lidar_config(config_name: str) -> bool:
        """True when ``config_name`` is safe for ``IsaacComputeRTXLidarFlatScan``.

        FlatScan requires elevationDeg == 0. ``Example_Rotary`` is 3D (−15°) and
        will abort the node; prefer RPLIDAR / SICK / Debug_Rotary / *_2D.
        """
        text = str(config_name or "").lower()
        if not text:
            return False
        if any(
            token in text
            for token in (
                "rplidar",
                "tim781",
                "picoscan",
                "microscan",
                "debug_rotary",
                "2d",
                "2dtof",
            )
        ):
            return True
        # Known 3D packs — do not attach FlatScan.
        if any(
            token in text
            for token in (
                "example_rotary",
                "velodyne",
                "hesai",
                "ouster",
                "vls",
                "xt32",
                "os0",
                "os1",
                "os2",
            )
        ):
            return "2d" in text
        return True

    def mount_robot_lidar(
        self,
        robot_prim_path: str,
        *,
        translation: tuple[float, float, float] = (0.0, 0.0, 0.2),
        config_name: str = "RPLIDAR_S2E",
        prim_suffix: str = "Lidar",
        warmup_frames: int = 4,
        flat_scan: bool | None = None,
    ) -> bool:
        """Mount an RTX Lidar under ``robot_prim`` (experimental API, then LidarRtx)."""
        prim_path = f"{robot_prim_path.rstrip('/')}/{prim_suffix}"
        tx, ty, tz = (float(v) for v in translation)
        config = str(config_name or "RPLIDAR_S2E")
        want_flat = (
            self.is_2d_lidar_config(config) if flat_scan is None else bool(flat_scan)
        )
        self.lidar_flat_scan = want_flat
        sensor: Any = None

        # Isaac Sim 6+: isaacsim.sensors.experimental.rtx
        try:
            from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

            authored = Lidar.create(
                prim_path,
                config=config,
                translations=np.array([[tx, ty, tz]], dtype=np.float64),
                orientations=np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
            )
            sensor = LidarSensor(authored, annotators=["generic-model-output"])
            self.lidar_sensor = sensor
            self.lidar_annotator = sensor
            logger.info(
                "RTX Lidar (experimental) at %s config=%s flat_scan=%s",
                prim_path,
                config,
                want_flat,
            )
        except Exception as exc:
            logger.debug("experimental Lidar unavailable: %s", exc)
            sensor = None

        if sensor is None:
            try:
                from isaacsim.sensors.rtx import LidarRtx

                sensor = LidarRtx(
                    prim_path=prim_path,
                    name="autosim_lidar",
                    translation=np.array([tx, ty, tz], dtype=np.float64),
                    orientation=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
                    config_file_name=config,
                )
                if want_flat:
                    try:
                        sensor.attach_annotator("IsaacComputeRTXLidarFlatScan")
                    except Exception as flat_exc:
                        logger.warning(
                            "FlatScan annotator failed (%s); falling back to point cloud",
                            flat_exc,
                        )
                        want_flat = False
                        self.lidar_flat_scan = False
                if not want_flat:
                    try:
                        sensor.attach_annotator(
                            "IsaacExtractRTXSensorPointCloudNoAccumulator"
                        )
                    except Exception as ann_exc:
                        logger.debug("LidarRtx point-cloud annotator skipped: %s", ann_exc)
                try:
                    sensor.initialize()
                except Exception:
                    pass
                self.lidar_sensor = sensor
                self.lidar_annotator = sensor
                logger.info(
                    "RTX LidarRtx at %s config=%s flat_scan=%s",
                    prim_path,
                    config,
                    self.lidar_flat_scan,
                )
            except Exception as exc:
                logger.warning("failed to mount RTX Lidar at %s: %s", prim_path, exc)
                return False

        for _ in range(max(0, int(warmup_frames))):
            update_app()
        return True

    def mount_robot_imu(
        self,
        robot_prim_path: str,
        *,
        translation: tuple[float, float, float] = (0.0, 0.0, 0.0),
        frequency: float = 200.0,
        prim_suffix: str = "Imu",
    ) -> bool:
        """Mount an Isaac PhysX ``IMUSensor`` under ``robot_prim``."""
        prim_path = f"{robot_prim_path.rstrip('/')}/{prim_suffix}"
        try:
            from isaacsim.core.utils.extensions import enable_extension
        except Exception:
            try:
                from omni.isaac.core.utils.extensions import enable_extension  # type: ignore
            except Exception:
                enable_extension = None  # type: ignore
        if enable_extension is not None:
            for ext in ("isaacsim.sensors.physics", "omni.isaac.sensor"):
                try:
                    enable_extension(ext)
                except Exception:
                    pass
        try:
            from isaacsim.sensors.physics import IMUSensor
        except Exception as exc:
            logger.warning("Isaac IMUSensor unavailable: %s", exc)
            return False
        try:
            imu = IMUSensor(
                prim_path=prim_path,
                name="autosim_imu",
                frequency=int(max(1.0, float(frequency))),
                translation=np.array(
                    [float(translation[0]), float(translation[1]), float(translation[2])],
                    dtype=np.float64,
                ),
            )
            try:
                imu.initialize()
            except Exception:
                pass
            self.imu_sensor = imu
            logger.info(
                "Isaac IMU at %s (%.0f Hz)", prim_path, float(frequency)
            )
            return True
        except Exception as exc:
            logger.warning("failed to mount Isaac IMU at %s: %s", prim_path, exc)
            return False

    def read_imu(self) -> dict[str, Any] | None:
        """Return Isaac IMU frame ``{lin_acc, ang_vel, orientation}`` or None."""
        imu = self.imu_sensor
        if imu is None:
            return None
        try:
            frame = imu.get_current_frame(read_gravity=True)
        except TypeError:
            try:
                frame = imu.get_current_frame()
            except Exception:
                return None
        except Exception:
            return None
        if not isinstance(frame, dict):
            return None
        try:
            lin = np.asarray(frame.get("lin_acc"), dtype=np.float64).reshape(-1)
            ang = np.asarray(frame.get("ang_vel"), dtype=np.float64).reshape(-1)
            quat = np.asarray(frame.get("orientation"), dtype=np.float64).reshape(-1)
        except Exception:
            return None
        if lin.size < 3 or ang.size < 3:
            return None
        # Isaac quat is often wxyz; Autolink Imu uses xyzw.
        if quat.size >= 4:
            orientation = (
                float(quat[1]),
                float(quat[2]),
                float(quat[3]),
                float(quat[0]),
            )
        else:
            orientation = (0.0, 0.0, 0.0, 1.0)
        return {
            "lin_acc": (float(lin[0]), float(lin[1]), float(lin[2])),
            "ang_vel": (float(ang[0]), float(ang[1]), float(ang[2])),
            "orientation": orientation,
        }

    def mount_surround_cameras(
        self,
        robot_prim_path: str,
        views: list[dict[str, Any]],
        *,
        rate_hz: float = 15.0,
        warmup_frames: int = 4,
    ) -> int:
        """Mount extra surround cameras; returns how many succeeded.

        Each view dict: ``name``, ``yaw_deg`` (relative to base), optional
        ``xyz``, ``hfov_deg``, ``width``, ``height``.
        """
        mounted = 0
        saved = (
            self.camera_prim,
            self.width,
            self.height,
            self.eye_offset,
        )
        for index, view in enumerate(views or []):
            if not isinstance(view, dict):
                continue
            name = str(view.get("name") or f"Surround{index}")
            yaw_deg = float(view.get("yaw_deg", 0.0))
            xyz = view.get("xyz") or view.get("sensor_xyz")
            if isinstance(xyz, (list, tuple)) and len(xyz) >= 3:
                self.eye_offset = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
            w = int(view.get("width") or self.width)
            h = int(view.get("height") or self.height)
            self.width, self.height = max(1, w), max(1, h)
            # Temporarily clear primary so mount does not overwrite tracking.
            primary = self.camera_prim
            self.camera_prim = None
            ok = self.mount_robot_camera(
                robot_prim_path,
                hfov_deg=float(view.get("hfov_deg", 90.0)),
                rate_hz=float(rate_hz),
                prim_suffix=name,
                warmup_frames=0,
            )
            cam = self.camera_prim
            self.camera_prim = primary
            if not ok or cam is None:
                continue
            # Yaw offset about +Z in local frame (simple surround ring).
            try:
                half = 0.5 * math.radians(yaw_deg)
                quat = [math.cos(half), 0.0, 0.0, math.sin(half)]
                ox, oy, oz = self.eye_offset
                cam.set_local_pose(
                    translation=[ox, oy, oz],
                    orientation=quat,
                    camera_axes="world",
                )
            except Exception as exc:
                logger.debug("surround local pose skipped: %s", exc)
            self.surround_cameras.append(cam)
            mounted += 1
        self.camera_prim, self.width, self.height, self.eye_offset = saved
        for _ in range(max(0, int(warmup_frames))):
            update_app()
        if mounted:
            logger.info("mounted %d surround cameras under %s", mounted, robot_prim_path)
        return mounted

    def read_surround_rgb(self) -> list[np.ndarray]:
        """RGB frames from surround cameras (may be empty placeholders)."""
        frames: list[np.ndarray] = []
        for cam in self.surround_cameras:
            try:
                data = cam.get_rgb(device="cpu")
            except Exception:
                data = None
            rgb = self.reshape_rgb(data) if data is not None else None
            if rgb is None:
                rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            frames.append(rgb)
        return frames

    @staticmethod
    def snap_camera_frequency(
        rate_hz: float, *, default_render_hz: int = 60
    ) -> int | None:
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
        cams = [self.camera_prim, *list(self.surround_cameras)]
        self.camera_prim = None
        self.surround_cameras = []
        self.rgb_annotator = None
        self.depth_annotator = None
        self.lidar_annotator = None
        lidar = self.lidar_sensor
        self.lidar_sensor = None
        imu = self.imu_sensor
        self.imu_sensor = None
        if imu is not None:
            try:
                destroy = getattr(imu, "destroy", None) or getattr(imu, "cleanup", None)
                if callable(destroy):
                    destroy()
            except Exception:
                pass
        for cam in cams:
            if cam is None:
                continue
            try:
                destroy = getattr(cam, "destroy", None)
                if callable(destroy):
                    destroy()
            except Exception:
                pass
        if lidar is not None:
            try:
                destroy = getattr(lidar, "destroy", None)
                if callable(destroy):
                    destroy()
            except Exception:
                pass

    def color_depth(self) -> tuple[np.ndarray, np.ndarray]:
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

    def reshape_rgb(self, arr: Any) -> np.ndarray | None:
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

    def _flat_scan_ranges(
        self,
        angle_min: float,
        angle_max: float,
        num_beams: int,
        range_max: float,
    ) -> np.ndarray | None:
        """Read ``IsaacComputeRTXLidarFlatScan`` depths into a fixed beam grid."""
        sensor = self.lidar_sensor if self.lidar_sensor is not None else self.lidar_annotator
        if sensor is None:
            return None
        payload: Any = None
        try:
            frame = sensor.get_current_frame()
            if isinstance(frame, dict):
                payload = frame.get("IsaacComputeRTXLidarFlatScan")
                if payload is None:
                    for key in ("linear_depth_data", "depths", "ranges", "data"):
                        if key in frame and frame[key] is not None:
                            payload = frame[key]
                            break
        except Exception:
            payload = None
        if payload is None:
            return None
        depths: np.ndarray | None = None
        azimuths: np.ndarray | None = None
        if isinstance(payload, dict):
            for key in (
                "linearDepth",
                "linear_depth",
                "depths",
                "ranges",
                "distances",
                "data",
            ):
                if key in payload and payload[key] is not None:
                    depths = np.asarray(payload[key], dtype=np.float64).reshape(-1)
                    break
            for key in ("azimuths", "azimuth", "angles"):
                if key in payload and payload[key] is not None:
                    azimuths = np.asarray(payload[key], dtype=np.float64).reshape(-1)
                    break
        else:
            arr = np.asarray(payload, dtype=np.float64)
            if arr.ndim == 1 and arr.size > 0:
                depths = arr.reshape(-1)
        if depths is None or depths.size == 0:
            return None
        out = np.full((int(num_beams),), np.nan, dtype=np.float32)
        target = np.linspace(
            float(angle_min), float(angle_max), num=int(num_beams), dtype=np.float64
        )
        if azimuths is not None and azimuths.size == depths.size:
            step = (float(angle_max) - float(angle_min)) / max(int(num_beams) - 1, 1)
            for ang, dist in zip(azimuths, depths):
                if not np.isfinite(dist) or float(dist) > float(range_max):
                    continue
                # Wrap azimuth into [angle_min, angle_max] neighborhood.
                delta = ((float(ang) - target + math.pi) % (2 * math.pi)) - math.pi
                index = int(np.argmin(np.abs(delta)))
                if abs(float(delta[index])) > 0.5 * step + 1e-3:
                    continue
                prev = out[index]
                if not np.isfinite(prev) or float(dist) < float(prev):
                    out[index] = float(dist)
            return out
        # Densities often match num_beams already (or resample).
        if depths.size == int(num_beams):
            clipped = np.where(
                np.isfinite(depths) & (depths <= float(range_max)),
                depths,
                np.nan,
            )
            return clipped.astype(np.float32)
        # Nearest-neighbor resample onto requested beam count.
        src = np.linspace(
            float(angle_min), float(angle_max), num=int(depths.size), dtype=np.float64
        )
        for i, ang in enumerate(target):
            index = int(np.argmin(np.abs(src - ang)))
            dist = float(depths[index])
            if np.isfinite(dist) and dist <= float(range_max):
                out[i] = dist
        return out

    def laser_ranges(
        self,
        angle_min: float,
        angle_max: float,
        num_beams: int,
        range_max: float,
        *,
        pose: tuple[float, float, float],
    ) -> np.ndarray:
        """Prefer FlatScan / lidar annotator; else empty (NaN) planar scan."""
        del pose
        flat = self._flat_scan_ranges(angle_min, angle_max, num_beams, range_max)
        if flat is not None and np.isfinite(flat).any():
            return flat
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
    ) -> tuple[np.ndarray, np.ndarray]:
        del h_min, h_max, h_beams, v_min, v_max, v_rings
        cloud = self.lidar_points_xyz()
        if cloud is None or cloud.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=np.float32)
        dist = np.linalg.norm(cloud, axis=1)
        mask = dist <= float(range_max)
        cloud = cloud[mask]
        rings = np.zeros((cloud.shape[0],), dtype=np.float32)
        return cloud.astype(np.float32), rings

    def lidar_points_xy(self) -> np.ndarray | None:
        xyz = self.lidar_points_xyz()
        if xyz is None:
            return None
        return xyz[:, :2]

    def lidar_points_xyz(self) -> np.ndarray | None:
        sensor = self.lidar_sensor if self.lidar_sensor is not None else self.lidar_annotator
        if sensor is None:
            return None
        # LidarSensor (experimental): get_data("generic-model-output")
        try:
            packed = sensor.get_data("generic-model-output")
            if isinstance(packed, tuple) and packed:
                data = packed[0]
            else:
                data = packed
            cloud = self._points_from_gmo(data)
            if cloud is not None:
                return cloud
        except Exception:
            pass
        # LidarRtx: get_current_frame / get_data dict
        try:
            frame = sensor.get_current_frame()
            if isinstance(frame, dict):
                for key in (
                    "IsaacComputeRTXLidarFlatScan",
                    "linear_depth_data",
                    "point_cloud_data",
                    "data",
                ):
                    if key in frame and frame[key] is not None:
                        cloud = self._coerce_point_cloud(frame[key])
                        if cloud is not None:
                            return cloud
        except Exception:
            pass
        try:
            data = sensor.get_data()
            cloud = self._coerce_point_cloud(data)
            if cloud is not None:
                return cloud
        except Exception:
            pass
        if self.lidar_annotator is not None and self.lidar_annotator is not sensor:
            try:
                data = np.asarray(self.lidar_annotator.get_data(), dtype=np.float32)
                return self._coerce_point_cloud(data)
            except Exception:
                pass
        return None

    @staticmethod
    def _coerce_point_cloud(data: Any) -> np.ndarray | None:
        if data is None:
            return None
        arr = np.asarray(data)
        if arr.size == 0:
            return np.zeros((0, 3), dtype=np.float32)
        if arr.ndim == 1 and arr.size % 3 == 0:
            return arr.reshape(-1, 3).astype(np.float32, copy=False)
        if arr.ndim >= 2 and arr.shape[-1] >= 3:
            return arr.reshape(-1, arr.shape[-1])[:, :3].astype(np.float32, copy=False)
        # FlatScan depths → synthesize XY ring later in laser_ranges.
        if arr.ndim == 1:
            return None
        return None

    @staticmethod
    def _points_from_gmo(data: Any) -> np.ndarray | None:
        """Best-effort GenericModelOutput → Nx3."""
        try:
            from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data

            gmo = parse_generic_model_output_data(data)
            for attr in ("azimElevRangeToPointCloud", "points", "xyz"):
                pts = getattr(gmo, attr, None)
                if pts is not None:
                    cloud = SensorSuite._coerce_point_cloud(pts)
                    if cloud is not None:
                        return cloud
        except Exception:
            pass
        return SensorSuite._coerce_point_cloud(data)
