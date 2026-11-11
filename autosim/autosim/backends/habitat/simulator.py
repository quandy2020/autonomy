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

"""Habitat-Sim session wrapper for agent pose and ``cast_ray`` sensing.

Planar bridge coordinates map to Habitat Y-up: planar ``y`` → world ``z``.
Requires ``habitat-sim``; there is no mock backend.
"""

from __future__ import annotations

import math
import shutil
import warnings
from pathlib import Path
from typing import Any, Mapping, Tuple

import numpy as np

from autosim.bridge.cameras import (
    SurroundCamera,
    camera_rig,
    projection_height,
    resolve_surround,
    ros_to_habitat_mount,
)
from autosim.bridge.chase import CHASE_UUID, chase_spec
from autosim.bridge.stitch import PANORAMA_UUID, panorama_spec
from autosim.bridge.topdown import TOPDOWN_UUID, topdown_spec
from autosim.robot.urdf import UrdfModel


class Simulator:
    """Scene and agent pose backend (Habitat only)."""

    def __init__(
        self,
        backend: str,
        width: int,
        height: int,
        settings: Mapping[str, Any] | None = None,
        open_session: bool = True,
    ) -> None:
        """Construct the simulator.

        Args:
            backend: ``minimal`` (empty path) or ``habitat`` (scene file).
            width: Camera width in pixels.
            height: Camera height in pixels.
            settings: Full config map (needs ``habitat``).
            open_session: When ``False``, skip Habitat open (unit tests / URDF only).

        Raises:
            ImportError: Habitat required but not installed.
            FileNotFoundError: Scene path invalid.
        """
        self.backend = backend
        self.width = int(width)
        self.height = int(height)
        self.settings = dict(settings or {})
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.floor_y = 0.0
        self.session = None
        self.articulated = None
        self.urdf = self.load_urdf_model()
        self.traffic = None
        self.humanoid_sync = None
        robot = self.settings.get("habitat", {}).get("robot", {})
        self.block_motion = bool(robot.get("block_motion", True)) if isinstance(
            robot, Mapping
        ) else True
        self.map_builder = None

        if open_session:
            self.open_habitat()

    @classmethod
    def create(cls, settings: Mapping[str, Any]) -> "Simulator":
        """Factory from config; always opens Habitat.

        Args:
            settings: Validated config (e.g. ``Config.data``).

        Returns:
            Ready :class:`Simulator`.

        Raises:
            ImportError: ``habitat-sim`` missing.
            FileNotFoundError: Scene path set but missing on disk.
        """
        habitat = settings["habitat"]
        scene_path = str(habitat.get("path") or "").strip()
        backend = "minimal" if not scene_path else "habitat"
        camera = habitat["sensors"]["camera"]
        return cls(
            backend=backend,
            width=int(camera["width"]),
            height=int(camera["height"]),
            settings=settings,
            open_session=True,
        )

    def load_urdf_model(self) -> UrdfModel | None:
        """Parse ``habitat.robot.urdf`` mounts when configured.

        Returns:
            :class:`UrdfModel` or ``None``.
        """
        robot = self.settings.get("habitat", {}).get("robot", {})
        urdf = robot.get("urdf", "") if isinstance(robot, Mapping) else ""
        try:
            return UrdfModel.load(str(urdf or ""))
        except (FileNotFoundError, ValueError) as exc:
            warnings.warn(f"URDF mounts unavailable ({exc!r})", stacklevel=2)
            return None

    def open_habitat(self) -> None:
        """Open Habitat, load optional URDF, and apply spawn pose.

        Raises:
            ImportError: ``habitat-sim`` missing.
            FileNotFoundError: Required scene path missing on disk.
        """
        try:
            import habitat_sim
        except ImportError as exc:
            raise ImportError(
                "habitat-sim is not installed. Install with: pip install habitat-sim"
            ) from exc

        configuration = self.habitat_configuration(habitat_sim)
        agent_configuration = self.agent_configuration(habitat_sim)
        self.session = habitat_sim.Simulator(
            habitat_sim.Configuration(configuration, [agent_configuration])
        )
        try:
            self.session.set_stage_is_collidable(True)
        except Exception:
            pass
        self.attach_urdf(habitat_sim)
        spawn = self.settings["habitat"].get("spawn", [0.0, 0.0, 0.0])
        self.reset(float(spawn[0]), float(spawn[1]), float(spawn[2]))
        self.snap_to_navmesh()
        self.ensure_semantic_spawn()
        self.attach_humanoids()

    def habitat_configuration(self, habitat_sim: Any) -> Any:
        """Build ``SimulatorConfiguration`` from settings.

        Supports two loading modes (mirrors autonomy_ros habitat/sim.py _open()):
        1. MP3D dataset config mode: set ``habitat.dataset_config`` (path to
           ``mp3d.scene_dataset_config.json``) and ``habitat.scene_id`` (e.g.
           ``17DRP5sb8fy/17DRP5sb8fy.glb``).  Enables semantic mesh loading.
        2. Direct GLB mode (legacy): set ``habitat.path`` to the ``.glb`` file.
        """
        configuration = habitat_sim.SimulatorConfiguration()
        configuration.gpu_device_id = int(self.settings["habitat"]["gpu"])
        # cast_ray (Lidar/map) requires Bullet physics.
        configuration.enable_physics = True
        configuration.allow_sliding = True
        configuration.requires_textures = True

        hab_cfg = self.settings.get("habitat", {})
        dataset_config = str(hab_cfg.get("dataset_config") or "").strip()
        scene_id = str(hab_cfg.get("scene_id") or "").strip()
        scene_path = str(hab_cfg.get("path") or "").strip()

        resolved = self.resolve_scene_dataset(hab_cfg)
        if resolved is not None:
            dataset_file, resolved_scene_id = resolved
            configuration.scene_dataset_config_file = dataset_file
            configuration.scene_id = resolved_scene_id
            configuration.load_semantic_mesh = True
            return configuration

        if dataset_config and scene_id:
            raise FileNotFoundError(
                f"MP3D scene {scene_id!r} not found relative to dataset config "
                f"{dataset_config!r}. Place mp3d.scene_dataset_config.json beside "
                "the scene folders (e.g. .../src/mp3d.scene_dataset_config.json) "
                "or set habitat.path to the .glb for auto-detection."
            )

        # Legacy: direct GLB path (no semantic mesh).
        if not scene_path:
            stage = self.install_scenario(hab_cfg)
            if stage is not None:
                configuration.scene_id = stage
                return configuration
            if hasattr(habitat_sim, "STAGE_EMPTY_SCENE"):
                configuration.scene_id = habitat_sim.STAGE_EMPTY_SCENE
            else:
                configuration.scene_id = habitat_sim.utils.settings.default_sim_settings.get(
                    "scene", "NONE"
                )
            return configuration
        glb_path = Path(scene_path)
        if not glb_path.exists():
            raise FileNotFoundError(f"scene path does not exist: {scene_path}")

        camera_cfg = hab_cfg.get("sensors", {}).get("camera", {})
        if camera_cfg.get("semantic_enabled", False):
            warnings.warn(
                "semantic_enabled but no MP3D semantic assets beside the GLB "
                f"({glb_path.stem}_semantic.ply + {glb_path.stem}.house); "
                "semantic sensor will not align with RGB. Use dataset_config + "
                "scene_id beside the scene folders, or habitat.path auto-detection.",
                stacklevel=2,
            )

        configuration.scene_id = scene_path
        return configuration

    def install_scenario(self, hab_cfg: Mapping[str, Any]) -> str | None:
        """Build a mockamap stage when ``habitat.path`` is empty and scenario is on.

        Args:
            hab_cfg: ``habitat`` mapping.

        Returns:
            OBJ path for Habitat ``scene_id``, or ``None`` when scenario is off.
        """
        block = hab_cfg.get("scenario")
        if not isinstance(block, Mapping) or not block.get("enabled", False):
            return None
        from autosim.scene import Scenario

        world = Scenario(block).build()
        self.scenario_points = world.points
        self.scenario_grid = world.grid
        return str(world.mesh)

    @classmethod
    def resolve_scene_dataset(cls, hab_cfg: Mapping[str, Any]) -> Tuple[str, str] | None:
        """Resolve MP3D dataset config + scene_id from yaml or GLB path.

        Habitat resolves ``*/*.glb`` globs relative to the directory that
        contains ``mp3d.scene_dataset_config.json``, not the autosim package.
        """
        dataset_config = str(hab_cfg.get("dataset_config") or "").strip()
        scene_id = str(hab_cfg.get("scene_id") or "").strip()
        scene_path = str(hab_cfg.get("path") or "").strip()

        if dataset_config and scene_id:
            ds_path = Path(dataset_config)
            if ds_path.is_file() and (ds_path.parent / scene_id).is_file():
                return str(ds_path), scene_id

        if scene_path:
            glb_path = Path(scene_path)
            if glb_path.is_file():
                mp3d = cls.resolve_mp3d_scene_dataset(glb_path)
                if mp3d is not None:
                    return mp3d

        return None

    @staticmethod
    def bundled_mp3d_dataset_config() -> Path:
        """Packaged ``mp3d.scene_dataset_config.json`` (same as autonomy_ros)."""
        from autosim.runtime.paths import project_root

        return project_root() / "config" / "mp3d.scene_dataset_config.json"

    @classmethod
    def ensure_mp3d_dataset_config(cls, mp3d_root: Path) -> Path:
        """Install dataset config beside MP3D scene folders when missing."""
        dest = mp3d_root / "mp3d.scene_dataset_config.json"
        if dest.is_file():
            return dest
        bundled = cls.bundled_mp3d_dataset_config()
        if not bundled.is_file():
            raise FileNotFoundError(
                f"MP3D scene dataset config missing at {dest} and bundled copy "
                f"not found at {bundled}"
            )
        mp3d_root.mkdir(parents=True, exist_ok=True)
        shutil.copy2(bundled, dest)
        return dest

    @classmethod
    def resolve_mp3d_scene_dataset(cls, glb_path: Path) -> Tuple[str, str] | None:
        """Infer dataset config + scene_id from ``{scene}/{scene}.glb`` MP3D layout.

        Habitat semantic cameras require ``{scene}_semantic.ply`` and ``{scene}.house``
        registered via ``mp3d.scene_dataset_config.json`` (not raw GLB loading).
        """
        if glb_path.suffix.lower() != ".glb":
            return None
        scene_dir = glb_path.parent
        scene_name = glb_path.stem
        semantic_ply = scene_dir / f"{scene_name}_semantic.ply"
        house_file = scene_dir / f"{scene_name}.house"
        if not semantic_ply.is_file() or not house_file.is_file():
            return None
        mp3d_root = scene_dir.parent
        dataset_config = cls.ensure_mp3d_dataset_config(mp3d_root)
        scene_id = f"{scene_name}/{glb_path.name}"
        return str(dataset_config), scene_id

    def camera_semantic_enabled(self) -> bool:
        """Whether the Habitat agent includes a semantic pinhole sensor."""
        camera_cfg = self.settings.get("habitat", {}).get("sensors", {}).get("camera", {})
        return bool(camera_cfg.get("semantic_enabled", False))

    def semantic_unique_count(self) -> int:
        """Count distinct Habitat semantic / instance ids in the current view."""
        if self.session is None:
            return 0
        observations = self.session.get_sensor_observations()
        semantic_key = None
        for name in ("semantic", "semantic_sensor"):
            if name in observations:
                semantic_key = name
                break
        if semantic_key is None:
            return 0
        semantic = np.squeeze(np.asarray(observations[semantic_key]))
        return int(len(np.unique(semantic)))

    def ensure_semantic_spawn(
        self,
        *,
        min_unique: int = 20,
        max_tries: int = 48,
    ) -> bool:
        """Relocate spawn when the semantic buffer is degenerate at the configured pose.

        MP3D semantic meshes can render as a flat id field at some navigable poses
        (e.g. map origin) while RGB still looks valid.  Sample random navmesh points
        and keep the pose with the richest semantic view.

        Returns:
            ``True`` when the agent pose was changed.
        """
        if self.session is None or not self.camera_semantic_enabled():
            return False

        initial = self.semantic_unique_count()
        if initial >= min_unique:
            return False

        pathfinder = getattr(self.session, "pathfinder", None)
        random_nav = getattr(pathfinder, "get_random_navigable_point", None)
        if pathfinder is None or not callable(random_nav):
            warnings.warn(
                "semantic_enabled but semantic view has only "
                f"{initial} ids and navmesh is unavailable for respawn",
                stacklevel=2,
            )
            return False
        if not bool(getattr(pathfinder, "is_loaded", False)):
            warnings.warn(
                "semantic_enabled but semantic view has only "
                f"{initial} ids and navmesh is not loaded",
                stacklevel=2,
            )
            return False

        best_count = initial
        best_pose = self.pose()
        spawn_yaw = best_pose[2]
        for _ in range(max(1, int(max_tries))):
            try:
                sample = self.finite_xyz(random_nav())
            except Exception:
                sample = None
            if sample is None:
                continue
            map_x = float(sample[0])
            map_y = float(-sample[2])
            self.set_pose(map_x, map_y, spawn_yaw)
            self.snap_to_navmesh()
            count = self.semantic_unique_count()
            if count > best_count:
                best_count = count
                best_pose = self.pose()
            if count >= min_unique:
                break

        if best_count <= initial:
            warnings.warn(
                "semantic_enabled but could not find a spawn with rich semantic "
                f"labels (best={best_count}, need>={min_unique}); "
                "segmentation may not align with RGB at this pose",
                stacklevel=2,
            )
            self.set_pose(*best_pose)
            return False

        self.set_pose(*best_pose)
        warnings.warn(
            "Relocated spawn for semantic camera: "
            f"unique ids {initial} -> {best_count} at "
            f"({best_pose[0]:.2f}, {best_pose[1]:.2f}, yaw={best_pose[2]:.2f})",
            stacklevel=2,
        )
        return True

    @staticmethod
    def semantic_ids_to_colored_rgb(semantic: np.ndarray) -> np.ndarray:
        """Colorize Habitat semantic ids for human viewing (matches autonomy_ros)."""
        ids = np.squeeze(np.asarray(semantic))
        if ids.ndim != 2:
            raise RuntimeError(f"unexpected semantic shape: {ids.shape!r}")
        try:
            from habitat_sim.utils.viz_utils import semantic_to_rgb
        except ImportError as exc:
            raise RuntimeError("habitat-sim is required for semantic visualization") from exc
        colored = np.asarray(semantic_to_rgb(ids.astype(np.uint32, copy=False)))
        if colored.ndim == 3 and colored.shape[-1] >= 3:
            return np.ascontiguousarray(colored[..., :3].astype(np.uint8, copy=False))
        raise RuntimeError(f"unexpected semantic_to_rgb shape: {colored.shape!r}")

    @staticmethod
    def semantic_bgr_to_colored_rgb(semantic_bgr: np.ndarray) -> np.ndarray:
        """Decode packed BGR8 ids and colorize for visualization."""
        bgr = np.asarray(semantic_bgr)
        ids = (
            bgr[..., 0].astype(np.uint32)
            | (bgr[..., 1].astype(np.uint32) << 8)
            | (bgr[..., 2].astype(np.uint32) << 16)
        )
        return Simulator.semantic_ids_to_colored_rgb(ids)

    def agent_configuration(self, habitat_sim: Any) -> Any:
        """RGB-D agent specs; camera height from URDF when present."""
        camera_cfg = self.settings.get("habitat", {}).get("sensors", {}).get("camera", {})
        hfov_deg = float(camera_cfg.get("hfov_deg", 90.0))
        agent_configuration = habitat_sim.agent.AgentConfiguration()
        color_sensor = habitat_sim.CameraSensorSpec()
        color_sensor.uuid = "rgb"
        color_sensor.sensor_type = habitat_sim.SensorType.COLOR
        color_sensor.resolution = [self.height, self.width]
        color_sensor.hfov = hfov_deg
        depth_sensor = habitat_sim.CameraSensorSpec()
        depth_sensor.uuid = "depth"
        depth_sensor.sensor_type = habitat_sim.SensorType.DEPTH
        depth_sensor.resolution = [self.height, self.width]
        depth_sensor.hfov = hfov_deg
        if hasattr(habitat_sim, "SensorSubType"):
            pinhole = habitat_sim.SensorSubType.PINHOLE
            color_sensor.sensor_subtype = pinhole
            depth_sensor.sensor_subtype = pinhole
        offset = self.camera_local_offset()
        color_sensor.position = [offset[0], offset[1], offset[2]]
        depth_sensor.position = [offset[0], offset[1], offset[2]]
        # Match autonomy_ros sim.engine_sensors(): orientation [0,0,0]. Agent quaternion
        # Ry(yaw - π/2) maps pinhole look (-Z) onto ROS/map +X forward.
        color_sensor.orientation = [0.0, 0.0, 0.0]
        depth_sensor.orientation = [0.0, 0.0, 0.0]
        for spec in (color_sensor, depth_sensor):
            if hasattr(spec, "near"):
                spec.near = 0.05
            if hasattr(spec, "far"):
                spec.far = 100.0
        sensor_specs = []
        if camera_cfg.get("enabled", True):
            sensor_specs.extend([color_sensor, depth_sensor])
            if camera_cfg.get("semantic_enabled", False):
                semantic_sensor = habitat_sim.CameraSensorSpec()
                semantic_sensor.uuid = "semantic"
                semantic_sensor.sensor_type = habitat_sim.SensorType.SEMANTIC
                semantic_sensor.resolution = [self.height, self.width]
                semantic_sensor.hfov = hfov_deg
                if hasattr(habitat_sim, "SensorSubType"):
                    semantic_sensor.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
                semantic_sensor.position = [offset[0], offset[1], offset[2]]
                semantic_sensor.orientation = [0.0, 0.0, 0.0]
                sensor_specs.append(semantic_sensor)
        sensors = self.settings.get("habitat", {}).get("sensors")
        surround = camera_rig(sensors)
        for camera in resolve_surround(surround, urdf=self.urdf):
            sensor_specs.append(self.surround_color_spec(habitat_sim, camera))
        pano = panorama_spec(sensors)
        if pano is not None:
            sensor_specs.append(self.panorama_color_spec(habitat_sim, pano))
        topdown = topdown_spec(sensors if isinstance(sensors, Mapping) else None)
        if topdown is not None:
            sensor_specs.append(self.topdown_color_spec(habitat_sim, topdown))
        chase = chase_spec(sensors if isinstance(sensors, Mapping) else None)
        if chase is not None:
            sensor_specs.append(self.chase_color_spec(habitat_sim, chase))
        agent_configuration.sensor_specifications = sensor_specs
        return agent_configuration

    @staticmethod
    def surround_color_spec(habitat_sim: Any, camera: SurroundCamera) -> Any:
        """Pinhole color spec aimed at ``camera.yaw`` in the agent frame.

        Habitat ``orientation`` is Euler radians; Y is yaw about the up axis.
        Positive Y matches ROS yaw (left).
        """
        spec = habitat_sim.CameraSensorSpec()
        spec.uuid = camera.uuid
        spec.sensor_type = habitat_sim.SensorType.COLOR
        render_h = projection_height(
            camera.width, camera.height, camera.hfov_deg, camera.vfov_deg
        )
        spec.resolution = [int(render_h), int(camera.width)]
        spec.hfov = float(camera.hfov_deg)
        if hasattr(habitat_sim, "SensorSubType"):
            subtypes = {
                "pinhole": habitat_sim.SensorSubType.PINHOLE,
                "fisheye": habitat_sim.SensorSubType.FISHEYE,
                "equirectangular": habitat_sim.SensorSubType.EQUIRECTANGULAR,
            }
            spec.sensor_subtype = subtypes[camera.model]
        ax, ay, az = ros_to_habitat_mount(camera.x, camera.y, camera.z)
        spec.position = [ax, ay, az]
        spec.orientation = [0.0, float(camera.yaw), 0.0]
        if hasattr(spec, "near"):
            spec.near = 0.05
        if hasattr(spec, "far"):
            spec.far = 100.0
        return spec

    @staticmethod
    def panorama_color_spec(habitat_sim: Any, pano: Mapping[str, Any]) -> Any:
        """Equirectangular color camera. The image is a full 360°×180° sphere.

        Resolution is ``[height, width]``. A 2:1 image is the usual panorama.
        The center column looks along the same forward axis as ``cam_front``.
        """
        spec = habitat_sim.EquirectangularSensorSpec()
        spec.uuid = PANORAMA_UUID
        spec.sensor_type = habitat_sim.SensorType.COLOR
        spec.resolution = [int(pano["height"]), int(pano["width"])]
        ax, ay, az = ros_to_habitat_mount(*pano["xyz"])
        spec.position = [ax, ay, az]
        spec.orientation = [0.0, 0.0, 0.0]
        if hasattr(spec, "near"):
            spec.near = 0.05
        if hasattr(spec, "far"):
            spec.far = 100.0
        return spec

    @staticmethod
    def topdown_color_spec(habitat_sim: Any, topdown: Mapping[str, Any]) -> Any:
        """Overhead color camera looking straight down (global debug / people).

        Default ``orthographic`` + runtime ``fit_scene`` covers the navmesh AABB.
        Pitch −π/2 so the pinhole −Z axis aims at Habitat −Y (floor).
        """
        spec = habitat_sim.CameraSensorSpec()
        spec.uuid = str(topdown.get("uuid") or TOPDOWN_UUID)
        spec.sensor_type = habitat_sim.SensorType.COLOR
        spec.resolution = [int(topdown["height"]), int(topdown["width"])]
        spec.hfov = float(topdown["hfov_deg"])
        mode = str(topdown.get("mode") or "orthographic").lower()
        if hasattr(habitat_sim, "SensorSubType"):
            if mode == "orthographic":
                spec.sensor_subtype = habitat_sim.SensorSubType.ORTHOGRAPHIC
                # Habitat: visible ≈ 1/ortho_scale (NOT meters of half-extent).
                # Default 0.1 ≈ 10 m; larger scale zooms IN. Bootstrap wide view.
                scale = topdown.get("ortho_scale")
                if hasattr(spec, "ortho_scale"):
                    spec.ortho_scale = (
                        float(scale) if scale is not None else 0.04
                    )
            else:
                spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
        # Agent frame is Habitat Y-up: (0, h, 0) sits above the robot.
        spec.position = [0.0, float(topdown["height_m"]), 0.0]
        spec.orientation = [-0.5 * math.pi, 0.0, 0.0]
        if hasattr(spec, "near"):
            spec.near = 0.05
        if hasattr(spec, "far"):
            spec.far = 200.0
        return spec

    @staticmethod
    def chase_color_spec(habitat_sim: Any, chase: Mapping[str, Any]) -> Any:
        """Third-person pinhole behind/above the agent looking at the chassis.

        Habitat agent frame: ``-Z`` forward, ``+Y`` up → mount at
        ``(0, height, distance)`` with a downward pitch so the robot stays framed.
        """
        spec = habitat_sim.CameraSensorSpec()
        spec.uuid = str(chase.get("uuid") or CHASE_UUID)
        spec.sensor_type = habitat_sim.SensorType.COLOR
        spec.resolution = [int(chase["height"]), int(chase["width"])]
        spec.hfov = float(chase["hfov_deg"])
        if hasattr(habitat_sim, "SensorSubType"):
            spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
        spec.position = [
            0.0,
            float(chase["height_m"]),
            float(chase["distance_m"]),
        ]
        pitch = math.radians(float(chase.get("pitch_deg", -25.0)))
        spec.orientation = [pitch, 0.0, 0.0]
        if hasattr(spec, "near"):
            spec.near = 0.05
        if hasattr(spec, "far"):
            spec.far = 100.0
        return spec

    def topdown_scene_bounds(self) -> tuple[float, float, float, float, float] | None:
        """Return ``(cx, cz, half_extent, y_lo, y_hi)`` for the overhead camera.

        Prefers the **navigable-cell** AABB from ``get_topdown_view`` (tight floor
        plan). Full ``pathfinder.get_bounds()`` often spans empty mesh padding and
        leaves the apartment as a small island in the frame.
        """
        if self.session is None:
            return None
        pathfinder = getattr(self.session, "pathfinder", None)
        if pathfinder is None or not bool(getattr(pathfinder, "is_loaded", False)):
            return None
        get_bounds = getattr(pathfinder, "get_bounds", None)
        if not callable(get_bounds):
            return None
        try:
            lo, hi = get_bounds()
        except Exception:
            return None
        y0, y1 = float(lo[1]), float(hi[1])
        tight = self.topdown_navigable_bounds(pathfinder, lo, hi)
        if tight is not None:
            cx, cz, half = tight
            return (cx, cz, half, y0, y1)
        x0, x1 = float(lo[0]), float(hi[0])
        z0, z1 = float(lo[2]), float(hi[2])
        half = 0.5 * max(abs(x1 - x0), abs(z1 - z0), 1.0)
        return (0.5 * (x0 + x1), 0.5 * (z0 + z1), half, y0, y1)

    def topdown_navigable_points(
        self, pathfinder: Any, lo: Any, hi: Any
    ) -> np.ndarray | None:
        """Navigable cell centers as ``(N, 2)`` Habitat ``(X, Z)``, or None."""
        get_topdown_view = getattr(pathfinder, "get_topdown_view", None)
        if not callable(get_topdown_view):
            return None
        resolution = 0.1
        island_radius = 0.5
        floor_height = float(self.floor_y or 0.0)
        if abs(floor_height) < 1e-6:
            random_nav = getattr(pathfinder, "get_random_navigable_point", None)
            if callable(random_nav):
                try:
                    point = random_nav()
                    if point is not None and len(point) >= 2:
                        floor_height = float(point[1])
                except Exception:
                    pass
        try:
            navigable = np.asarray(
                get_topdown_view(resolution, floor_height, island_radius),
                dtype=bool,
            )
        except Exception:
            return None
        if navigable.size == 0 or not np.any(navigable):
            return None
        rows, cols = np.where(navigable)
        origin_x = float(min(float(lo[0]), float(hi[0])))
        origin_z = float(min(float(lo[2]), float(hi[2])))
        # Cell centers: row↑ → +Habitat Z, col↑ → +Habitat X.
        xs = origin_x + (cols.astype(np.float64) + 0.5) * resolution
        zs = origin_z + (rows.astype(np.float64) + 0.5) * resolution
        return np.column_stack([xs, zs])

    def topdown_navigable_bounds(
        self, pathfinder: Any, lo: Any, hi: Any
    ) -> tuple[float, float, float] | None:
        """``(cx, cz, half_extent)`` from navigable cells, or None if empty."""
        points = self.topdown_navigable_points(pathfinder, lo, hi)
        if points is None or points.size == 0:
            return None
        x0, z0 = float(points[:, 0].min()), float(points[:, 1].min())
        x1, z1 = float(points[:, 0].max()), float(points[:, 1].max())
        half = 0.5 * max(x1 - x0, z1 - z0, 1.0)
        return (0.5 * (x0 + x1), 0.5 * (z0 + z1), half)

    def topdown_align_yaw(self, pathfinder: Any, lo: Any, hi: Any) -> float:
        """Yaw about Habitat +Y that makes the navmesh long axis image-horizontal.

        Cached on first call so the overhead view does not spin as the agent moves.
        """
        cached = getattr(self, "_topdown_align_yaw", None)
        if cached is not None:
            return float(cached)
        yaw = 0.0
        points = self.topdown_navigable_points(pathfinder, lo, hi)
        if points is not None and points.shape[0] >= 8:
            centered = points - points.mean(axis=0, keepdims=True)
            cov = centered.T @ centered / float(points.shape[0])
            try:
                evals, evecs = np.linalg.eigh(cov)
                axis = evecs[:, int(np.argmax(evals))]
                # atan2(Z, X): rotate so principal axis → Habitat +X (image right).
                yaw = -float(math.atan2(float(axis[1]), float(axis[0])))
            except Exception:
                yaw = 0.0
        self._topdown_align_yaw = float(yaw)
        return float(yaw)

    def topdown_oriented_bounds(
        self, pathfinder: Any, lo: Any, hi: Any, yaw: float
    ) -> tuple[float, float, float] | None:
        """Center + half-extent in the yaw-aligned frame (tighter when tilted)."""
        points = self.topdown_navigable_points(pathfinder, lo, hi)
        if points is None or points.size == 0:
            return None
        c = math.cos(float(yaw))
        s = math.sin(float(yaw))
        # Rotate Habitat XZ by +yaw into camera-aligned axes.
        xr = c * points[:, 0] - s * points[:, 1]
        zr = s * points[:, 0] + c * points[:, 1]
        half = 0.5 * max(float(xr.max() - xr.min()), float(zr.max() - zr.min()), 1.0)
        # Center back in world XZ.
        cx_r = 0.5 * (float(xr.min()) + float(xr.max()))
        cz_r = 0.5 * (float(zr.min()) + float(zr.max()))
        cx = c * cx_r + s * cz_r
        cz = -s * cx_r + c * cz_r
        return (cx, cz, half)

    def align_topdown_sensor(self, cfg: Mapping[str, Any]) -> None:
        """Place topdown camera above the scene (or agent) before rendering.

        When ``fit_scene`` is true, centers on the navmesh AABB and sizes
        ``ortho_scale`` so the whole floor plan fits (with ``ortho_margin``).

        Habitat orthographic math (CameraSensor.cpp)::

            orthographicProjection(nearPlaneSize / ortho_scale, ...)

        so **visible meters ≈ 1/ortho_scale** (default ``0.1`` ≈ 10 m). Larger
        scale zooms in. ``set_projection_params`` does **not** copy
        ``ortho_scale``; mutate ``specification()`` then nudge ``set_near`` to
        recompute the base projection matrix.
        """
        if self.session is None:
            return
        uuid = str(cfg.get("uuid") or TOPDOWN_UUID)
        getter = getattr(self.session, "get_sensor", None)
        if not callable(getter):
            return
        try:
            sensor = getter(uuid)
        except Exception:
            return
        if sensor is None:
            return
        node = getattr(sensor, "node", None)
        if node is None:
            return
        import magnum as mn

        height_m = float(cfg.get("height_m", 8.0))
        fit = bool(cfg.get("fit_scene", True))
        align = bool(cfg.get("align_navmesh", True))
        align_yaw = 0.0
        pathfinder = getattr(self.session, "pathfinder", None)
        get_bounds = getattr(pathfinder, "get_bounds", None) if pathfinder else None
        if (
            align
            and pathfinder is not None
            and callable(get_bounds)
            and bool(getattr(pathfinder, "is_loaded", False))
        ):
            try:
                lo, hi = get_bounds()
                align_yaw = self.topdown_align_yaw(pathfinder, lo, hi)
            except Exception:
                align_yaw = 0.0

        bounds = None
        if fit and pathfinder is not None and callable(get_bounds):
            try:
                lo, hi = get_bounds()
                if align:
                    oriented = self.topdown_oriented_bounds(
                        pathfinder, lo, hi, align_yaw
                    )
                    if oriented is not None:
                        cx, cz, half = oriented
                        y0, y1 = float(lo[1]), float(hi[1])
                        bounds = (cx, cz, half, y0, y1)
                if bounds is None:
                    bounds = self.topdown_scene_bounds()
            except Exception:
                bounds = self.topdown_scene_bounds()
        elif fit:
            bounds = self.topdown_scene_bounds()

        if bounds is not None:
            cx, cz, half, _y0, _y1 = bounds
            margin = max(1.0, float(cfg.get("ortho_margin", 1.05)))
            fill = float(cfg.get("ortho_fill", 0.92))
            fill = min(max(fill, 0.4), 0.99)
            cam_y = float(self.floor_y) + height_m
            extent = max(2.0 * float(half) * margin, 1.0)
            scale = cfg.get("ortho_scale")
            ortho = float(scale) if scale is not None else (fill / extent)
            world = mn.Vector3(cx, cam_y, cz)
        else:
            extent = max(2.0 * height_m, 10.0)
            fill = min(max(float(cfg.get("ortho_fill", 0.92)), 0.4), 0.99)
            scale = cfg.get("ortho_scale")
            ortho = float(scale) if scale is not None else (fill / extent)
            world = mn.Vector3(self.x, float(self.floor_y) + height_m, -self.y)
        ortho = float(min(max(ortho, 0.005), 2.0))

        # Sensor is parented under the agent; convert world pose → agent-local
        # so the view stays world-fixed (not spinning with robot yaw).
        agent = self.session.get_agent(0)
        agent_node = getattr(agent, "scene_node", None)
        if agent_node is None:
            return
        parent_T = agent_node.absolute_transformation()
        local = parent_T.inverted().transform_point(world)
        node.translation = local
        # World look-down: Ry(align) * Rx(−π/2). Cancel parent rotation so
        # image axes stay map/navmesh-aligned instead of robot-tilted.
        q_pitch = mn.Quaternion.rotation(
            mn.Rad(-0.5 * math.pi), mn.Vector3(1.0, 0.0, 0.0)
        )
        q_yaw = mn.Quaternion.rotation(
            mn.Rad(float(align_yaw)), mn.Vector3(0.0, 1.0, 0.0)
        )
        world_rot = q_yaw * q_pitch
        try:
            parent_rot = mn.Quaternion.from_matrix(parent_T.rotation())
        except Exception:
            parent_rot = mn.Quaternion()
        node.rotation = parent_rot.conjugated() * world_rot

        if str(cfg.get("mode") or "orthographic").lower() != "orthographic":
            return
        obj = getattr(sensor, "sensor_object", None)
        if obj is None or not hasattr(obj, "specification"):
            warnings.warn(
                "Habitat topdown: CameraSensor missing specification()",
                stacklevel=2,
            )
            return
        try:
            live = obj.specification()
        except Exception as exc:
            warnings.warn(
                f"Habitat topdown: specification() failed: {exc!r}",
                stacklevel=2,
            )
            return
        if live is None or not hasattr(live, "ortho_scale"):
            return
        live.ortho_scale = ortho
        # set_projection_params ignores ortho_scale; set_near recomputes
        # baseProjMatrix_ from the live CameraSensorSpec.
        near = float(getattr(live, "near", 0.05) or 0.05)
        try:
            if hasattr(obj, "set_near"):
                obj.set_near(near)
            elif hasattr(obj, "reset_zoom"):
                obj.reset_zoom()
        except Exception as exc:
            warnings.warn(
                f"Habitat topdown: failed to recompute projection: {exc!r}",
                stacklevel=2,
            )
        if hasattr(obj, "reset_zoom"):
            try:
                obj.reset_zoom()
            except Exception:
                pass

    def render_topdown(self) -> np.ndarray | None:
        """Return overhead RGB ``HxWx3 uint8``, or None if disabled / missing."""
        sensors = self.settings.get("habitat", {}).get("sensors")
        cfg = topdown_spec(sensors if isinstance(sensors, Mapping) else None)
        if cfg is None or self.session is None:
            return None
        self.align_topdown_sensor(cfg)
        observations = self.session.get_sensor_observations()
        uuid = str(cfg["uuid"])
        if uuid not in observations:
            warnings.warn(
                f"Habitat topdown observation missing {uuid!r}; "
                f"got {list(observations)!r}",
                stacklevel=2,
            )
            return None
        image = np.asarray(observations[uuid])
        if image.ndim == 3 and image.shape[2] >= 3:
            image = image[..., :3]
        # Prefer native sensor resolution — nearest-neighbor scale_image softens HD.
        color = self.rgb_to_uint8(image)
        dst_h, dst_w = int(cfg["height"]), int(cfg["width"])
        if color.shape[0] == dst_h and color.shape[1] == dst_w:
            return np.ascontiguousarray(color)
        return self.scale_image(color, dst_h, dst_w)

    def align_chase_sensor(self, cfg: Mapping[str, Any]) -> None:
        """Keep the chase camera behind the agent in Habitat local space.

        ``sensor.node`` is sometimes the scene-root camera, not an agent child.
        Writing ``(0, height, distance)`` in that case pins the view in the
        world and the Autoviz ``chase_link`` axes fly through the map.
        """
        if self.session is None:
            return
        uuid = str(cfg.get("uuid") or CHASE_UUID)
        getter = getattr(self.session, "get_sensor", None)
        if not callable(getter):
            return
        try:
            sensor = getter(uuid)
        except Exception:
            return
        if sensor is None:
            return
        node = getattr(sensor, "node", None)
        if node is None:
            return
        import magnum as mn

        local = mn.Vector3(
            0.0, float(cfg["height_m"]), float(cfg["distance_m"])
        )
        pitch = math.radians(float(cfg.get("pitch_deg", -25.0)))
        rotation = mn.Quaternion.rotation(
            mn.Rad(pitch), mn.Vector3(1.0, 0.0, 0.0)
        )
        agent = self.session.get_agent(0)
        agent_node = getattr(agent, "scene_node", None)
        parent = getattr(node, "parent", None)
        if agent_node is None or parent is agent_node:
            node.translation = local
            node.rotation = rotation
            return
        parent_T = agent_node.absolute_transformation()
        world_p = parent_T.transform_point(local)
        try:
            world_r = mn.Quaternion.from_matrix(parent_T.rotation()) * rotation
        except Exception:
            world_r = rotation
        if parent is None:
            node.translation = world_p
            node.rotation = world_r
            return
        pT = parent.absolute_transformation()
        node.translation = pT.inverted().transform_point(world_p)
        try:
            node.rotation = (
                mn.Quaternion.from_matrix(pT.rotation()).inverted() * world_r
            )
        except Exception:
            node.rotation = world_r

    def render_chase(self) -> np.ndarray | None:
        """Return third-person RGB ``HxWx3 uint8``, or None if disabled / missing.

        Briefly shows the instanced URDF so the chassis is visible, then hides it
        again so the next ego RGB-D capture stays unobstructed.
        """
        sensors = self.settings.get("habitat", {}).get("sensors")
        cfg = chase_spec(sensors if isinstance(sensors, Mapping) else None)
        if cfg is None or self.session is None:
            return None
        self.align_chase_sensor(cfg)
        self.set_articulated_visible(visible=True)
        try:
            observations = self.session.get_sensor_observations()
        finally:
            self.set_articulated_visible(visible=False)
        uuid = str(cfg["uuid"])
        if uuid not in observations:
            warnings.warn(
                f"Habitat chase observation missing {uuid!r}; "
                f"got {list(observations)!r}",
                stacklevel=2,
            )
            return None
        image = np.asarray(observations[uuid])
        if image.ndim == 3 and image.shape[2] >= 3:
            image = image[..., :3]
        color = self.rgb_to_uint8(image)
        dst_h, dst_w = int(cfg["height"]), int(cfg["width"])
        if color.shape[0] == dst_h and color.shape[1] == dst_w:
            return np.ascontiguousarray(color)
        return self.scale_image(color, dst_h, dst_w)

    @staticmethod
    def scale_image(image: np.ndarray, height: int, width: int) -> np.ndarray:
        """Scale ``image`` to ``height``×``width``. Same shape is returned as-is.

        Uses bilinear weights when upsizing so HD topdown stays sharper than
        nearest-neighbor indexing.
        """
        array = np.asarray(image)
        src_h, src_w = array.shape[:2]
        if src_h == height and src_w == width:
            return array
        if src_h <= 0 or src_w <= 0:
            return array
        # Bilinear via map_coordinates when available; else fallback NN.
        try:
            from scipy.ndimage import map_coordinates

            ys = np.linspace(0, src_h - 1, int(height), dtype=np.float64)
            xs = np.linspace(0, src_w - 1, int(width), dtype=np.float64)
            grid_y, grid_x = np.meshgrid(ys, xs, indexing="ij")
            if array.ndim == 2:
                return map_coordinates(
                    array.astype(np.float64, copy=False),
                    [grid_y, grid_x],
                    order=1,
                    mode="nearest",
                ).astype(array.dtype, copy=False)
            channels = [
                map_coordinates(
                    array[..., c].astype(np.float64, copy=False),
                    [grid_y, grid_x],
                    order=1,
                    mode="nearest",
                )
                for c in range(array.shape[2])
            ]
            return np.stack(channels, axis=-1).astype(array.dtype, copy=False)
        except Exception:
            rows = np.linspace(0, src_h - 1, int(height)).round().astype(np.int64)
            cols = np.linspace(0, src_w - 1, int(width)).round().astype(np.int64)
            return array[rows][:, cols]

    def render_surround(self) -> dict[str, np.ndarray]:
        """Render every surround color camera in one Habitat observation.

        Returns:
            ``uuid → HxWx3 uint8`` RGB.

        Raises:
            RuntimeError: Session closed or a configured uuid is missing.
        """
        sensors = self.settings.get("habitat", {}).get("sensors")
        cameras = resolve_surround(camera_rig(sensors), urdf=self.urdf)
        pano = panorama_spec(sensors)
        if not cameras and pano is None:
            return {}
        if self.session is None:
            raise RuntimeError("Habitat session is not open")
        observations = self.session.get_sensor_observations()
        images: dict[str, np.ndarray] = {}
        for camera in cameras:
            if camera.uuid not in observations:
                raise RuntimeError(
                    f"Habitat observations missing {camera.uuid!r}; "
                    f"got {list(observations)!r}"
                )
            image = self.align_camera_image(
                self.rgb_to_uint8(observations[camera.uuid])
            )
            images[camera.uuid] = self.scale_image(image, camera.height, camera.width)
        if pano is not None:
            if PANORAMA_UUID not in observations:
                raise RuntimeError(
                    f"Habitat observations missing {PANORAMA_UUID!r}; "
                    f"got {list(observations)!r}"
                )
            image = self.align_camera_image(
                self.rgb_to_uint8(observations[PANORAMA_UUID])
            )
            images[PANORAMA_UUID] = self.scale_image(
                image, int(pano["height"]), int(pano["width"])
            )
        return images

    def camera_local_offset(self) -> Tuple[float, float, float]:
        """Return the RGB-D mount in Habitat agent coordinates.

        URDF mounts use REP-103 ``(x forward, y left, z up)`` while Habitat's
        agent-local axes are ``(x right, y up, -z forward)``.  Applying the
        full mount here keeps rendered RGB-D rays at the same physical origin
        advertised by TF.  An explicit ``sensor_height`` overrides only the
        URDF z coordinate, preserving its calibrated planar offset.
        """
        camera_cfg = self.settings.get("habitat", {}).get("sensors", {}).get("camera", {})
        ros_x = 0.0
        ros_y = 0.0
        ros_z = 0.5
        if self.urdf is not None and "camera_link" in self.urdf.mounts:
            ros_x, ros_y, ros_z = self.urdf.camera_xyz()
        elif self.urdf is not None:
            ros_x, ros_y, ros_z = self.urdf.laser_xyz()
        # ``height`` is image pixels; never use it as a physical z offset.
        if camera_cfg.get("sensor_height") is not None:
            ros_z = float(camera_cfg["sensor_height"])
        return (-float(ros_y), float(ros_z), -float(ros_x))

    @staticmethod
    def habitat_look_direction(yaw: float) -> Tuple[float, float, float]:
        """Agent-local Habitat look vector after a camera yaw about ``+Y``.

        Default pinhole look is ``(0, 0, -1)``. ``Ry(yaw)`` yields
        ``(-sin(yaw), 0, -cos(yaw))``.
        """
        return (-math.sin(yaw), 0.0, -math.cos(yaw))

    @staticmethod
    def habitat_agent_quaternion(yaw: float) -> Tuple[float, float, float, float]:
        """Habitat y-up ``[x, y, z, w]`` for a ROS yaw about z-up (REP 103).

        Habitat agent local forward is ``-Z``. ROS ``base_link`` forward is ``+X``.
        With ``map_x = Habitat_X`` and ``map_y = -Habitat_Z``, ros yaw=0 should
        face ``+map_x = Habitat +X``.  A rotation ``Ry(θ)`` sends local ``-Z`` to
        world ``(-sin(θ), 0, -cos(θ))``. To face ``+X`` at yaw=0 we need
        ``-sin(θ) = 1 → θ = -π/2``. For general ros yaw: ``θ = yaw - π/2``.
        This matches ``autonomy_ros`` ``coords.habitat_yaw``.
        """
        half = 0.5 * (float(yaw) - math.pi / 2.0)
        return (0.0, math.sin(half), 0.0, math.cos(half))

    @staticmethod
    def habitat_agent_forward(yaw: float) -> Tuple[float, float, float]:
        """Habitat world direction of agent/ROS ``+X`` after planar yaw.

        map_x = Habitat_X, map_y = -Habitat_Z  →  Habitat_Z = -map_y.
        Forward in map frame: (cos_yaw, 0, sin_yaw) as (map_x, map_y) →
        Habitat: (cos_yaw, 0, -sin_yaw).
        """
        return (math.cos(float(yaw)), 0.0, -math.sin(float(yaw)))

    def chase_enabled(self) -> bool:
        """True when ``habitat.sensors.chase.enabled`` is set."""
        sensors = self.settings.get("habitat", {}).get("sensors")
        return chase_spec(sensors if isinstance(sensors, Mapping) else None) is not None

    def should_instance_urdf(self) -> bool:
        """Whether to add the URDF mesh into the Habitat scene.

        Default is off: ego RGB-D is attached to the agent, so a visible chassis
        or camera housing fills the frustum with solid colour blocks. Chase /
        third-person view auto-enables instancing so the robot is visible there;
        visuals stay hidden except during :meth:`render_chase`.
        """
        if self.urdf is None:
            return False
        robot = self.settings.get("habitat", {}).get("robot", {})
        if isinstance(robot, Mapping) and bool(robot.get("instance", False)):
            return True
        return self.chase_enabled()

    def camera_height(self) -> float | None:
        """Habitat Y height for cameras from URDF, else default eye height."""
        return float(self.camera_local_offset()[1])

    def attach_urdf(self, habitat_sim: Any) -> None:
        """Instance kinematic articulated object from configured URDF."""
        if self.session is None or not self.should_instance_urdf():
            return
        try:
            manager = self.session.get_articulated_object_manager()
            robot = manager.add_articulated_object_from_urdf(
                filepath=str(self.urdf.path),
                fixed_base=False,
            )
            robot.motion_type = habitat_sim.physics.MotionType.KINEMATIC
            try:
                robot.override_collidable_state(False)
            except Exception:
                try:
                    robot.collision_group = habitat_sim.physics.CollisionGroups.Noncollidable
                except Exception:
                    pass
            # Keep hidden by default so ego RGB-D stays clean; chase toggles.
            self.set_articulated_visible(robot, False)
            self.articulated = robot
            self.urdf_z_up = True
        except Exception as exc:
            warnings.warn(
                f"Habitat URDF load failed ({exc!r}); mounts still applied to rays",
                stacklevel=2,
            )
            self.articulated = None

    def set_articulated_visible(
        self, robot: Any | None = None, visible: bool = True
    ) -> None:
        """Show or hide the instanced URDF mesh (and link visual nodes)."""
        target = self.articulated if robot is None else robot
        Simulator.apply_articulated_visibility(target, visible)

    @staticmethod
    def hide_articulated(robot: Any) -> None:
        """Hide URDF visuals so first-person sensors see only the scene."""
        Simulator.apply_articulated_visibility(robot, False)

    @staticmethod
    def apply_articulated_visibility(robot: Any, visible: bool) -> None:
        """Toggle AO root + per-link visual nodes."""
        if robot is None:
            return
        for name in ("visible", "is_visible"):
            if hasattr(robot, name):
                try:
                    setattr(robot, name, bool(visible))
                except Exception:
                    pass
        n_links = int(getattr(robot, "num_links", 0) or 0)
        for link_id in range(-1, n_links):
            getter = getattr(robot, "get_link_visual_nodes", None)
            if callable(getter):
                try:
                    for node in getter(link_id) or []:
                        Simulator.set_scene_node_visible(node, visible)
                except Exception:
                    pass
            scene_getter = getattr(robot, "get_link_scene_node", None)
            if callable(scene_getter):
                try:
                    Simulator.set_scene_node_visible(scene_getter(link_id), visible)
                except Exception:
                    pass
        Simulator.set_scene_node_visible(
            getattr(robot, "root_scene_node", None), visible
        )

    @staticmethod
    def set_scene_node_visible(node: Any, visible: bool) -> None:
        """Show or hide a Magnum/Habitat scene node and its children."""
        if node is None:
            return
        for name in ("is_visible", "visible"):
            if hasattr(node, name):
                try:
                    setattr(node, name, bool(visible))
                except Exception:
                    pass
        setter = getattr(node, "set_visibility", None)
        if callable(setter):
            try:
                setter(bool(visible))
            except Exception:
                pass
        children = getattr(node, "children", None)
        if not children:
            return
        try:
            for child in list(children):
                Simulator.set_scene_node_visible(child, visible)
        except Exception:
            return

    @staticmethod
    def hide_scene_node(node: Any) -> None:
        """Hide a Magnum/Habitat scene node and its children."""
        Simulator.set_scene_node_visible(node, False)

    def sync_articulated(self, mn: Any) -> None:
        """Keep Habitat URDF root aligned; apply URDF z-up → Habitat y-up.

        Agent sensors use ``Ry(yaw − π/2)`` because Habitat local forward is
        ``−Z``. Instanced URDFs are different: after ``Rx(−π/2)``, URDF ``+X``
        already lies on Habitat ``+X``, so planar heading is plain ``Ry(yaw)``
        (same as humanoid AOs). The old ``yaw − π/2`` left the mesh 90° off.
        """
        if self.articulated is None:
            return
        yaw = mn.Quaternion.rotation(mn.Rad(self.yaw), mn.Vector3(0, 1, 0))
        # URDF uses z-up; Habitat is y-up → rotate −90° about X.
        z_up_to_y_up = mn.Quaternion.rotation(mn.Rad(-math.pi / 2.0), mn.Vector3(1, 0, 0))
        # map_y = -Habitat_Z  →  Habitat_Z = -map_y
        self.articulated.translation = mn.Vector3(self.x, self.floor_y, -self.y)
        self.articulated.rotation = yaw * z_up_to_y_up

    def reset(self, x: float, y: float, yaw: float) -> None:
        """Reset agent pose.

        Args:
            x: Planar x (m) → Habitat world x.
            y: Planar y (m) → Habitat world -z  (map_y = -Habitat_Z convention).
            yaw: Heading about vertical (rad).
        """
        self.set_pose(x, y, yaw)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        """Write planar pose and sync Habitat agent when live.

        Args:
            x: Planar x (m).
            y: Planar y (m).
            yaw: Heading (rad).
        """
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        if self.session is None:
            return
        import magnum as mn

        agent = self.session.get_agent(0)
        state = agent.get_state()
        # map_y = -Habitat_Z  →  Habitat_Z = -map_y
        state.position = mn.Vector3(self.x, self.floor_y, -self.y)
        qx, qy, qz, qw = self.habitat_agent_quaternion(self.yaw)
        state.rotation = [qx, qy, qz, qw]
        agent.set_state(state, infer_sensor_states=True)
        self.sync_articulated(mn)

    def constrain_motion(
        self, start_x: float, start_y: float, end_x: float, end_y: float
    ) -> Tuple[float, float]:
        """Keep a planar step on the Habitat navmesh and occupancy grid.

        Kinematic ``cmd_vel`` integration otherwise teleports through walls.
        Yaw is unchanged: the robot may still turn in place when translation
        is blocked.
        """
        if not self.block_motion:
            return float(end_x), float(end_y)
        sx, sy = float(start_x), float(start_y)
        placed_x, placed_y = float(end_x), float(end_y)
        pathfinder = None
        if self.session is not None:
            pathfinder = getattr(self.session, "pathfinder", None)
        if pathfinder is not None and bool(getattr(pathfinder, "is_loaded", False)):
            try_step = getattr(pathfinder, "try_step_no_sliding", None)
            if not callable(try_step):
                try_step = getattr(pathfinder, "try_step", None)
            if callable(try_step):
                start_h = (sx, float(self.floor_y), -sy)
                end_h = (placed_x, float(self.floor_y), -placed_y)
                hit = None
                try:
                    hit = try_step(start_h, end_h)
                except Exception:
                    try:
                        import magnum as mn

                        hit = try_step(
                            mn.Vector3(*start_h), mn.Vector3(*end_h)
                        )
                    except Exception:
                        hit = None
                xyz = self.finite_xyz(hit)
                if xyz is not None:
                    placed_x, placed_y = float(xyz[0]), -float(xyz[2])
                    if math.isfinite(float(xyz[1])):
                        self.floor_y = float(xyz[1])
                else:
                    placed_x, placed_y = sx, sy
        radius = 0.16
        if self.urdf is not None:
            poly = getattr(self.urdf, "footprint", ()) or ()
            if poly:
                radius = max(
                    radius,
                    max(math.hypot(float(px), float(py)) for px, py, *_ in poly),
                )
        mapper = self.map_builder
        recover = getattr(mapper, "recover_from_occupied", None)
        if callable(recover):
            sx, sy = recover(sx, sy, radius=radius)
        clip = getattr(mapper, "last_free_on_segment", None)
        if callable(clip):
            return clip(sx, sy, placed_x, placed_y, radius=radius)
        occupied = getattr(mapper, "disc_occupied", None)
        if callable(occupied) and occupied(placed_x, placed_y, radius=radius):
            return sx, sy
        return float(placed_x), float(placed_y)

    def laser_origin(self) -> Tuple[float, float, float]:
        """Habitat world origin for lidar rays (Y-up).

        Planar mount offsets are applied in the map/ROS yaw frame. When
        ``/scan`` is published in ``base_link`` (slam), rays still use the
        physical lidar height so thin walls are not missed.

        Returns:
            ``(x, y, z)`` with planar offsets rotated by yaw.
        """
        mx = my = mz = 0.0
        if self.urdf is not None:
            mx, my, mz = self.urdf.laser_xyz()
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        # map_y = -Habitat_Z  →  Habitat_Z = -map_y
        return (
            self.x + cos_y * mx - sin_y * my,
            self.floor_y + mz,
            -(self.y + sin_y * mx + cos_y * my),
        )

    def attach_humanoids(self) -> None:
        """Create TrackVLA-style SMPL-X humanoid sync from ``habitat.humanoids``."""
        if self.session is None:
            return
        from autosim.backends.habitat.humanoids.sync import (
            humanoid_config_from_habitat,
            maybe_create_humanoid_sync,
        )

        cfg = humanoid_config_from_habitat(self.settings.get("habitat"))
        pathfinder = None
        try:
            pathfinder = getattr(self.session, "pathfinder", None)
        except Exception:
            pathfinder = None
        self.humanoid_sync = maybe_create_humanoid_sync(
            self.session,
            config=cfg,
            floor_y=float(self.floor_y),
            pathfinder=pathfinder,
        )

    def bind_traffic_navmesh(self, traffic: Any = None) -> None:
        """Hard-constrain CPU pedestrians onto the Habitat navmesh.

        Call after Traffic spawn / map warmup so walkers cannot leave navigable
        free space (2D Field alone still allows void outside the crop).
        """
        host = traffic if traffic is not None else self.traffic
        if host is None or self.session is None:
            return
        ped = getattr(host, "pedestrian", None)
        binder = getattr(ped, "bind_pathfinder", None)
        if not callable(binder):
            return
        pathfinder = getattr(self.session, "pathfinder", None)
        if pathfinder is None or not bool(getattr(pathfinder, "is_loaded", False)):
            return
        binder(pathfinder, floor_y=float(self.floor_y))

    def step(self) -> None:
        """Advance one sim step; sync kinematic humanoids from Traffic."""
        sync = self.humanoid_sync
        if sync is not None and self.traffic is not None:
            try:
                # Keep feet height aligned with the robot floor sample.
                sync.floor_y = float(self.floor_y)
                sync.sync(self.traffic)
            except Exception:
                pass

    def pose(self) -> Tuple[float, float, float]:
        """Current planar pose.

        Returns:
            ``(x, y, yaw)``.
        """
        return self.x, self.y, self.yaw

    def linspace_angles(self, angle_min: float, angle_max: float, count: int) -> np.ndarray:
        """Evenly spaced angles over a FOV.

        Args:
            angle_min: Start angle (rad).
            angle_max: End angle (rad).
            count: Sample count (≥ 1).

        Returns:
            ``float64`` angle array.
        """
        if count <= 1:
            return np.array([float(angle_min)], dtype=np.float64)
        return np.linspace(float(angle_min), float(angle_max), num=int(count), dtype=np.float64)

    def eye_height(self) -> float:
        """Vertical Habitat Y for panoramic / lidar casts (meters)."""
        if self.urdf is not None:
            return self.floor_y + float(self.urdf.laser_xyz()[2])
        return self.floor_y + 0.5

    def refine_floor_y_from_mesh(
        self, *, probe_height: float = 2.0, range_max: float = 5.0
    ) -> float:
        """Align ``floor_y`` to the render-mesh floor under the agent.

        Navmesh Y is often a few centimetres above the GLB collision/visual
        floor (MP3D). Lidar rays then hit geometry below ``floor_y`` while the
        odometry root still assumes ground at z=0, so Autoviz shows the
        robot floating above the ground ring. A downward mesh cast fixes that.
        """
        if self.session is None:
            return float(self.floor_y)
        hit = self.cast_from_origin(
            float(self.x),
            float(self.floor_y) + float(probe_height),
            -float(self.y),
            float(self.yaw),
            -math.pi / 2.0,
            float(range_max),
        )
        if hit is None:
            return float(self.floor_y)
        # cast_from_origin returns map-frame (x, y, z_up); z_up is Habitat Y.
        mesh_floor = float(hit[2])
        if not math.isfinite(mesh_floor):
            return float(self.floor_y)
        delta = mesh_floor - float(self.floor_y)
        # Ignore wild hits (ceilings / voids); accept typical navmesh bias.
        if abs(delta) < 0.75:
            self.floor_y = mesh_floor
        return float(self.floor_y)

    def snap_to_navmesh(self) -> Tuple[float, float, float]:
        """Project the planar pose onto the loaded navmesh.

        Updates ``floor_y`` so RGB-D sits on the floor. Never jumps to a
        random island: that desyncs heading from the corridor the camera sees.
        """
        if self.session is None:
            return self.pose()
        pathfinder = getattr(self.session, "pathfinder", None)
        if pathfinder is None or not bool(getattr(pathfinder, "is_loaded", False)):
            self.refine_floor_y_from_mesh()
            self.set_pose(self.x, self.y, self.yaw)
            return self.pose()

        heights = []
        for height in (self.floor_y, 0.0, 0.3, 0.6, 1.0, 1.5):
            if height not in heights:
                heights.append(float(height))
        random_nav = getattr(pathfinder, "get_random_navigable_point", None)
        if callable(random_nav):
            try:
                sample = self.finite_xyz(random_nav())
                if sample is not None and sample[1] not in heights:
                    heights.append(sample[1])
            except Exception:
                pass

        best: Tuple[float, float, float] | None = None
        best_dist = float("inf")
        for height in heights:
            # Habitat_Z = -map_y under the map_y = -Habitat_Z convention.
            snapped = self.try_snap_point(pathfinder, self.x, height, -self.y)
            if snapped is None:
                continue
            # snapped = (Habitat_X, Habitat_Y, Habitat_Z); convert to map frame.
            dist = (snapped[0] - self.x) ** 2 + (-snapped[2] - self.y) ** 2
            if dist < best_dist:
                best = snapped
                best_dist = dist
        if best is None:
            # Spawn is off-navmesh (scene origins differ; e.g. MP3D vs skokloster).
            # Pick the navigable sample closest to the requested planar pose.
            best = self.nearest_navigable_sample(pathfinder, self.x, -self.y)
            if best is None:
                self.refine_floor_y_from_mesh()
                self.set_pose(self.x, self.y, self.yaw)
                return self.pose()
        # best = (Habitat_X, Habitat_Y, Habitat_Z); store as map coords.
        self.x = best[0]
        self.floor_y = best[1]
        self.y = -best[2]   # map_y = -Habitat_Z
        # Navmesh Y ≠ GLB floor; pull lidar/agent down onto the mesh.
        self.refine_floor_y_from_mesh()
        self.set_pose(self.x, self.y, self.yaw)
        return self.pose()

    def nearest_navigable_sample(
        self, pathfinder: Any, habitat_x: float, habitat_z: float, trials: int = 32
    ) -> Tuple[float, float, float] | None:
        """Return a navigable Habitat ``(x, y, z)`` closest to ``(habitat_x, *, habitat_z)``."""
        random_nav = getattr(pathfinder, "get_random_navigable_point", None)
        if not callable(random_nav):
            return None
        best: Tuple[float, float, float] | None = None
        best_dist = float("inf")
        for _ in range(max(1, int(trials))):
            try:
                sample = self.finite_xyz(random_nav())
            except Exception:
                continue
            if sample is None:
                continue
            dist = (sample[0] - habitat_x) ** 2 + (sample[2] - habitat_z) ** 2
            if dist < best_dist:
                best = sample
                best_dist = dist
        return best

    def try_snap_point(
        self, pathfinder: Any, x: float, height: float, z: float
    ) -> Tuple[float, float, float] | None:
        """Snap ``(x, height, z)`` with Habitat ``snap_point`` when available."""
        snap = getattr(pathfinder, "snap_point", None)
        if not callable(snap):
            return None
        try:
            import magnum as mn

            return self.finite_xyz(snap(mn.Vector3(float(x), float(height), float(z))))
        except Exception:
            try:
                return self.finite_xyz(snap([float(x), float(height), float(z)]))
            except Exception:
                return None

    @staticmethod
    def finite_xyz(point: Any) -> Tuple[float, float, float] | None:
        """Return ``(x, y, z)`` when all components are finite."""
        if point is None:
            return None
        try:
            values = (float(point[0]), float(point[1]), float(point[2]))
        except (TypeError, IndexError, ValueError):
            return None
        if not all(math.isfinite(value) for value in values):
            return None
        return values

    def cast_from_origin(
        self,
        origin_x: float,
        origin_y: float,
        origin_z: float,
        yaw: float,
        pitch: float,
        range_max: float,
    ) -> Tuple[float, float, float] | None:
        """Cast one ray; return map-frame hit ``(x, y, z)`` with z up.

        Args:
            origin_x: Habitat world x (= map x).
            origin_y: Habitat world y (up).
            origin_z: Habitat world z (= map y).
            yaw: Azimuth in the horizontal plane (rad).
            pitch: Elevation (rad).
            range_max: Maximum range (m).

        Returns:
            Map-frame point, or ``None`` on miss.
        """
        if self.session is None:
            raise RuntimeError("Habitat session is not open")
        import habitat_sim
        import magnum as mn

        origin = mn.Vector3(float(origin_x), float(origin_y), float(origin_z))
        # map_y = -Habitat_Z: yaw rotates +X toward +map_y = -Habitat_Z,
        # so Habitat direction_Z = -sin(yaw).
        dx = math.cos(yaw) * math.cos(pitch)
        dy = math.sin(pitch)
        dz = -math.sin(yaw) * math.cos(pitch)
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length < 1e-12:
            return None
        direction = mn.Vector3(dx / length, dy / length, dz / length)
        ray = habitat_sim.geo.Ray(origin, direction)
        try:
            results = self.session.cast_ray(ray, max_distance=float(range_max))
        except TypeError:
            results = self.session.cast_ray(ray)
        distance = self.raycast_hit_distance(results, origin)
        if distance is None or not math.isfinite(distance) or distance > float(range_max):
            return None
        if distance <= 1e-4:
            return None
        hx = float(origin_x) + distance * dx / length
        hy = float(origin_y) + distance * dy / length
        hz = float(origin_z) + distance * dz / length
        # map frame: map_x = hx, map_y = -hz, map_z = hy  (map_y = -Habitat_Z)
        return (hx, -hz, hy)

    @staticmethod
    def raycast_hit_distance(results: Any, origin: Any = None) -> float | None:
        """Closest hit range from Habitat ``RaycastResults``.

        Bullet may return multiple intersections; always take the nearest so
        rays stop at the first wall instead of a far surface behind it.
        """
        if results is None:
            return None
        checker = getattr(results, "has_hits", None)
        if checker is None:
            checker = getattr(results, "has_hit", None)
        if callable(checker):
            try:
                if not checker():
                    return None
            except Exception:
                return None
        elif checker is False:
            return None

        candidates: list[Any] = []
        hits = getattr(results, "hits", None)
        if hits:
            try:
                candidates.extend(list(hits))
            except TypeError:
                candidates.append(hits)
        if not candidates:
            candidates.append(results)

        best: float | None = None
        for target in candidates:
            distance = None
            if hasattr(target, "ray_distance"):
                try:
                    distance = float(target.ray_distance)
                except (TypeError, ValueError):
                    distance = None
            if distance is None or not math.isfinite(distance):
                point = getattr(target, "point", None)
                if point is None or origin is None:
                    continue
                try:
                    delta = point - origin
                    distance = float(math.sqrt(delta.dot(delta)))
                except Exception:
                    continue
            if not math.isfinite(distance) or distance <= 1e-4:
                continue
            if best is None or distance < best:
                best = distance
        return best

    def world_cloud(
        self,
        planar_x: float,
        planar_y: float,
        eye_y: float,
        h_angles: np.ndarray,
        v_angles: np.ndarray,
        range_max: float,
    ) -> np.ndarray:
        """Panoramic cloud in map frame (z-up) from a fixed planar origin.

        Args:
            planar_x: Map x (= Habitat x).
            planar_y: Map y (= -Habitat Z, map_y = -Habitat_Z convention).
            eye_y: Habitat up height for the cast origin.
            h_angles: Absolute yaw samples (rad).
            v_angles: Pitch samples (rad).
            range_max: Cast budget (m).

        Returns:
            ``Nx3 float32`` map-frame points.
        """
        points = []
        for pitch in v_angles:
            for yaw in h_angles:
                hit = self.cast_from_origin(
                    planar_x, eye_y, -planar_y, float(yaw), float(pitch), float(range_max)
                )
                if hit is None:
                    continue
                points.append(hit)
        if not points:
            return np.zeros((0, 3), dtype=np.float32)
        return np.asarray(points, dtype=np.float32)

    def cast_range(self, yaw_offset: float, pitch: float, range_max: float) -> float | None:
        """Cast one Habitat ray and return hit distance within ``range_max``.

        Args:
            yaw_offset: Azimuth relative to agent yaw (rad).
            pitch: Elevation (rad); 0 is horizontal.
            range_max: Maximum valid range (m); farther hits count as miss.

        Returns:
            Distance in meters, or ``None`` on miss / beyond ``range_max``.
        """
        ox, oy, oz = self.laser_origin()  # Habitat: (x, floor_y, -map_y)
        # Single-height ray for 2D SLAM: a vertical min-fan pulls ranges onto
        # furniture tops/edges and paints fuzzy / double walls in Cartographer.
        hit = self.cast_from_origin(ox, oy, oz, self.yaw + yaw_offset, pitch, range_max)
        # hit is map frame; laser_origin is Habitat → map: (ox, -oz, oy)
        return self.closer_actor(
            self.hit_distance(hit, ox, oy, oz),
            (ox, -oz, oy),
            self.yaw + yaw_offset,
            pitch,
            range_max,
        )

    @staticmethod
    def hit_distance(
        hit: Tuple[float, float, float] | None, ox: float, oy: float, oz: float
    ) -> float | None:
        """Map-frame distance from a Habitat origin ``(ox, oy, oz)`` to ``hit``."""
        if hit is None:
            return None
        dx = hit[0] - ox
        dy = hit[1] - (-oz)
        dz = hit[2] - oy
        return float(math.sqrt(dx * dx + dy * dy + dz * dz))

    @staticmethod
    def ray_direction(yaw: float, pitch: float) -> tuple[float, float, float]:
        """Unit map-frame direction for yaw (rad) and pitch (rad)."""
        horizontal = math.cos(float(pitch))
        return (
            math.cos(float(yaw)) * horizontal,
            math.sin(float(yaw)) * horizontal,
            math.sin(float(pitch)),
        )

    def actor_distance(
        self,
        origin: tuple[float, float, float],
        yaw: float,
        pitch: float,
        range_max: float,
    ) -> float | None:
        """Distance to the nearest dynamic body, or ``None`` when traffic is idle."""
        traffic = self.traffic
        if traffic is None:
            return None
        return traffic.nearest(origin, self.ray_direction(yaw, pitch), float(range_max))

    def closer_actor(
        self,
        distance: float | None,
        origin: tuple[float, float, float],
        yaw: float,
        pitch: float,
        range_max: float,
    ) -> float | None:
        """Keep the nearer of the static hit and a dynamic body."""
        actor = self.actor_distance(origin, yaw, pitch, range_max)
        if actor is None:
            return distance
        if distance is None or actor < distance:
            return actor
        return distance

    def laser_ranges(
        self,
        angle_min: float,
        angle_max: float,
        num_beams: int,
        range_max: float,
    ) -> np.ndarray:
        """Planar lidar ranges via ray casting.

        Args:
            angle_min: Start azimuth (rad).
            angle_max: End azimuth (rad).
            num_beams: Beam count.
            range_max: Miss fill value (m).

        Returns:
            ``float32`` ranges ``(num_beams,)``.
        """
        angles = self.linspace_angles(angle_min, angle_max, num_beams)
        ranges = np.empty((angles.shape[0],), dtype=np.float32)
        for index, yaw_offset in enumerate(angles):
            distance = self.cast_range(float(yaw_offset), 0.0, float(range_max))
            ranges[index] = (
                float(distance) if distance is not None else float("inf")
            )
        return ranges

    def clip_laser_to_occupancy(
        self, ranges: np.ndarray, angles: np.ndarray
    ) -> np.ndarray:
        """Stop beams at the first occupied cell of the published map grid."""
        mapper = getattr(self, "map_builder", None)
        if mapper is None:
            return ranges
        clip = getattr(mapper, "clip_laser_ranges", None)
        if not callable(clip):
            return ranges
        finite = np.asarray(ranges, dtype=np.float64)
        finite = finite[np.isfinite(finite)]
        range_max = float(np.max(finite)) if finite.size else 30.0
        if range_max <= 0.0:
            range_max = 30.0
        # Keep a little headroom above the farthest Habitat hit.
        range_max = max(range_max, 30.0)
        return clip(
            ranges,
            angles,
            float(self.x),
            float(self.y),
            float(self.yaw),
            float(range_max),
        )

    @staticmethod
    def spherical_point(
        radius: float, yaw_offset: float, pitch: float
    ) -> Tuple[float, float, float]:
        """Sensor-frame point from spherical coordinates.

        Args:
            radius: Range (m).
            yaw_offset: Azimuth in sensor frame (rad).
            pitch: Elevation (rad).

        Returns:
            ``(x, y, z)`` with x forward, y left, z up.
        """
        return (
            radius * math.cos(yaw_offset) * math.cos(pitch),
            radius * math.sin(yaw_offset) * math.cos(pitch),
            radius * math.sin(pitch),
        )

    def laser_origin_at(
        self, x: float, y: float, yaw: float
    ) -> Tuple[float, float, float]:
        """Habitat lidar origin for an arbitrary planar pose (Y-up)."""
        mx = my = mz = 0.0
        if self.urdf is not None:
            mx, my, mz = self.urdf.laser_xyz()
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        return (
            float(x) + cos_y * mx - sin_y * my,
            self.floor_y + mz,
            -(float(y) + sin_y * mx + cos_y * my),
        )

    def map_laser_origin_at(
        self, x: float, y: float, yaw: float
    ) -> Tuple[float, float, float]:
        """Map-frame lidar origin ``(map_x, map_y, map_z)`` for planar pose."""
        ox, oy, oz = self.laser_origin_at(x, y, yaw)
        return (ox, -oz, oy)

    def map_hit_to_sensor(
        self,
        hit: Tuple[float, float, float],
        x: float,
        y: float,
        yaw: float,
    ) -> Tuple[float, float, float]:
        """Express a map-frame hit in the lidar frame at ``(x, y, yaw)``."""
        ox, oy, oz = self.map_laser_origin_at(x, y, yaw)
        dx = float(hit[0]) - ox
        dy = float(hit[1]) - oy
        dz = float(hit[2]) - oz
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return (
            cosine * dx + sine * dy,
            -sine * dx + cosine * dy,
            dz,
        )

    @staticmethod
    def pose_at_scan_fraction(
        x_end: float,
        y_end: float,
        yaw_end: float,
        linear: float,
        angular: float,
        scan_period: float,
        fraction: float,
    ) -> Tuple[float, float, float]:
        """Planar pose at scan fraction ``∈[0,1]`` given end pose and twist.

        Habitat finishes the sweep at ``(x_end, y_end, yaw_end)``. Fraction 0 is
        scan start; 1 is scan end. Constant body twist is integrated backward.
        """
        from autosim.robot import Robot

        age = (1.0 - float(fraction)) * max(0.0, float(scan_period))
        if age <= 1e-12:
            return float(x_end), float(y_end), float(yaw_end)
        return Robot.integrate_unicycle(
            float(x_end),
            float(y_end),
            float(yaw_end),
            -float(linear),
            -float(angular),
            age,
        )

    def cast_range_at(
        self,
        x: float,
        y: float,
        yaw: float,
        yaw_offset: float,
        pitch: float,
        range_max: float,
    ) -> Tuple[float, float, float] | None:
        """Cast from an arbitrary pose; return map-frame hit or ``None``."""
        ox, oy, oz = self.laser_origin_at(x, y, yaw)
        heading = float(yaw) + float(yaw_offset)
        elevation = float(pitch)
        hit = self.cast_from_origin(ox, oy, oz, heading, elevation, float(range_max))
        origin = (ox, -oz, oy)
        habitat = self.hit_distance(hit, ox, oy, oz)
        actor = self.actor_distance(origin, heading, elevation, range_max)
        if actor is not None and (habitat is None or actor < habitat):
            direction = self.ray_direction(heading, elevation)
            return (
                origin[0] + actor * direction[0],
                origin[1] + actor * direction[1],
                origin[2] + actor * direction[2],
            )
        return hit

    def raycast_cloud(
        self,
        h_angles: np.ndarray,
        v_angles: np.ndarray,
        range_max: float,
        *,
        linear: float = 0.0,
        angular: float = 0.0,
        scan_period: float = 0.0,
        motion_compensate: bool = True,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Multi-ring cloud from Habitat ray casts; misses omitted.

        When ``motion_compensate`` and ``scan_period>0``, each azimuth column is
        cast at the pose during a spinning sweep and expressed in the **scan-end**
        lidar frame (deskew). That removes the classic motion-distortion smear
        without relying on downstream LIO undistort.

        Args:
            h_angles: Horizontal samples (rad), ordered start→end of spin.
            v_angles: Vertical samples (rad).
            range_max: Cast budget (m).
            linear, angular: Body twist during the scan (m/s, rad/s).
            scan_period: Full spin duration (s); ``0`` → instantaneous.
            motion_compensate: Deskew into end frame when motion is present.

        Returns:
            ``(Nx3 float32 points, N float32 ring ids)``; empty when no hits.
        """
        points = []
        rings = []
        x_e, y_e, yaw_e = float(self.x), float(self.y), float(self.yaw)
        n_h = int(len(h_angles))
        use_motion = (
            bool(motion_compensate)
            and float(scan_period) > 1e-6
            and n_h > 1
            and (abs(float(linear)) + abs(float(angular))) > 1e-9
        )
        for col, yaw_offset in enumerate(h_angles):
            if use_motion:
                fraction = float(col) / float(max(n_h - 1, 1))
                x_i, y_i, yaw_i = self.pose_at_scan_fraction(
                    x_e, y_e, yaw_e, linear, angular, scan_period, fraction
                )
            else:
                x_i, y_i, yaw_i = x_e, y_e, yaw_e
            for ring_id, pitch in enumerate(v_angles):
                if use_motion:
                    hit = self.cast_range_at(
                        x_i, y_i, yaw_i, float(yaw_offset), float(pitch), float(range_max)
                    )
                    if hit is None:
                        continue
                    # Deskew: express capture-time hit in the scan-end lidar frame.
                    point = self.map_hit_to_sensor(hit, x_e, y_e, yaw_e)
                else:
                    distance = self.cast_range(
                        float(yaw_offset), float(pitch), float(range_max)
                    )
                    if distance is None:
                        continue
                    point = self.spherical_point(
                        distance, float(yaw_offset), float(pitch)
                    )
                points.append(point)
                rings.append(float(ring_id))
        if not points:
            return (
                np.zeros((0, 3), dtype=np.float32),
                np.zeros((0,), dtype=np.float32),
            )
        return (
            np.asarray(points, dtype=np.float32),
            np.asarray(rings, dtype=np.float32),
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
        *,
        linear: float = 0.0,
        angular: float = 0.0,
        scan_period: float = 0.0,
        motion_compensate: bool = True,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Sample a multi-ring lidar cloud.

        Returns:
            ``(Nx3 float32 points, N float32 ring ids)`` in the sensor frame
            (scan-end frame when motion-compensated).
        """
        h_angles = self.linspace_angles(h_min, h_max, h_beams)
        v_angles = self.linspace_angles(v_min, v_max, v_rings)
        return self.raycast_cloud(
            h_angles,
            v_angles,
            range_max,
            linear=linear,
            angular=angular,
            scan_period=scan_period,
            motion_compensate=motion_compensate,
        )

    def camera_observations(
        self, *, with_semantic: bool = False
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray | None]:
        """One Habitat render for RGB, depth, and optional semantic.

        Args:
            with_semantic: When ``True``, decode the semantic sensor from the
                same ``get_sensor_observations()`` call as RGB-D.

        Returns:
            ``(color, depth, semantic_ids)`` where ``semantic_ids`` is ``HxW uint16``
            when ``with_semantic`` is false or the sensor is absent, ``None``.

        Raises:
            RuntimeError: Session not open or unexpected tensor shapes.
        """
        if self.session is None:
            raise RuntimeError("Habitat session is not open")
        observations = self.session.get_sensor_observations()
        rgb_key = self.observation_key(observations, ("rgb", "color_sensor", "color"))
        depth_key = self.observation_key(observations, ("depth", "depth_sensor"))
        color = self.align_camera_image(self.rgb_to_uint8(observations[rgb_key]))
        depth = np.squeeze(np.asarray(observations[depth_key], dtype=np.float32))
        depth = self.align_camera_image(depth)
        if depth.ndim != 2:
            raise RuntimeError(f"unexpected depth shape: {depth.shape!r}")
        np.nan_to_num(depth, copy=False, nan=0.0, posinf=0.0, neginf=0.0)

        semantic_ids: np.ndarray | None = None
        if with_semantic:
            semantic_key = None
            for name in ("semantic", "semantic_sensor"):
                if name in observations:
                    semantic_key = name
                    break
            if semantic_key is not None:
                semantic = self.align_camera_image(observations[semantic_key])
                semantic_ids = np.squeeze(np.asarray(semantic)).astype(np.uint16, copy=False)
                if semantic_ids.ndim != 2:
                    raise RuntimeError(f"unexpected semantic shape: {semantic_ids.shape!r}")
        return color, depth, semantic_ids

    def color_depth(self) -> Tuple[np.ndarray, np.ndarray]:
        """RGB and depth observations from Habitat.

        Returns:
            ``(color, depth)`` with depth squeezed to ``HxW float32``.

        Raises:
            RuntimeError: Session not open.
        """
        color, depth, _ = self.camera_observations(with_semantic=False)
        return color, depth

    def color_depth_semantic(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """RGB, depth, and semantic from a single Habitat observation.

        Returns:
            ``(color, depth, semantic_ids)`` with semantic as HxW uint16 instance ids.

        Raises:
            RuntimeError: Session not open or semantic sensor missing.
        """
        color, depth, semantic = self.camera_observations(with_semantic=True)
        if semantic is None:
            raise RuntimeError("Habitat semantic sensor is not configured")
        return color, depth, semantic

    @staticmethod
    def semantic_ids_to_bgr(semantic: np.ndarray) -> np.ndarray:
        """Encode Habitat semantic/instance ids as BGR8 for visualization."""
        ids = np.squeeze(np.asarray(semantic))
        if ids.ndim != 2:
            raise RuntimeError(f"unexpected semantic shape: {ids.shape!r}")
        ids = ids.astype(np.uint32, copy=False)
        b = (ids & 0xFF).astype(np.uint8)
        g = ((ids >> 8) & 0xFF).astype(np.uint8)
        r = ((ids >> 16) & 0xFF).astype(np.uint8)
        return np.stack([b, g, r], axis=-1)

    def semantic_bgr(self) -> np.ndarray:
        """Semantic segmentation as HxWx3 uint8 (BGR), aligned with RGB-D."""
        _, _, semantic = self.camera_observations(with_semantic=True)
        if semantic is None:
            raise RuntimeError("Habitat semantic sensor is not configured")
        return self.semantic_ids_to_bgr(semantic)

    @staticmethod
    def align_camera_image(image: np.ndarray) -> np.ndarray:
        """Return the pinhole image in OpenCV/REP-103 optical orientation.

        With agent ``Ry(yaw - π/2)`` and sensor orientation ``[0,0,0]``, Habitat
        pinhole pixels already use x-right, y-down, z-forward optical axes, so
        no image rotation or reflection is required.
        """
        return np.ascontiguousarray(np.asarray(image))

    @staticmethod
    def observation_key(observations: Mapping[str, Any], names: Tuple[str, ...]) -> str:
        """Pick the first Habitat observation key that exists."""
        for name in names:
            if name in observations:
                return name
        raise RuntimeError(
            f"Habitat observations missing {names!r}; got {list(observations)!r}"
        )

    @staticmethod
    def rgb_to_uint8(image: Any) -> np.ndarray:
        """Convert Habitat RGB(A) to contiguous ``HxWx3 uint8``."""
        color = np.asarray(image)
        if color.ndim == 3 and color.shape[-1] >= 3:
            color = color[..., :3]
        if np.issubdtype(color.dtype, np.floating):
            peak = float(np.nanmax(color)) if color.size else 1.0
            scale = 255.0 if peak <= 1.5 else 1.0
            color = np.clip(color * scale, 0.0, 255.0).astype(np.uint8)
        else:
            color = color.astype(np.uint8, copy=False)
        return np.ascontiguousarray(color)

    def close(self) -> None:
        """Release the Habitat session."""
        sync = getattr(self, "humanoid_sync", None)
        if sync is not None:
            try:
                sync.close()
            except Exception:
                pass
            self.humanoid_sync = None
        if self.session is not None:
            self.session.close()
            self.session = None
