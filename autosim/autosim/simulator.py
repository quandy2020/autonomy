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

from autosim.urdf import UrdfModel


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
        return Path(__file__).resolve().parent.parent / "config" / "mp3d.scene_dataset_config.json"

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
        # Match autonomy_ros sim._sensors(): orientation [0,0,0]. Agent quaternion
        # Ry(yaw - π/2) maps pinhole look (-Z) onto ROS/map +X forward.
        color_sensor.orientation = [0.0, 0.0, 0.0]
        depth_sensor.orientation = [0.0, 0.0, 0.0]
        for spec in (color_sensor, depth_sensor):
            if hasattr(spec, "near"):
                spec.near = 0.05
            if hasattr(spec, "far"):
                spec.far = 100.0
        sensor_specs = [color_sensor, depth_sensor]
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
        agent_configuration.sensor_specifications = sensor_specs
        return agent_configuration

    def camera_local_offset(self) -> Tuple[float, float, float]:
        """Habitat agent-local RGB-D mount (``autonomy_ros`` ``sim._sensors``).

        Pinhole rig sits at ``[0, height, 0]`` in agent frame (Y-up). Only
        vertical offset is used; planar offsets belong in URDF static TF, not
        the Habitat agent sensor (agent forward is local ``-Z``).
        """
        camera_cfg = self.settings.get("habitat", {}).get("sensors", {}).get("camera", {})
        # ``height`` is image pixels; never use it as Habitat Y.
        if camera_cfg.get("sensor_height") is not None:
            height = float(camera_cfg["sensor_height"])
        elif self.urdf is not None and "camera_link" in self.urdf.mounts:
            _, _, height = self.urdf.camera_xyz()
        elif self.urdf is not None:
            _, _, height = self.urdf.laser_xyz()
        else:
            height = 0.5
        return (0.0, float(height), 0.0)

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

    def should_instance_urdf(self) -> bool:
        """Whether to add the URDF mesh into the Habitat scene.

        Default is off: ego RGB-D is attached to the agent, so a visible chassis
        or camera housing fills the frustum with solid colour blocks.
        """
        if self.urdf is None:
            return False
        robot = self.settings.get("habitat", {}).get("robot", {})
        return bool(robot.get("instance", False)) if isinstance(robot, Mapping) else False

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
            # Ego RGB-D is attached to the agent. If the Husky mesh stays visible,
            # the chassis / lidar fill the frustum as solid colour blocks.
            self.hide_articulated(robot)
            self.articulated = robot
            self.urdf_z_up = True
        except Exception as exc:
            warnings.warn(
                f"Habitat URDF load failed ({exc!r}); mounts still applied to rays",
                stacklevel=2,
            )
            self.articulated = None

    @staticmethod
    def hide_articulated(robot: Any) -> None:
        """Hide URDF visuals so first-person sensors see only the scene."""
        for name in ("visible", "is_visible"):
            if hasattr(robot, name):
                try:
                    setattr(robot, name, False)
                except Exception:
                    pass
        n_links = int(getattr(robot, "num_links", 0) or 0)
        for link_id in range(-1, n_links):
            getter = getattr(robot, "get_link_visual_nodes", None)
            if callable(getter):
                try:
                    for node in getter(link_id) or []:
                        Simulator.hide_scene_node(node)
                except Exception:
                    pass
            scene_getter = getattr(robot, "get_link_scene_node", None)
            if callable(scene_getter):
                try:
                    Simulator.hide_scene_node(scene_getter(link_id))
                except Exception:
                    pass
        Simulator.hide_scene_node(getattr(robot, "root_scene_node", None))

    @staticmethod
    def hide_scene_node(node: Any) -> None:
        """Hide a Magnum/Habitat scene node and its children."""
        if node is None:
            return
        for name in ("is_visible", "visible"):
            if hasattr(node, name):
                try:
                    setattr(node, name, False)
                except Exception:
                    pass
        setter = getattr(node, "set_visibility", None)
        if callable(setter):
            try:
                setter(False)
            except Exception:
                pass
        children = getattr(node, "children", None)
        if not children:
            return
        try:
            for child in list(children):
                Simulator.hide_scene_node(child)
        except Exception:
            return

    def sync_articulated(self, mn: Any) -> None:
        """Keep Habitat URDF root aligned; apply URDF z-up → Habitat y-up."""
        if self.articulated is None:
            return
        yaw = mn.Quaternion.rotation(mn.Rad(self.yaw - math.pi / 2.0), mn.Vector3(0, 1, 0))
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

    def step(self) -> None:
        """Advance one sim step (pose is driven by :meth:`set_pose`)."""
        return None

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

    def snap_to_navmesh(self) -> Tuple[float, float, float]:
        """Project the planar pose onto the loaded navmesh.

        Updates ``floor_y`` so RGB-D sits on the floor. Never jumps to a
        random island: that desyncs heading from the corridor the camera sees.
        """
        if self.session is None:
            return self.pose()
        pathfinder = getattr(self.session, "pathfinder", None)
        if pathfinder is None or not bool(getattr(pathfinder, "is_loaded", False)):
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
            return self.pose()
        # best = (Habitat_X, Habitat_Y, Habitat_Z); store as map coords.
        self.x = best[0]
        self.floor_y = best[1]
        self.y = -best[2]   # map_y = -Habitat_Z
        self.set_pose(self.x, self.y, self.yaw)
        return self.pose()

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
        if hit is None:
            return None
        # hit is map frame; laser_origin is Habitat → map: (ox, -oz, oy)
        dx = hit[0] - ox
        dy = hit[1] - (-oz)
        dz = hit[2] - oy
        return float(math.sqrt(dx * dx + dy * dy + dz * dz))

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

    def raycast_cloud(
        self, h_angles: np.ndarray, v_angles: np.ndarray, range_max: float
    ) -> np.ndarray:
        """Multi-ring cloud from Habitat ray casts; misses omitted.

        Args:
            h_angles: Horizontal samples (rad).
            v_angles: Vertical samples (rad).
            range_max: Cast budget (m).

        Returns:
            ``Nx3 float32`` points (possibly empty).
        """
        points = []
        for pitch in v_angles:
            for yaw_offset in h_angles:
                distance = self.cast_range(float(yaw_offset), float(pitch), float(range_max))
                if distance is None:
                    continue
                points.append(self.spherical_point(distance, float(yaw_offset), float(pitch)))
        if not points:
            return np.zeros((0, 3), dtype=np.float32)
        return np.asarray(points, dtype=np.float32)

    def lidar_points(
        self,
        h_min: float,
        h_max: float,
        h_beams: int,
        v_min: float,
        v_max: float,
        v_rings: int,
        range_max: float,
    ) -> np.ndarray:
        """Sample a multi-ring lidar cloud.

        Returns:
            ``Nx3 float32`` in the sensor frame.
        """
        h_angles = self.linspace_angles(h_min, h_max, h_beams)
        v_angles = self.linspace_angles(v_min, v_max, v_rings)
        return self.raycast_cloud(h_angles, v_angles, range_max)

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
        """Return camera image for ROS ``camera_link`` (``autonomy_ros`` style).

        With agent ``Ry(yaw - π/2)`` and sensor orientation ``[0,0,0]``, Habitat
        pinhole output already matches REP-103 ``camera_link`` (+X forward).
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
        if self.session is not None:
            self.session.close()
            self.session = None
