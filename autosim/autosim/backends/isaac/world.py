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

import atexit
import logging
import math
from pathlib import Path
from typing import Any, Mapping, Tuple

import numpy as np

from autosim.backends.capabilities import ISAAC_CAPABILITIES
from autosim.backends.isaac.people import (
    people_sync_config_from_isaac,
    maybe_create_people_sync,
)
from autosim.backends.isaac.app import (
    close_app,
    ensure_simulation_app,
    pause_timeline,
    play_timeline,
    set_simulation_seed,
    start_viewport_recording,
    update_app,
)
from autosim.backends.isaac.randomization import (
    apply_randomization,
    randomization_config,
)
from autosim.backends.isaac.physics import (
    PHYSICS_DEFAULTS,
    PhysicsContext,
    physics_config_from_habitat,
)
from autosim.backends.isaac.sensors import SensorSuite
from autosim.backends.isaac.stage import SceneStage, require_isaac
from autosim.robot.profiles import apply_profile, resolve_usd_paths
from autosim.robot.urdf import UrdfModel
from autosim.scene.map import Map

atexit_registered = False
logger = logging.getLogger(__name__)


class IsaacWorld:
    """WorldBackend for Isaac Sim + USD (z-up map frame)."""

    capabilities = ISAAC_CAPABILITIES

    def __init__(
        self,
        *,
        settings: Mapping[str, Any],
        stage: SceneStage,
        sensors: SensorSuite,
        urdf: UrdfModel | None,
        width: int,
        height: int,
        eye_height: float,
    ) -> None:
        self.settings = settings
        self.stage = stage
        self.engine_sensors = sensors
        self.urdf = urdf
        self.width = int(width)
        self.height = int(height)
        self.sensor_height = float(eye_height)
        self.traffic: Any = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.map_builder: Any = None
        self.physics: PhysicsContext | None = None
        self.people: Any = None
        self.people_sync: Any = None
        self.hunav_manager: Any = None
        self.hunav_own_motion: bool = False
        self.scenario_grid: Any = None
        self.scenario_points: Any = None
        self.articulation: Any = None
        self.wheeled: Any = None  # WheeledDrive when mode=wheeled
        self.physics_mode: str = "wheeled"
        self.lidar_backend: str = "rtx"  # cpu | rtx | merge (Isaac default: rtx)
        self._paused: bool = False
        self._contact_hits: int = 0
        self._domain_random: dict[str, Any] = {}

    @property
    def articulation_driven(self) -> bool:
        """True when PhysX Articulation owns planar pose (balance / fall)."""
        return (
            self.physics_mode == "articulation"
            and self.articulation is not None
            and bool(getattr(self.articulation, "ready", False))
        )

    @classmethod
    def create(cls, settings: Mapping[str, Any]) -> "IsaacWorld":
        """Factory from validated config; requires Isaac Sim Python APIs."""
        habitat = settings["habitat"]
        camera = habitat["sensors"]["camera"]
        lidar_2d_cfg = (habitat.get("sensors") or {}).get("lidar_2d") or {}
        lidar_backend = str(lidar_2d_cfg.get("backend") or "rtx").strip().lower()
        want_camera = bool(camera.get("enabled", False))
        # RTX Lidar also needs the rendering / rtx_sensors experience.
        want_rtx = want_camera or (
            bool(lidar_2d_cfg.get("enabled", False))
            and lidar_backend in ("rtx", "merge")
        )
        isaac_block = habitat.get("isaac") if isinstance(habitat.get("isaac"), Mapping) else {}
        headless_cfg = None
        if isinstance(isaac_block, Mapping) and "headless" in isaac_block:
            headless_cfg = bool(isaac_block["headless"])
        ensure_simulation_app(
            headless=headless_cfg,
            enable_cameras=want_rtx,
        )
        # Optional episode seed (isaac.seed or domain_random.seed).
        seed_val = None
        if isinstance(isaac_block, Mapping):
            if isaac_block.get("seed") is not None:
                seed_val = int(isaac_block["seed"])
            else:
                dr = isaac_block.get("domain_random")
                if isinstance(dr, Mapping) and dr.get("seed") is not None:
                    seed_val = int(dr["seed"])
        if seed_val is not None:
            set_simulation_seed(seed_val)
        require_isaac()
        global atexit_registered
        if not atexit_registered:
            atexit.register(close_app)
            atexit_registered = True
        robot_cfg = dict(habitat.get("robot") or {})
        robot_cfg = apply_profile(robot_cfg)
        width = int(camera["width"])
        height = int(camera["height"])
        eye = float(camera.get("sensor_height", robot_cfg.get("base_link_height", 0.5)))
        eye_offset = (0.0, 0.0, eye)
        try:
            urdf = UrdfModel.load(str(robot_cfg.get("urdf") or ""))
        except (FileNotFoundError, ValueError):
            urdf = None
        # Prefer URDF camera_link XYZ over Z-only height.
        if urdf is not None:
            try:
                cx, cy, cz, _ = urdf.link_pose("camera_link")
                eye_offset = (float(cx), float(cy), float(cz))
                eye = float(cz)
            except ValueError:
                pass
        yaml_xyz = camera.get("sensor_xyz") or camera.get("xyz")
        if isinstance(yaml_xyz, (list, tuple)) and len(yaml_xyz) >= 3:
            eye_offset = (
                float(yaml_xyz[0]),
                float(yaml_xyz[1]),
                float(yaml_xyz[2]),
            )
            eye = float(eye_offset[2])
        world_usd, robot_usd = resolve_usd_paths(habitat, robot_cfg)
        base_z = float(robot_cfg.get("base_link_height", 0.0))
        prim = str(
            (habitat.get("usd") or {}).get("robot_prim")
            or robot_cfg.get("usd_prim")
            or "/World/Robot"
        )
        phys_cfg = physics_config_from_habitat(habitat)
        physics_mode = str(phys_cfg.get("mode") or "wheeled").strip().lower()
        if physics_mode != "wheeled":
            raise RuntimeError(
                f"habitat.isaac.physics.mode={physics_mode!r} is unsupported; "
                "use mode=wheeled (turtlebot3_burger | husky)"
            )
        stage = SceneStage(
            world_usd,
            robot_usd=robot_usd,
            robot_prim_path=prim,
            base_z=base_z,
        )
        sensors = SensorSuite(
            width=width, height=height, eye_height=eye, eye_offset=eye_offset
        )
        sim = cls(
            settings=settings,
            stage=stage,
            sensors=sensors,
            urdf=urdf,
            width=width,
            height=height,
            eye_height=eye,
        )
        sim.physics_mode = physics_mode
        sim.attach_floorplan_map(world_usd, habitat.get("map"))
        sim.seed_scenario_geometry(habitat)
        # World first (needed for Replicator RGB even when collision is off),
        # then camera under robot so the view tracks teleop like Habitat.
        # wheeled: robot USD collision + scene colliders; wheel joints later.
        sim.physics = PhysicsContext.attach(
            stage,
            robot_prim_path=prim,
            config=dict(phys_cfg),
            render=want_camera,
        )
        camera_parent = prim
        if want_camera:
            sensors.mount_robot_camera(
                camera_parent,
                hfov_deg=float(camera.get("hfov_deg", 90.0)),
                rate_hz=float(camera.get("rate_hz", 30.0)),
            )
            surround = camera.get("surround") or []
            if isinstance(surround, list) and surround:
                sensors.mount_surround_cameras(
                    camera_parent,
                    surround,
                    rate_hz=float(camera.get("surround_rate_hz", camera.get("rate_hz", 15.0))),
                )
        lidar_2d = (habitat.get("sensors") or {}).get("lidar_2d") or {}
        # Isaac world defaults to Kit RTX — cpu/merge are explicit debug overrides.
        sim.lidar_backend = str(lidar_2d.get("backend") or "rtx").strip().lower()
        want_rtx_lidar = sim.lidar_backend in ("rtx", "merge") and bool(
            lidar_2d.get("enabled", False)
        )
        if want_rtx_lidar:
            lx, ly, lz = 0.0, 0.0, float(lidar_2d.get("sensor_height", 0.2))
            if urdf is not None:
                for link in ("laser_link", "base_scan"):
                    try:
                        lx, ly, lz, _ = urdf.link_pose(link)
                        break
                    except ValueError:
                        continue
            rtx_name = str(lidar_2d.get("rtx_config") or "RPLIDAR_S2E")
            if not SensorSuite.is_2d_lidar_config(rtx_name):
                logger.warning(
                    "lidar_2d.rtx_config=%r is not a 2D FlatScan profile "
                    "(elevationDeg≠0); forcing RPLIDAR_S2E",
                    rtx_name,
                )
                rtx_name = "RPLIDAR_S2E"
            sensors.mount_robot_lidar(
                camera_parent,
                translation=(float(lx), float(ly), float(lz)),
                config_name=rtx_name,
                flat_scan=True,
            )
        imu_cfg = (habitat.get("sensors") or {}).get("imu") or {}
        if bool(imu_cfg.get("enabled", False)):
            ix = iy = iz = 0.0
            if urdf is not None:
                for link in ("imu_link", "imu", "base_link"):
                    try:
                        ix, iy, iz, _ = urdf.link_pose(link)
                        break
                    except ValueError:
                        continue
            sensors.mount_robot_imu(
                camera_parent,
                translation=(float(ix), float(iy), float(iz)),
                frequency=float(imu_cfg.get("rate_hz", 200.0)),
            )
        if physics_mode == "wheeled":
            from autosim.backends.isaac.robots.wheeled import WheeledDrive

            wheel_cfg = {}
            if isinstance(isaac_block, Mapping):
                raw_w = isaac_block.get("wheeled")
                if isinstance(raw_w, Mapping):
                    wheel_cfg = dict(raw_w)
            sim.wheeled = WheeledDrive.try_create(
                stage.stage,
                camera_parent,
                track_width=float(wheel_cfg.get("track_width", 0.16)),
                wheel_radius=float(wheel_cfg.get("wheel_radius", 0.033)),
                joint_names=wheel_cfg.get("joints"),
            )
            if sim.wheeled is None:
                raise RuntimeError(
                    "physics.mode=wheeled but no wheel joints found on "
                    f"{camera_parent!r}; check isaac.wheeled.joints / robot USD"
                )
        sim._domain_random = randomization_config(
            isaac_block if isinstance(isaac_block, Mapping) else None
        )
        record_path = ""
        if isinstance(isaac_block, Mapping):
            record_path = str(isaac_block.get("record_path") or "").strip()
        if record_path:
            start_viewport_recording(record_path)
        traffic_cfg = people_sync_config_from_isaac(
            isaac_block if isinstance(isaac_block, Mapping) else None
        )
        parent = prim.rsplit("/", 1)[0] if "/" in prim else "/Root"
        if not traffic_cfg.get("usd_root"):
            traffic_cfg["usd_root"] = f"{parent}/AutosimTraffic"
        sim.people_sync = maybe_create_people_sync(stage, config=traffic_cfg)
        from autosim.backends.isaac.articulation import kit_log

        kit_log(
            f"IsaacWorld ready: articulation_driven="
            f"{sim.articulation_driven} mode={sim.physics_mode} "
            f"block_motion={bool(getattr(sim.physics, 'block_motion', False))}"
        )
        return sim

    def attach_floorplan_map(
        self, world_usd: str, map_cfg: Mapping[str, Any] | None
    ) -> None:
        """Prefer InteriorAgent ``rooms.json`` so nav can publish GT ``/map``.

        Occupancy stays floorplan-based; ``/overall/map`` prefers USD mesh
        vertices so furniture and full walls are not missing.
        """
        if not isinstance(map_cfg, Mapping) or not map_cfg.get("enabled", False):
            return
        from autosim.scene.floorplan import FloorplanMap

        rooms_path = FloorplanMap.find_rooms_json(world_usd)
        if rooms_path is None:
            return
        rooms = FloorplanMap.load_rooms(rooms_path)
        if not rooms:
            return
        built = FloorplanMap(map_cfg).build_from_rooms(rooms)
        grid_cfg = map_cfg.get("grid") or {}
        ply_cfg = map_cfg.get("ply") or {}
        ply_channel = str(ply_cfg.get("channel") or "").strip()
        # Habitat publishes scene geometry on /overall/map; do the same from USD
        # mesh vertices so Autoviz shape matches the Isaac viewport.
        mesh_cloud = np.zeros((0, 3), dtype=np.float32)
        # Occupancy carve: USD vertices only (+ inflate). Viz cloud:
        # - surface_spacing_m > 0 → densified USD (slow; optional)
        # - else keep rooms.json dense_floorplan_cloud (solid walls, fast)
        surface_spacing = float(ply_cfg.get("surface_spacing_m", 0.0))
        mesh_stride = max(1, int(ply_cfg.get("mesh_stride", 1)))
        isaac = (self.settings.get("habitat") or {}).get("isaac") or {}
        phys = isaac.get("physics") if isinstance(isaac, Mapping) else {}
        if not isinstance(phys, Mapping):
            phys = {}
        z_lo, z_hi, inflate = FloorplanMap.robot_body_band(
            map_cfg,
            robot_height_m=float(phys.get("robot_height", 0.28)),
            robot_radius_m=float(phys.get("robot_radius", 0.12)),
        )
        furniture = np.zeros((0, 3), dtype=np.float32)
        try:
            furniture = self.stage.sample_mesh_cloud(
                z_min=z_lo,
                z_max=z_hi,
                stride=mesh_stride,
                max_points=int(ply_cfg.get("max_points", 400_000)),
                surface_spacing_m=0.0,
                skip_path_substrings=("door",),
            )
        except Exception as exc:
            logger.warning("USD furniture sample failed: %s", exc)
        if furniture.shape[0] > 0:
            changed = built.mark_elevated_obstacles(
                furniture,
                z_min=z_lo,
                z_max=z_hi,
                inflate_m=inflate,
            )
        else:
            changed = 0
        opened = 0
        if bool(map_cfg.get("free_doors", True)):
            try:
                door_stride = max(
                    1, min(4, int(ply_cfg.get("mesh_stride", 8)) // 2)
                )
                door_cloud = self.stage.sample_mesh_cloud(
                    z_min=0.0,
                    z_max=max(z_hi, 2.2),
                    stride=door_stride,
                    max_points=200_000,
                    surface_spacing_m=0.0,
                    only_path_substrings=("door",),
                )
                door_inflate = float(
                    map_cfg.get("door_opening_inflate_m", 0.15)
                )
                opened = built.free_openings(
                    door_cloud,
                    z_min=0.0,
                    z_max=max(z_hi, 2.2),
                    inflate_m=door_inflate,
                )
            except Exception as exc:
                logger.warning("door opening free failed: %s", exc)
        logger.info(
            "Isaac /map body-band z=[%.2f, %.2f] inflate=%.2f "
            "(%d furniture → %d occ, %d door cells freed)",
            z_lo,
            z_hi,
            inflate,
            int(furniture.shape[0]),
            int(changed),
            int(opened),
        )
        if ply_channel and surface_spacing > 0.0:
            try:
                mesh_cloud = self.stage.sample_mesh_cloud(
                    z_min=float(grid_cfg.get("z_min", 0.0)),
                    z_max=float(grid_cfg.get("z_max", 2.5)),
                    stride=mesh_stride,
                    max_points=int(ply_cfg.get("max_points", 400_000)),
                    surface_spacing_m=surface_spacing,
                )
                if mesh_cloud.shape[0] > 0:
                    built.set_cloud(mesh_cloud)
            except Exception as exc:
                logger.warning("USD mesh cloud densify failed: %s", exc)
        elif ply_channel and furniture.shape[0] > 0:
            # Append furniture verts onto solid floorplan walls.
            base = np.asarray(built.cached_cloud, dtype=np.float32).reshape(-1, 3)
            built.set_cloud(np.vstack([base, furniture]))
        self.map_builder = built
        self.scenario_grid = built.grid
        self.scenario_points = getattr(built, "points", None)

    def seed_scenario_geometry(self, habitat: Mapping[str, Any]) -> None:
        """Fill ``scenario.x_length`` / center / spawn from the floorplan grid."""
        packed = self.scenario_grid
        if packed is None:
            return
        cells, resolution, ox, oy, width, height = packed
        min_x = float(ox)
        min_y = float(oy)
        max_x = min_x + float(width) * float(resolution)
        max_y = min_y + float(height) * float(resolution)
        cx = 0.5 * (min_x + max_x)
        cy = 0.5 * (min_y + max_y)
        spawn = habitat.get("spawn") or [cx, cy, 0.0]
        scenario = habitat.setdefault("scenario", {})
        if not isinstance(scenario, dict):
            return
        scenario.setdefault("x_length", max(1.0, max_x - min_x))
        scenario.setdefault("y_length", max(1.0, max_y - min_y))
        scenario.setdefault("center_x", cx)
        scenario.setdefault("center_y", cy)
        scenario.setdefault("spawn_x", float(spawn[0]))
        scenario.setdefault("spawn_y", float(spawn[1]))
        scenario.setdefault("spawn_radius", 0.8)

    def reset(self, x: float, y: float, yaw: float) -> None:
        # Spawn must teleport even if the footprint overlaps furniture initially.
        self.set_pose(x, y, yaw, force=True)
        if self._domain_random.get("enabled"):
            apply_randomization(
                self.stage,
                people_sync=self.people_sync,
                config=self._domain_random,
            )
        self._contact_hits = 0

    def set_pose(
        self, x: float, y: float, yaw: float, *, force: bool = False
    ) -> None:
        """Write planar pose to the robot Xform / Articulation root.

        When PhysX ``block_motion`` is on, overlapping queries keep the previous
        pose (footprint sphere vs scene colliders). Articulation mode ignores
        non-force teleports so dynamics own the base (balance / fall).
        """
        nx, ny, nyaw = float(x), float(y), float(yaw)
        if self.articulation_driven and not force:
            # Physics owns the base; still keep camera synced to measured pose.
            self._sync_camera_to_pose()
            return
        physics = self.physics
        if (
            not force
            and physics is not None
            and physics.block_motion
            and physics.overlaps(nx, ny)
        ):
            return
        self.x = nx
        self.y = ny
        self.yaw = nyaw
        if self.articulation_driven and force:
            base_z = float(getattr(self.stage, "base_z", 0.14) or 0.14)
            if base_z < 0.12:
                base_z = 0.14
            self.articulation.teleport(nx, ny, nyaw, z=base_z, joint_defaults=True)
        else:
            try:
                self.stage.set_pose(self.x, self.y, self.yaw)
            except RuntimeError as exc:
                # Invalid robot_xform after a failed articulation fallback.
                logger.warning("stage.set_pose skipped (%s); refreshing xform", exc)
                try:
                    from pxr import UsdGeom

                    path = str(getattr(self.stage, "robot_prim_path", "") or "")
                    prim_obj = self.stage.stage.GetPrimAtPath(path) if path else None
                    if prim_obj and prim_obj.IsValid():
                        self.stage.robot_xform = UsdGeom.Xformable(prim_obj)
                        self.stage.set_pose(self.x, self.y, self.yaw)
                except Exception as exc2:  # noqa: BLE001
                    logger.error("stage.set_pose recovery failed: %s", exc2)
        self._sync_camera_to_pose()

    def _sync_camera_to_pose(self) -> None:
        sync = getattr(self.engine_sensors, "sync_world_pose", None)
        if not callable(sync):
            return
        base_z = float(getattr(self.stage, "base_z", 0.0) or 0.0)
        if self.articulation_driven:
            _x, _y, _yaw, z = self.articulation.read_pose()
            # Eye offset already relative to base_link; pass measured root z.
            sync(self.x, self.y, self.yaw, base_z=float(z))
            return
        sync(self.x, self.y, self.yaw, base_z=base_z)

    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw

    def pull_articulation_pose(self) -> tuple[float, float, float]:
        """Read floating-base pose from PhysX into ``self.x/y/yaw``."""
        if not self.articulation_driven:
            return self.pose()
        x, y, yaw, _z = self.articulation.read_pose()
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        return self.pose()

    def drive_wheels(self, vx: float, wz: float) -> bool:
        """Apply differential-drive twist when ``physics.mode=wheeled``."""
        wheeled = self.wheeled
        if wheeled is None:
            return False
        return bool(wheeled.apply(float(vx), float(wz)))

    def joint_states(self) -> dict[str, float]:
        """Joint name → position (articulation) for ``/joint_states``."""
        art = self.articulation
        if art is None or not getattr(art, "ready", False):
            return {}
        q: dict[str, float] = {}
        dq: dict[str, float] = {}
        try:
            art.sync_joint_state(q, dq)
        except Exception:
            return {}
        return q

    def joint_velocities(self) -> dict[str, float]:
        art = self.articulation
        if art is None or not getattr(art, "ready", False):
            return {}
        q: dict[str, float] = {}
        dq: dict[str, float] = {}
        try:
            art.sync_joint_state(q, dq)
        except Exception:
            return {}
        return dq

    def fallen(self) -> bool:
        """True when an articulation fall heuristic is available (unused)."""
        art = self.articulation
        if art is None or not hasattr(art, "fallen"):
            return False
        try:
            return bool(art.fallen())
        except Exception:
            return False

    def contact_count(self) -> int:
        """Rough contact counter (overlap probes while block_motion is on)."""
        physics = self.physics
        if physics is None or not getattr(physics, "block_motion", False):
            return int(self._contact_hits)
        try:
            if physics.overlaps(self.x, self.y):
                self._contact_hits += 1
        except Exception:
            pass
        return int(self._contact_hits)

    def pause(self) -> None:
        """Pause Kit timeline (PhysX / animation freeze)."""
        pause_timeline()
        self._paused = True

    def resume(self) -> None:
        """Resume Kit timeline."""
        play_timeline()
        self._paused = False

    def step_once(self) -> None:
        """Single control tick even when paused (debug)."""
        was = self._paused
        if was:
            play_timeline()
        try:
            self.step()
        finally:
            if was:
                pause_timeline()

    def surround_images(self) -> list[Any]:
        """RGB frames from surround cameras (empty list when none)."""
        read = getattr(self.engine_sensors, "read_surround_rgb", None)
        if not callable(read):
            return []
        return list(read())

    def attach_hunav(
        self,
        *,
        pedestrian: Any,
        config_file: str = "",
        own_motion: bool = True,
        compute_client: Any | None = None,
        engine: Any | None = None,
        robot_pose_fn: Any | None = None,
        dt: float = 0.02,
    ) -> Any:
        """Bind :class:`HuNavManager` (Hunav_isaac_wrapper port).

        Spawns SkelRoot characters + AnimationGraph under ``{World|Root}/Characters``.
        When ``own_motion`` is True, Runner should skip ``Traffic.pedestrian.step``
        and let ``step_hunav`` drive SFM via ``/compute_agents`` (in-process).
        """
        from autosim.backends.isaac.hunav import HuNavManager

        habitat = self.settings.get("habitat") or {}
        isaac_cfg = habitat.get("isaac") if isinstance(habitat, Mapping) else {}
        stage = getattr(self.stage, "stage", None)
        if stage is None:
            return None
        # Wrapper layout: /World/Characters (or /Root/Characters on InteriorAgent).
        root = "/World/Characters"
        for candidate in ("/World", "/Root"):
            prim = stage.GetPrimAtPath(candidate)
            if prim and prim.IsValid():
                root = f"{candidate}/Characters"
                break
        # Stop PeopleSync from mirroring Traffic (manager owns USD people).
        if self.people_sync is not None:
            try:
                self.people_sync.enabled = False
            except Exception:
                pass
        self.hunav_own_motion = bool(own_motion)
        self.hunav_dt = float(dt)
        pose_fn = robot_pose_fn or (
            lambda: (
                float(self.x),
                float(self.y),
                float(self.yaw),
                0.0,
                0.0,
                0.0,
            )
        )
        self.hunav_manager = HuNavManager(
            stage,
            config_file=config_file or None,
            robot_pose_fn=pose_fn,
            people_sync=self.people_sync,
            isaac_cfg=isaac_cfg if isinstance(isaac_cfg, Mapping) else {},
            engine=engine,
            compute_client=compute_client,
            pedestrian=pedestrian,
            own_motion=self.hunav_own_motion,
            root_path=root,
        )
        logger.info(
            "HuNavManager attached (own_motion=%s usd_agents=%s ped=%s)",
            self.hunav_own_motion,
            len(getattr(self.hunav_manager, "agents", []) or []),
            int(pedestrian.position.shape[0]) if pedestrian is not None else 0,
        )
        return self.hunav_manager

    def step_hunav(self, dt: float | None = None, stamp: tuple[int, int] = (0, 0)) -> Any:
        """Run one HuNavManager tick when attached; else ``None``."""
        manager = self.hunav_manager
        if manager is None:
            return None
        return manager.send_agents(
            dt=float(dt if dt is not None else getattr(self, "hunav_dt", 0.02)),
            stamp=stamp,
        )

    def step(self) -> None:
        """Advance PhysX without RTX (Habitat: render only on camera sample)."""
        if self._paused:
            return
        # HuNavManager owns USD people sync (and optionally SFM). Otherwise
        # mirror Traffic poses via PeopleSync.
        if self.hunav_manager is None:
            if self.people_sync is not None and self.traffic is not None:
                try:
                    self.people_sync.sync(self.traffic)
                except Exception as exc:
                    if not getattr(self, "people_sync_warned", False):
                        self.people_sync_warned = True
                        logger.warning(
                            "people.sync failed (further errors muted): %s", exc
                        )
        physics = self.physics
        if physics is not None:
            # Never render here — color_depth() owns the single Kit/RTX frame.
            physics.step(render=False)
            if self.articulation_driven:
                self.pull_articulation_pose()
                self._sync_camera_to_pose()
            return
        update_app()

    def close(self) -> None:
        # Close the USD stage only; Kit app teardown is deferred to atexit so
        # SimulationApp.close() cannot abort the process mid-finally.
        self.articulation = None
        try:
            if self.physics is not None:
                self.physics.close()
        except Exception:
            pass
        self.physics = None
        self.people_sync = None
        self.hunav_manager = None
        try:
            self.engine_sensors.close()
        except Exception:
            pass
        self.stage.close()

    def eye_height(self) -> float:
        return self.sensor_height

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
        """2D scan: ``lidar_2d.backend`` = cpu | rtx | merge (Isaac default rtx)."""
        angles = self.linspace_angles(angle_min, angle_max, num_beams)
        backend = str(self.lidar_backend or "rtx").strip().lower()
        cpu = np.full(angles.shape, np.nan, dtype=np.float64)
        rtx = np.full(angles.shape, np.nan, dtype=np.float64)
        if backend in ("cpu", "merge"):
            cpu = self.static_laser_ranges(angles, float(range_max))
            cpu = self.merge_traffic_ranges(cpu, angles, float(range_max))
        if backend in ("rtx", "merge"):
            annotated = self.engine_sensors.laser_ranges(
                angle_min,
                angle_max,
                num_beams,
                range_max,
                pose=(self.x, self.y, self.yaw),
            )
            if annotated is not None:
                rtx = np.asarray(annotated, dtype=np.float64).reshape(-1)
        if backend == "cpu":
            depths = cpu
        elif backend == "rtx":
            # Do not silently fall back to CPU floorplan — Isaac mode owns RTX.
            depths = rtx
            if not np.isfinite(depths).any():
                logger.debug("RTX lidar returned empty scan (FlatScan/warmup)")
        else:
            depths = self.merge_nearer_ranges(cpu, rtx)
        return depths.astype(np.float32)

    def read_imu(self) -> dict[str, Any] | None:
        """Isaac PhysX IMU sample, or None when unmounted / invalid."""
        sensors = getattr(self, "engine_sensors", None)
        read = getattr(sensors, "read_imu", None)
        if not callable(read):
            return None
        try:
            return read()
        except Exception:
            return None

    def laser_origin_xy(self) -> tuple[float, float]:
        """Map-frame XY of ``laser_link`` / ``base_scan`` (matches Autoviz TF)."""
        lx, ly = 0.0, 0.0
        urdf = self.urdf
        if urdf is not None:
            mounts = getattr(urdf, "mounts", {}) or {}
            for name in ("laser_link", "base_scan"):
                if name in mounts:
                    lx, ly = float(mounts[name][0]), float(mounts[name][1])
                    break
                if name in urdf.parents:
                    try:
                        mx, my, _ = UrdfModel.link_xyz(
                            name, urdf.parents, urdf.root
                        )
                        lx, ly = float(mx), float(my)
                        break
                    except Exception:
                        continue
        cosine = math.cos(float(self.yaw))
        sine = math.sin(float(self.yaw))
        return (
            float(self.x) + cosine * lx - sine * ly,
            float(self.y) + sine * lx + cosine * ly,
        )

    def static_laser_ranges(
        self, angles: np.ndarray, range_max: float
    ) -> np.ndarray:
        packed = self.scenario_grid
        if packed is None and self.map_builder is not None:
            packed = getattr(self.map_builder, "grid", None)
        out = np.full(angles.shape, np.nan, dtype=np.float64)
        if packed is None:
            return out
        grid, resolution, ox, oy, _, _ = packed
        ox_l, oy_l = self.laser_origin_xy()
        for index, angle in enumerate(np.asarray(angles, dtype=np.float64).reshape(-1)):
            hit = Map.occupancy_range(
                ox_l,
                oy_l,
                float(self.yaw) + float(angle),
                grid,
                float(resolution),
                float(ox),
                float(oy),
                float(range_max),
            )
            if hit is not None:
                out[index] = float(hit)
        return out

    def merge_traffic_ranges(
        self, depths: np.ndarray, angles: np.ndarray, range_max: float
    ) -> np.ndarray:
        traffic = self.traffic
        if traffic is None or not hasattr(traffic, "nearest_batch"):
            return depths
        dirs = np.stack(
            [
                np.cos(self.yaw + angles),
                np.sin(self.yaw + angles),
                np.zeros_like(angles),
            ],
            axis=1,
        )
        origin = np.array(
            [self.x, self.y, float(self.sensor_height)], dtype=np.float64
        )
        dynamic = traffic.nearest_batch(origin, dirs, float(range_max))
        return self.merge_nearer_ranges(depths, dynamic)

    @staticmethod
    def merge_nearer_ranges(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        left = np.asarray(a, dtype=np.float64).reshape(-1).copy()
        right = np.asarray(b, dtype=np.float64).reshape(-1)
        if right.shape != left.shape:
            return left
        better = np.isfinite(right) & (np.isnan(left) | (right < left))
        left[better] = right[better]
        return left

    def clip_laser_to_occupancy(
        self, ranges: np.ndarray, angles: np.ndarray
    ) -> np.ndarray:
        """Sensors helper: clip beams to floorplan walls when map exists."""
        mapper = self.map_builder
        clip = getattr(mapper, "clip_laser_ranges", None)
        if not callable(clip):
            return ranges
        lidar = (self.settings.get("habitat") or {}).get("sensors", {}).get(
            "lidar_2d", {}
        )
        range_max = float(lidar.get("range_max", 30.0))
        return clip(
            ranges,
            angles,
            self.x,
            self.y,
            self.yaw,
            range_max,
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
        **kwargs: Any,
    ) -> tuple[np.ndarray, np.ndarray]:
        del kwargs
        cloud, rings = self.engine_sensors.lidar_points(
            h_min, h_max, h_beams, v_min, v_max, v_rings, range_max
        )
        # Only synthesize a planar ring from /scan when using cpu/merge backends.
        backend = str(getattr(self, "lidar_backend", "rtx") or "rtx").strip().lower()
        if cloud.shape[0] == 0 and backend in ("cpu", "merge"):
            ranges = self.laser_ranges(h_min, h_max, h_beams, range_max)
            angles = self.linspace_angles(h_min, h_max, h_beams)
            pts: list[list[float]] = []
            ring_ids: list[float] = []
            for ang, dist in zip(angles, ranges):
                if not np.isfinite(dist):
                    continue
                pts.append(
                    [
                        float(dist * math.cos(ang)),
                        float(dist * math.sin(ang)),
                        0.0,
                    ]
                )
                ring_ids.append(0.0)
            if pts:
                return (
                    np.asarray(pts, dtype=np.float32),
                    np.asarray(ring_ids, dtype=np.float32),
                )
        return cloud, rings

    def color_depth(self) -> tuple[np.ndarray, np.ndarray]:
        """One RTX frame after eye pose, then sample RGB-D (Habitat timing).

        Control ticks use PhysX-only ``step(render=False)``; this is the only
        place that pays for rendering so publish rate can approach ``rate_hz``.
        """
        # Re-assert camera world pose in case PhysX overwrote the Xform since
        # the last set_pose (kinematic teleport vs Fabric cache).
        sync = getattr(self.engine_sensors, "sync_world_pose", None)
        if callable(sync):
            base_z = float(getattr(self.stage, "base_z", 0.0) or 0.0)
            sync(self.x, self.y, self.yaw, base_z=base_z)
        physics = self.physics
        if physics is not None:
            physics.render_frame()
        else:
            update_app()
        return self.engine_sensors.color_depth()

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
        points, unused_rings = self.engine_sensors.lidar_points(
            -math.pi, math.pi, 64, -0.5, 0.5, 8, float(range_max)
        )
        del unused_rings
        if points.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float32)
        # Sensor-frame → map frame (approx: rotate by yaw, translate).
        c = float(np.cos(self.yaw))
        s = float(np.sin(self.yaw))
        x = c * points[:, 0] - s * points[:, 1] + self.x
        y = s * points[:, 0] + c * points[:, 1] + self.y
        z = points[:, 2] + self.sensor_height
        return np.stack([x, y, z], axis=1).astype(np.float32)
