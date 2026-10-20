# Copyright 2026 The Openbot Authors (duyongquan)
#
# Port of Hunav_isaac_wrapper.hunav_manager — ROS 2 → Autolink / automsgs.
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

"""Spawn / step HuNav agents in Isaac USD (Hunav_isaac_wrapper layout).

Spawns SkelRoot characters under ``{world}/Characters``, sets up AnimationGraph
retargeting, and each tick packs Agents → ``/compute_agents`` (Autolink or
in-process :class:`ComputeAgentsEngine`) → writes poses back to USD.
"""

from __future__ import annotations

import logging
import math
import os
import random
from pathlib import Path
from typing import Any, List, Mapping, Optional, Tuple

import numpy as np
import yaml

from autosim.backends.isaac.hunav.animation_utils import (
    apply_animation_graph,
    create_agent_animation_graph,
    create_animation,
    find_skelroot_path,
    set_anim_graph_speed,
    setup_anim_retargeting,
)
from autosim.backends.isaac.people import (
    discover_character_root,
    people_sync_config_from_isaac,
)
from autosim.scene.hunav.codec import OBS_FAR, AgentCodec
from autosim.scene.hunav.compute import ComputeAgentsEngine
from autosim.scene.hunav.loader import load_hunav_scenario

logger = logging.getLogger(__name__)

# Isaac People character USDs (same order as Hunav_isaac_wrapper).
_CHARACTER_MODELS = [
    "F_Business_02/F_Business_02.usd",
    "F_Medical_01/F_Medical_01.usd",
    "M_Medical_01/M_Medical_01.usd",
    "male_adult_construction_01_new/male_adult_construction_01_new.usd",
    "male_adult_construction_05_new/male_adult_construction_05_new.usd",
    "female_adult_police_01_new/female_adult_police_01_new.usd",
    "female_adult_police_02/female_adult_police_02.usd",
    "female_adult_police_03_new/female_adult_police_03_new.usd",
    "male_adult_police_04/male_adult_police_04.usd",
    "original_female_adult_business_02/female_adult_business_02.usd",
    "original_female_adult_medical_01/female_adult_medical_01.usd",
]

_SKIN_TO_MODEL = {
    1: 0,
    2: 1,
    3: 2,
    4: 3,
    5: 4,
    6: 5,
    7: 6,
    8: 7,
    9: 8,
    10: 9,
    11: 10,
}

_DEFAULT_SFM = {
    "goal_force_factor": 10.0,
    "obstacle_force_factor": 2.0,
    "social_force_factor": 5.0,
}
_SFM_CONSTRAINTS = {
    "goal_force_factor": (5.0, 10.0),
    "obstacle_force_factor": (0.5, 5.0),
    "social_force_factor": (5.0, 20.0),
}


class HuNavManager:
    """Isaac People + Autolink ``/compute_agents`` (Hunav_isaac_wrapper port)."""

    def __init__(
        self,
        stage: Any,
        config_file: str | Path | None = None,
        *,
        robot_pose_fn: Any | None = None,
        people_sync: Any | None = None,
        isaac_cfg: Mapping[str, Any] | None = None,
        engine: ComputeAgentsEngine | None = None,
        compute_client: Any | None = None,
        pedestrian: Any | None = None,
        own_motion: bool = True,
        root_path: str = "/World/Characters",
    ) -> None:
        self.stage = stage
        self.config_file = Path(config_file).expanduser() if config_file else None
        self.robot_pose_fn = robot_pose_fn
        self.people_sync = people_sync
        self.codec = AgentCodec()
        self.engine = engine or ComputeAgentsEngine()
        self.compute_client = compute_client
        self.own_motion = bool(own_motion)
        self.root_path = str(root_path).rstrip("/") or "/World/Characters"

        self.config: dict[str, Any] = {}
        self.params: dict[str, Any] = {}
        if self.config_file and self.config_file.is_file():
            with self.config_file.open("r", encoding="utf-8") as handle:
                self.config = yaml.safe_load(handle) or {}
            loader = self.config.get("hunav_loader") or {}
            self.params = dict(loader.get("ros__parameters") or {})

        # Disable PeopleSync character mirroring — this manager owns USD people.
        if self.people_sync is not None:
            try:
                self.people_sync.enabled = False
            except Exception:
                pass

        isaac = isaac_cfg if isinstance(isaac_cfg, Mapping) else {}
        sync_cfg = people_sync_config_from_isaac(isaac)
        self.character_root = discover_character_root(
            str(sync_cfg.get("character_root") or "")
        )
        self.animation_root = str(sync_cfg.get("animation_root") or "").strip()
        if not self.animation_root and self.character_root:
            sibling = Path(self.character_root).parent / "Animations"
            if sibling.is_dir():
                self.animation_root = str(sibling)
        self._resolve_nucleus_fallback()

        self.target_model_paths = self._build_model_paths()
        self.skin_to_model_mapping = dict(_SKIN_TO_MODEL)

        self.agents: list[Any] = []
        self.agent_initial_states: list[dict[str, Any]] = []
        self.animation_dict: dict[int, Any] = {}
        self.source_animation_dict: dict[int, Any] = {}
        self.bound_animations: dict[Any, Any] = {}
        self.flag_anim: dict[Any, bool] = {}
        self.retarget_flag = False
        self.agent_previous_orientations: dict[Any, tuple[Any, Any]] = {}
        self.orientation_smoothing_factor = 0.15
        self.default_biped_usd = self._resolve_biped_usd()
        self.world_root = self._detect_world_root()
        self.characters_root = f"{self.world_root}/Characters"
        self.animations_prim_root = f"{self.world_root}/Animations"
        self.biped_prim_path = f"{self.world_root}/Biped_Setup"

        # Prefer Characters root from attach (PeopleSync path) when under world.
        if self.root_path.startswith(self.world_root):
            self.characters_root = self.root_path

        self._ag = None
        try:
            from isaacsim.core.utils.extensions import enable_extension

            enable_extension("omni.anim.retarget.core")
            import omni.anim.graph.core as ag

            self._ag = ag
        except Exception as exc:
            logger.warning("AnimGraph / retarget extension unavailable: %s", exc)

        if pedestrian is not None:
            self.bind_pedestrian(pedestrian)
        elif self.config_file and self.config_file.is_file():
            try:
                roster = load_hunav_scenario(self.config_file)
                self._seed_engine_from_roster(roster)
            except Exception as exc:
                logger.warning("HuNavManager roster seed failed: %s", exc)

        if self.params.get("agents"):
            self.initialize_agents()

    def _resolve_nucleus_fallback(self) -> None:
        """Fill character/animation roots from Isaac Nucleus when local missing."""
        if self.character_root and Path(self.character_root).is_dir():
            return
        try:
            from isaacsim.storage.native import get_assets_root_path

            assets = get_assets_root_path()
        except Exception:
            assets = None
        if not assets:
            return
        char = os.path.join(assets, "Isaac", "People", "Characters")
        anim = os.path.join(assets, "Isaac", "People", "Animations")
        if not self.character_root and os.path.isdir(char):
            self.character_root = char
        if not self.animation_root and os.path.isdir(anim):
            self.animation_root = anim

    def _build_model_paths(self) -> list[str]:
        root = self.character_root.rstrip("/") if self.character_root else ""
        if not root:
            return []
        paths = []
        for model in _CHARACTER_MODELS:
            full = f"{root}/{model}"
            if Path(full).is_file() or full.startswith("omniverse://"):
                paths.append(full)
        if paths:
            return paths
        # Local download layout: flat *.usd under Characters.
        try:
            from autosim.backends.isaac.people import list_character_usds

            return list_character_usds(root)
        except Exception:
            return []

    def _resolve_biped_usd(self) -> str:
        root = self.character_root
        if root:
            for name in (
                "Biped_Setup.usd",
                "biped_demo_meters.usd",
                "Biped_Setup/Biped_Setup.usd",
            ):
                path = Path(root) / name
                if path.is_file():
                    return str(path)
        try:
            from isaacsim.storage.native import get_assets_root_path

            assets = get_assets_root_path()
            if assets:
                return os.path.join(
                    assets, "Isaac", "People", "Characters", "Biped_Setup.usd"
                )
        except Exception:
            pass
        return ""

    def _detect_world_root(self) -> str:
        stage = self.stage
        for path in ("/World", "/Root"):
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                return path
        return "/World"

    def bind_pedestrian(self, pedestrian: Any) -> None:
        """Share Runner ``Traffic.pedestrian`` so laser / SFM stay aligned."""
        self.engine.pedestrian = pedestrian
        self.engine.settings = dict(pedestrian.settings)

    def _seed_engine_from_roster(self, roster: Any) -> None:
        ped = self.engine.pedestrian
        ped.settings["enabled"] = True
        ped.settings["social"] = True
        xs = [float(r["x"]) for r in roster.records]
        ys = [float(r["y"]) for r in roster.records]
        for goals in (r.get("goals") or [] for r in roster.records):
            for g in goals:
                if isinstance(g, Mapping):
                    xs.append(float(g["x"]))
                    ys.append(float(g["y"]))
                elif isinstance(g, (list, tuple)) and len(g) >= 2:
                    xs.append(float(g[0]))
                    ys.append(float(g[1]))
        if xs and ys:
            span = max(max(xs) - min(xs), max(ys) - min(ys), 4.0) + 4.0
            ped.settings["x_length"] = max(float(ped.settings.get("x_length", 10)), span)
            ped.settings["y_length"] = max(float(ped.settings.get("y_length", 10)), span)
        ped.arena = ped.make_arena()
        ped.spawn_agents(roster)

    def robot_state(self) -> tuple[float, float, float, float, float, float]:
        """Return ``(x, y, yaw, vx, vy, wz)`` from ``robot_pose_fn`` or zeros."""
        if self.robot_pose_fn is None:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        pose = self.robot_pose_fn()
        values = list(pose) + [0.0] * 6
        return tuple(float(v) for v in values[:6])  # type: ignore[return-value]

    def slerp_quaternions(self, q1: Any, q2: Any, t: float) -> Any:
        """Spherical linear interpolation between two ``Gf.Quatf``."""
        from pxr import Gf

        dot = q1.GetReal() * q2.GetReal() + sum(
            a * b for a, b in zip(q1.GetImaginary(), q2.GetImaginary())
        )
        if dot < 0.0:
            q2 = Gf.Quatf(
                -q2.GetReal(),
                -q2.GetImaginary()[0],
                -q2.GetImaginary()[1],
                -q2.GetImaginary()[2],
            )
            dot = -dot
        if dot > 0.9995:
            result_real = q1.GetReal() + t * (q2.GetReal() - q1.GetReal())
            result_imag = [
                q1.GetImaginary()[i] + t * (q2.GetImaginary()[i] - q1.GetImaginary()[i])
                for i in range(3)
            ]
            result = Gf.Quatf(
                result_real, result_imag[0], result_imag[1], result_imag[2]
            )
            return result.GetNormalized()
        theta_0 = math.acos(abs(dot))
        sin_theta_0 = math.sin(theta_0)
        theta = theta_0 * t
        sin_theta = math.sin(theta)
        s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        result_real = s0 * q1.GetReal() + s1 * q2.GetReal()
        result_imag = [
            s0 * q1.GetImaginary()[i] + s1 * q2.GetImaginary()[i] for i in range(3)
        ]
        return Gf.Quatf(
            result_real, result_imag[0], result_imag[1], result_imag[2]
        ).GetNormalized()

    def normalize_angle(self, a: float) -> float:
        value = float(a)
        while value <= -math.pi:
            value += 2.0 * math.pi
        while value > math.pi:
            value -= 2.0 * math.pi
        return value

    def get_character_model_from_skin(self, skin_value: Any) -> str | None:
        """Resolve character USD from HuNav ``skin`` (0=random, 1..N=index)."""
        if not self.target_model_paths:
            return None
        if skin_value == 0 or skin_value == "random":
            return random.choice(self.target_model_paths)
        if isinstance(skin_value, (int, float)) and int(skin_value) in self.skin_to_model_mapping:
            idx = self.skin_to_model_mapping[int(skin_value)]
            if 0 <= idx < len(self.target_model_paths):
                return self.target_model_paths[idx]
        return None

    def initialize_agents(self) -> None:
        """Spawn SkelRoot agents + AnimationGraphs from HuNav YAML (wrapper)."""
        from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

        if not self.params:
            logger.warning("HuNavManager: no config loaded, skipping agent creation")
            return
        agent_names = list(self.params.get("agents") or [])
        if not agent_names:
            logger.warning("HuNavManager: empty agents list")
            return
        if not self.target_model_paths:
            raise RuntimeError(
                "HuNavManager: no character USDs found; set "
                "habitat.isaac.traffic.character_root or Nucleus People assets"
            )
        if not self.animation_root:
            raise RuntimeError(
                "HuNavManager: no Animations folder; set "
                "habitat.isaac.traffic.animation_root"
            )

        walk_src = os.path.join(
            self.animation_root, "stand_walk_loop_in_place.skelanim.usd"
        )
        idle_src = os.path.join(self.animation_root, "stand_idle_loop.skelanim.usd")
        # Local download script may use shorter names.
        if not Path(walk_src).is_file():
            for alt in ("walk.skelanim.usd", "Walk.skelanim.usd"):
                cand = os.path.join(self.animation_root, alt)
                if Path(cand).is_file():
                    walk_src = cand
                    break
        if not Path(idle_src).is_file():
            for alt in ("idle.skelanim.usd", "Idle.skelanim.usd"):
                cand = os.path.join(self.animation_root, alt)
                if Path(cand).is_file():
                    idle_src = cand
                    break
        if not Path(walk_src).is_file() or not Path(idle_src).is_file():
            raise RuntimeError(
                f"HuNavManager: missing walk/idle animations under {self.animation_root}"
            )

        walk_anim = create_animation(
            self.stage, f"{self.animations_prim_root}/WalkLoop", walk_src
        )
        idle_anim = create_animation(
            self.stage, f"{self.animations_prim_root}/IdleLoop", idle_src
        )
        self.source_animation_dict = {
            0: idle_anim.GetPath(),
            1: walk_anim.GetPath(),
        }
        self.animation_dict = {
            0: f"{self.characters_root}/IdleLoop",
            1: f"{self.characters_root}/WalkLoop",
        }

        if self.default_biped_usd:
            init_pos_src = Gf.Vec3d(0, 0, 0)
            rot_x = Gf.Rotation(Gf.Vec3d(1, 0, 0), 90).GetQuat()
            init_rot_src = Gf.Quatf(rot_x)
            source_agent_prim = self.stage.DefinePrim(self.biped_prim_path, "SkelRoot")
            source_agent_prim.GetReferences().AddReference(self.default_biped_usd)
            xform_src = UsdGeom.Xformable(source_agent_prim)
            found_translate = False
            for op in xform_src.GetOrderedXformOps():
                if op.GetOpName() == "xformOp:translate":
                    op.Set(init_pos_src)
                    found_translate = True
                    break
            if not found_translate:
                xform_src.AddTranslateOp().Set(init_pos_src)
            xform_src.AddOrientOp().Set(init_rot_src)
            try:
                source_agent_prim.GetAttribute("visibility").Set("invisible")
            except Exception:
                pass
            source_agent_prim.CreateAttribute(
                "physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool
            ).Set(True)
            source_agent_prim.CreateAttribute(
                "physxContact:collisionEnabled", Sdf.ValueTypeNames.Bool
            ).Set(False)

        asset_cycle = self.target_model_paths.copy()
        random.shuffle(asset_cycle)
        self.agents = []
        self.agent_initial_states = []
        self.bound_animations = {}
        self.flag_anim = {}
        self.retarget_flag = False

        for agent_name in agent_names:
            agent_cfg = dict(self.params.get(agent_name) or {})
            if not agent_cfg:
                logger.warning("HuNavManager: missing config for %s", agent_name)
                continue
            skin_value = agent_cfg.get("skin", 0)
            asset_path = self.get_character_model_from_skin(skin_value)
            if asset_path is None:
                if not asset_cycle:
                    asset_cycle = self.target_model_paths.copy()
                    random.shuffle(asset_cycle)
                asset_path = asset_cycle.pop()
                logger.warning(
                    "Invalid skin %r for %s; using %s",
                    skin_value,
                    agent_name,
                    asset_path,
                )

            init_pose = agent_cfg.get("init_pose") or {}
            global_pos = Gf.Vec3d(
                float(init_pose.get("x", 0.0)),
                float(init_pose.get("y", 0.0)),
                float(init_pose.get("z", 0.0)),
            )
            h_rad = float(init_pose.get("h", 0.0))
            h_deg = h_rad * 180.0 / math.pi
            qd_x = Gf.Rotation(Gf.Vec3d(1, 0, 0), 90.0).GetQuat()
            rot_xq = Gf.Quatf(qd_x.GetReal(), Gf.Vec3f(qd_x.GetImaginary()))
            qd_z = Gf.Rotation(Gf.Vec3d(0, 0, 1), h_deg).GetQuat()
            rot_zq = Gf.Quatf(qd_z.GetReal(), Gf.Vec3f(qd_z.GetImaginary()))
            global_rot = rot_zq * rot_xq

            container_path = f"{self.characters_root}/{agent_name}"
            container = self.stage.DefinePrim(container_path, "Xform")
            xf = UsdGeom.Xformable(container)
            xf.AddTranslateOp().Set(global_pos)
            xf.AddOrientOp().Set(global_rot)
            PhysxSchema.PhysxRigidBodyAPI.Apply(container)
            UsdPhysics.RigidBodyAPI.Apply(container)

            anim_path = container_path + "/Animation"
            agent_skelroot = self.stage.DefinePrim(anim_path, "SkelRoot")
            agent_skelroot.GetReferences().AddReference(asset_path)

            if not self.retarget_flag:
                setup_anim_retargeting(
                    self.stage,
                    agent_skelroot,
                    self.source_animation_dict,
                    self.characters_root,
                    source_biped_path=f"{self.biped_prim_path}/biped_demo_meters",
                )
                self.retarget_flag = True

            anim_graph_path = create_agent_animation_graph(
                self.stage,
                agent_skelroot,
                idle_anim_path=self.animation_dict.get(0),
                walk_anim_path=self.animation_dict.get(1),
            )
            skelroot_path = find_skelroot_path(agent_skelroot)
            apply_animation_graph(
                self.stage.GetPrimAtPath(skelroot_path),
                anim_graph_path,
            )
            self.bound_animations[agent_skelroot.GetPath()] = anim_graph_path
            self.flag_anim[agent_skelroot.GetPath()] = False
            self.agents.append(container)
            self.agent_initial_states.append(
                {"position": global_pos, "orientation": global_rot}
            )

        logger.info(
            "HuNavManager initialized %d agents under %s",
            len(self.agents),
            self.characters_root,
        )

    def reset_agent_states(self) -> None:
        for agent, init_state in zip(self.agents, self.agent_initial_states):
            agent.GetAttribute("xformOp:translate").Set(init_state["position"])
            agent.GetAttribute("xformOp:orient").Set(init_state["orientation"])
        self.agent_previous_orientations.clear()

    def generate_lasers(self, num_lasers: int) -> List[Any]:
        from pxr import Gf

        directions = []
        angle_increment = 360.0 / max(int(num_lasers), 1)
        for i in range(int(num_lasers)):
            angle_rad = math.radians(i * angle_increment)
            directions.append(
                Gf.Vec3f(math.cos(angle_rad), math.sin(angle_rad), 0.0)
            )
        return directions

    def get_closest_obstacles(
        self,
        agent_position: Any,
        max_distance: float,
        sensor_offsets: List[float],
        num_lasers: int = 90,
    ) -> List[Tuple[float, Optional[Any]]]:
        try:
            import omni.physx
        except Exception:
            return [(max_distance, None)] * int(num_lasers)

        directions = self.generate_lasers(num_lasers)
        scene_query = omni.physx.get_physx_scene_query_interface()
        closest_hits: List[Tuple[float, Optional[Any]]] = []
        from pxr import Gf

        for direction in directions:
            best_distance = max_distance
            best_hit = None
            hit_found = False
            for offset in sensor_offsets:
                sensor_origin = Gf.Vec3f(
                    float(agent_position[0]),
                    float(agent_position[1]),
                    float(agent_position[2]) + float(offset),
                )
                hit = scene_query.raycast_closest(
                    sensor_origin, direction, max_distance
                )
                if hit.get("hit", False):
                    hit_found = True
                    distance = hit.get("distance", max_distance)
                    if distance < best_distance:
                        best_distance = distance
                        best_hit = hit.get("position")
            if hit_found:
                closest_hits.append((best_distance, best_hit))
            else:
                closest_hits.append((max_distance, None))
        return closest_hits

    def euler_from_quaternion(
        self, x: float, y: float, z: float, w: float
    ) -> Tuple[float, float, float]:
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return float(roll), float(pitch), float(yaw)

    def send_agents(
        self, *, dt: float = 0.02, stamp: tuple[int, int] = (0, 0)
    ) -> Any | None:
        """One HuNav step: pack USD agents → compute → apply USD + Pedestrian."""
        if not self.agents:
            return None
        if not self.own_motion:
            return self._mirror_pedestrian_to_usd(stamp=stamp)

        if not self.codec.available:
            raise RuntimeError(
                "HuNavManager requires automsgs hunav_msgs; rebuild automsgs"
            )

        agents_msg = self._pack_agents_from_usd(stamp=stamp)
        rx, ry, ryaw, vx, vy, wz = self.robot_state()
        robot_msg = self.codec.pack_robot(rx, ry, ryaw, vx=vx, vy=vy, wz=wz)

        updated = None
        if self.compute_client is not None:
            updated = self._call_remote(agents_msg, robot_msg, dt=dt)
        if updated is None:
            updated = self.engine.step(agents_msg, robot_msg, dt=dt, stamp=stamp)

        self._update_agents(updated)
        # Keep Traffic.pedestrian aligned for /scan and Autolink publisher.
        ped = self.engine.pedestrian
        if ped is not None and ped.position.shape[0] > 0:
            self.codec.apply_agents(ped, updated)
        return updated

    # Alias used by older call sites / wrapper naming.
    send_agents_msg = send_agents

    def _mirror_pedestrian_to_usd(
        self, *, stamp: tuple[int, int] = (0, 0)
    ) -> Any | None:
        """When ``own_motion`` is false, push Pedestrian poses into USD containers."""
        from pxr import Gf

        ped = self.engine.pedestrian
        n = min(len(self.agents), int(ped.position.shape[0]))
        for i in range(n):
            agent_prim = self.agents[i]
            pos = Gf.Vec3d(
                float(ped.position[i, 0]),
                float(ped.position[i, 1]),
                float(ped.settings.get("character_z", 0.0) or 0.0),
            )
            agent_prim.GetAttribute("xformOp:translate").Set(pos)
        if self.codec.available and n > 0:
            return self.codec.pack_agents(ped, stamp=stamp, frame_id="map")
        return None

    def _pack_agents_from_usd(self, *, stamp: tuple[int, int]) -> Any:
        agent_pb2, agents_pb2, _ = self.codec._pb  # type: ignore[misc]
        agents_msg = agents_pb2.Agents()
        agents_msg.header.stamp.sec = int(stamp[0])
        agents_msg.header.stamp.nanosec = int(stamp[1])
        agents_msg.header.frame_id = "world"
        for idx, agent_prim in enumerate(self.agents):
            agents_msg.agents.append(self._create_agent_msg(agent_prim, idx))
        return agents_msg

    def _create_agent_msg(self, agent_prim: Any, index: int) -> Any:
        agent_pb2, _, _ = self.codec._pb  # type: ignore[misc]
        agent_names = list(self.params.get("agents") or [])
        agent_ref = agent_names[index] if index < len(agent_names) else f"agent{index + 1}"
        agent_cfg = dict(self.params.get(agent_ref) or {})

        agent = agent_pb2.Agent()
        agent.id = int(agent_cfg.get("id", index + 1))
        agent.type = agent_pb2.Agent.PERSON
        agent.skin = 0
        agent.name = f"Agent{index + 1}"
        agent.group_id = int(agent_cfg.get("group_id", -1))
        agent.radius = float(agent_cfg.get("radius", 0.4))
        agent.desired_velocity = float(agent_cfg.get("max_vel", 1.5))

        pos = agent_prim.GetAttribute("xformOp:translate").Get()
        rot = agent_prim.GetAttribute("xformOp:orient").Get()
        rw = float(rot.GetReal())
        rx, ry, rz = [float(v) for v in rot.GetImaginary()]
        agent.position.position.x = float(pos[0])
        agent.position.position.y = float(pos[1])
        agent.position.position.z = float(pos[2])
        agent.position.orientation.x = rx
        agent.position.orientation.y = ry
        agent.position.orientation.z = rz
        agent.position.orientation.w = rw
        _, _, yaw = self.euler_from_quaternion(rx, ry, rz, rw)
        agent.yaw = self.normalize_angle(yaw - math.pi / 2.0)

        lin = agent_prim.GetAttribute("physics:velocity").Get()
        ang = agent_prim.GetAttribute("physics:angularVelocity").Get()
        if lin is None:
            lin = (0.0, 0.0, 0.0)
        if ang is None:
            ang = (0.0, 0.0, 0.0)
        agent.linear_vel = float(np.linalg.norm(lin))
        agent.angular_vel = float(np.linalg.norm(ang))
        agent.velocity.linear.x = float(lin[0])
        agent.velocity.linear.y = float(lin[1])
        agent.velocity.linear.z = float(lin[2])
        agent.velocity.angular.x = float(ang[0])
        agent.velocity.angular.y = float(ang[1])
        agent.velocity.angular.z = float(ang[2])

        agent.cyclic_goals = bool(agent_cfg.get("cyclic_goals", True))
        agent.goal_radius = float(agent_cfg.get("goal_radius", 0.3))

        beh = dict(agent_cfg.get("behavior") or {})
        configuration = int(beh.get("configuration", 0))
        if configuration == 0:
            sfm = dict(_DEFAULT_SFM)
            sfm["other_force_factor"] = float(beh.get("other_force_factor", 20.0))
        else:
            sfm = {
                "social_force_factor": float(beh.get("social_force_factor", 5.0)),
                "goal_force_factor": float(beh.get("goal_force_factor", 10.0)),
                "obstacle_force_factor": float(beh.get("obstacle_force_factor", 2.0)),
                "other_force_factor": float(beh.get("other_force_factor", 20.0)),
            }
            if configuration != 1:
                for key, (lo, hi) in _SFM_CONSTRAINTS.items():
                    sfm[key] = max(lo, min(hi, sfm[key]))

        agent.behavior.state = 1
        agent.behavior.configuration = configuration
        agent.behavior.social_force_factor = sfm["social_force_factor"]
        agent.behavior.goal_force_factor = sfm["goal_force_factor"]
        agent.behavior.obstacle_force_factor = sfm["obstacle_force_factor"]
        agent.behavior.other_force_factor = sfm["other_force_factor"]

        # Goals from global_goals index list.
        global_goals = self.params.get("global_goals") or {}
        for gid in agent_cfg.get("goals") or []:
            gcfg = global_goals.get(gid) or global_goals.get(str(gid)) or {}
            if not gcfg:
                continue
            g = agent.goals.add()
            g.position.x = float(gcfg.get("x", 0.0))
            g.position.y = float(gcfg.get("y", 0.0))
            g.position.z = float(gcfg.get("z", 0.0))
            g.orientation.w = 1.0

        max_distance = 4.0
        sensor_offsets = [0.05, 0.1, 0.25, 0.5, 1.0]
        hits = self.get_closest_obstacles(pos, max_distance, sensor_offsets)
        for hit in hits:
            pt = agent.closest_obs.add()
            if hit[1] is not None:
                pt.x = float(hit[1][0])
                pt.y = float(hit[1][1])
                pt.z = float(hit[1][2])
            else:
                pt.x = OBS_FAR
                pt.y = OBS_FAR
                pt.z = OBS_FAR
        return agent

    def _call_remote(
        self, agents_msg: Any, robot_msg: Any, *, dt: float
    ) -> Any | None:
        from autosim.scene.hunav.client import call_compute_agents

        try:
            return call_compute_agents(
                self.compute_client, agents_msg, robot_msg, dt=dt
            )
        except Exception as exc:
            logger.debug("HuNavManager remote compute failed: %s", exc)
            return None

    def _update_agents(self, updated_agents: Any) -> None:
        from pxr import Gf

        if updated_agents is None:
            return
        rows = list(updated_agents.agents)
        for upd in rows:
            idx = int(upd.id) - 1
            if idx < 0 or idx >= len(self.agents):
                continue
            agent_prim = self.agents[idx]
            agent_skelroot_prim = self.stage.GetPrimAtPath(
                agent_prim.GetPath().AppendChild("Animation")
            )
            anim_graph_path = self.bound_animations.get(agent_skelroot_prim.GetPath())

            skel_path = find_skelroot_path(agent_skelroot_prim)
            if not self.flag_anim.get(agent_skelroot_prim.GetPath(), False):
                try:
                    self.stage.GetPrimAtPath(skel_path).SetMetadata("kind", "component")
                except Exception:
                    pass
                self.flag_anim[agent_skelroot_prim.GetPath()] = True

            char = None
            if self._ag is not None:
                try:
                    char = self._ag.get_character(str(skel_path))
                except Exception:
                    char = None

            new_pos = Gf.Vec3d(
                float(upd.position.position.x),
                float(upd.position.position.y),
                float(upd.position.position.z),
            )
            agent_prim.GetAttribute("xformOp:translate").Set(new_pos)

            new_quat = Gf.Quatf(
                float(upd.position.orientation.w),
                float(upd.position.orientation.x),
                float(upd.position.orientation.y),
                float(upd.position.orientation.z),
            )
            rot_x = Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat()
            rot_xq = Gf.Quatf(rot_x)
            rot_z = Gf.Rotation(Gf.Vec3d(1, 0, 0), 90).GetQuat()
            rot_zq = Gf.Quatf(rot_z)
            target_prim_quat = new_quat * rot_xq * rot_zq
            rot_zq_anim = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), 0).GetQuat())
            target_anim_quat = new_quat * rot_xq * rot_zq_anim

            agent_path = agent_prim.GetPath()
            if agent_path in self.agent_previous_orientations:
                prev_prim, prev_anim = self.agent_previous_orientations[agent_path]
                smoothed_prim = self.slerp_quaternions(
                    prev_prim, target_prim_quat, self.orientation_smoothing_factor
                )
                smoothed_anim = self.slerp_quaternions(
                    prev_anim, target_anim_quat, self.orientation_smoothing_factor
                )
            else:
                smoothed_prim = target_prim_quat
                smoothed_anim = target_anim_quat
            self.agent_previous_orientations[agent_path] = (
                smoothed_prim,
                smoothed_anim,
            )
            agent_prim.GetAttribute("xformOp:orient").Set(smoothed_prim)

            lin = Gf.Vec3d(
                float(upd.velocity.linear.x),
                float(upd.velocity.linear.y),
                float(upd.velocity.linear.z),
            )
            if char is not None:
                try:
                    import carb

                    pos_carb = carb.Float3(new_pos[0], new_pos[1], new_pos[2])
                    real = smoothed_anim.GetReal()
                    imag = smoothed_anim.GetImaginary()
                    rot_carb = carb.Float4(imag[0], imag[1], imag[2], real)
                    char.set_world_transform(pos_carb, rot_carb)
                except Exception as exc:
                    logger.debug("anim graph set_world_transform failed: %s", exc)

            try:
                agent_prim.GetAttribute("physics:velocity").Set(lin)
                ang = Gf.Vec3d(
                    float(upd.velocity.angular.x),
                    float(upd.velocity.angular.y),
                    float(upd.velocity.angular.z),
                )
                agent_prim.GetAttribute("physics:angularVelocity").Set(ang)
            except Exception:
                pass

            speed = float(np.linalg.norm(lin))
            normalized_speed = float(np.clip(speed / 1.5, 0.0, 1.0))
            if anim_graph_path:
                set_anim_graph_speed(
                    self.stage, char, anim_graph_path, normalized_speed
                )

    def poses(self) -> np.ndarray:
        """Current planar agent poses ``(N, 3)`` from Pedestrian."""
        return self.engine.pedestrian.poses()
