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

"""Walking pedestrians inside the scenario bounds.

Default mode: free-space goals, wall skirting, separation. With
``social: true``, motion uses a Social Force Model and per-agent reactions
to the robot. Optional ``agents`` YAML lists poses, goals, and behaviors.
"""

from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from autosim.scene.agents.behavior import Behavior
from autosim.scene.agents.force import Force
from autosim.scene.agents.roster import Agents
from autosim.scene.arena import Arena
from autosim.scene.field import Field

logger = logging.getLogger(__name__)


class Pedestrian:
    """Goal-directed walkers; optional SFM + robot reaction profiles."""

    DEFAULTS: dict[str, Any] = {
        "enabled": False,
        "count": 8,
        "radius": 0.2,
        "height": 1.7,
        "speed": 1.2,
        "seed": 1,
        "x_length": 10.0,
        "y_length": 10.0,
        "spawn_radius": 1.0,
        "goal_tolerance": 0.4,
        "stuck_steps": 25,
        "social": False,
        "behavior": "regular",
        "agents": "",
        "notice": 5.0,
        "approach": 0.55,
        "scare": 2.5,
        "block": 1.2,
        "tau": 0.5,
        "A": 2.1,
        "B": 0.3,
        "A_obs": 10.0,
        "B_obs": 0.2,
        "goal_force_factor": 1.0,
        "social_force_factor": 1.0,
        "obstacle_force_factor": 1.0,
        # Navmesh wandering: pick far free-space goals instead of short loops.
        "wander": False,
        "wander_min_dist": 3.5,
        "spread_spawn": False,
        "spread_min_dist": 2.0,
        "social_margin": 0.08,
    }

    def __init__(self, settings: Mapping[str, Any] | None = None) -> None:
        """Copy settings and spawn when ``enabled`` is true.

        Args:
            settings: ``habitat.scenario.pedestrian`` mapping.
        """
        merged = dict(self.DEFAULTS)
        merged.update(dict(settings or {}))
        self.settings = merged
        self.position = np.zeros((0, 2), dtype=np.float64)
        self.velocity = np.zeros((0, 2), dtype=np.float64)
        self.goal = np.zeros((0, 2), dtype=np.float64)
        self.yaw = np.zeros((0,), dtype=np.float64)
        self.speed = np.zeros((0,), dtype=np.float64)
        self.stuck = np.zeros((0,), dtype=np.int32)
        self.kinds = np.zeros((0,), dtype=object)
        self.agent_ids = np.zeros((0,), dtype=np.int32)
        self.agent_names = np.zeros((0,), dtype=object)
        self.group_ids = np.zeros((0,), dtype=np.int32)
        self.skins = np.zeros((0,), dtype=np.int32)
        self.avatars = np.zeros((0,), dtype=object)
        self.radii = np.zeros((0,), dtype=np.float64)
        self.goal_radii = np.zeros((0,), dtype=np.float64)
        self.beh_config = np.zeros((0,), dtype=np.int32)
        self.goal_force_factor = np.zeros((0,), dtype=np.float64)
        self.social_force_factor = np.zeros((0,), dtype=np.float64)
        self.obstacle_force_factor = np.zeros((0,), dtype=np.float64)
        self.other_force_factor = np.zeros((0,), dtype=np.float64)
        self.goal_lists: list[list[tuple[float, float]]] = []
        self.goal_index = np.zeros((0,), dtype=np.int32)
        self.cyclic = np.zeros((0,), dtype=bool)
        self.arena: Arena | None = None
        self.field: Field | None = None
        # Optional Habitat pathfinder — hard constraint onto navigable surface.
        self.pathfinder: Any = None
        self.nav_floor_y: float = 0.0
        # Per-agent navmesh polylines (map x/y); SFM steers to the next vertex.
        self.path_waypoints: list[list[tuple[float, float]]] = []
        self.force = Force(merged)
        self.behavior = Behavior(
            notice=float(merged["notice"]),
            approach=float(merged["approach"]),
            scare=float(merged["scare"]),
            block=float(merged["block"]),
        )
        self.rng = np.random.default_rng(int(self.settings["seed"]))
        if self.settings.get("enabled", False):
            self.spawn()

    @property
    def social(self) -> bool:
        """True when SFM + robot reactions are active."""
        return bool(self.settings.get("social", False)) or bool(
            str(self.settings.get("agents") or "").strip()
        )

    def spawn(self) -> None:
        """Place walkers from an agents YAML, or sample the arena."""
        if float(self.settings["height"]) <= 0.0:
            raise ValueError("scenario.pedestrian.height must be > 0")
        self.arena = self.make_arena()
        loaded = self.load_agents()
        if loaded is not None:
            self.spawn_agents(loaded)
        else:
            self.spawn_random()
        delta = self.goal - self.position
        self.yaw = np.arctan2(delta[:, 1], delta[:, 0])
        self.velocity = np.zeros_like(self.position)

    def load_agents(self) -> Agents | None:
        """Parse ``agents`` path or inline list; ``None`` keeps random spawn."""
        raw = self.settings.get("agents")
        if raw in (None, "", [], {}):
            return None
        if isinstance(raw, (str, Path)):
            path = Path(str(raw)).expanduser()
            if not path.is_absolute():
                try:
                    from autosim.runtime.paths import project_root

                    path = (project_root() / path).resolve()
                except Exception:
                    path = path.resolve()
            # HuNavSim / Hunav_isaac_wrapper YAML (hunav_loader + global_goals).
            try:
                text = path.read_text(encoding="utf-8")
            except OSError:
                text = ""
            if "hunav_loader" in text or "global_goals" in text:
                from autosim.scene.hunav.loader import load_hunav_scenario

                return load_hunav_scenario(path)
            return Agents.load(path)
        if isinstance(raw, (list, Mapping)):
            return Agents.from_mapping(raw)
        raise ValueError("scenario.pedestrian.agents must be a path or mapping")

    def spawn_agents(self, agents: Agents) -> None:
        """Install agents from a YAML / mapping."""
        self.settings["social"] = True
        self.settings["count"] = agents.count
        if agents.records:
            self.settings["radius"] = float(agents.records[0]["radius"])
            self.settings["height"] = float(agents.records[0]["height"])
            # Grow arena so HuNav warehouse/hospital spawns are not clipped.
            xs = [float(r["x"]) for r in agents.records]
            ys = [float(r["y"]) for r in agents.records]
            for goals in agents.goal_lists():
                for gx, gy in goals:
                    xs.append(float(gx))
                    ys.append(float(gy))
            if xs and ys:
                span = max(max(xs) - min(xs), max(ys) - min(ys), 4.0) + 4.0
                self.settings["x_length"] = max(
                    float(self.settings.get("x_length", 10.0)), span
                )
                self.settings["y_length"] = max(
                    float(self.settings.get("y_length", 10.0)), span
                )
                self.arena = self.make_arena()
        n = agents.count
        self.position = agents.positions()
        self.velocity = np.zeros((n, 2), dtype=np.float64)
        self.goal = agents.first_goals()
        self.yaw = agents.yaws()
        self.speed = agents.speeds()
        self.kinds = agents.behaviors()
        self.agent_ids = np.array(
            [int(row.get("id", i + 1)) for i, row in enumerate(agents.records)],
            dtype=np.int32,
        )
        self.agent_names = np.array(
            [str(row.get("name") or f"agent{i + 1}") for i, row in enumerate(agents.records)],
            dtype=object,
        )
        self.group_ids = np.array(
            [int(row.get("group_id", -1)) for row in agents.records], dtype=np.int32
        )
        self.skins = np.array(
            [int(row.get("skin", 0) or 0) for row in agents.records], dtype=np.int32
        )
        self.avatars = np.array(
            [str(row.get("avatar") or "") for row in agents.records], dtype=object
        )
        self.radii = np.array(
            [float(row.get("radius", self.settings.get("radius", 0.2))) for row in agents.records],
            dtype=np.float64,
        )
        self.goal_radii = np.array(
            [
                float(row.get("goal_tolerance", self.settings.get("goal_tolerance", 0.4)))
                for row in agents.records
            ],
            dtype=np.float64,
        )
        self.beh_config = np.array(
            [int(row.get("behavior_configuration", 0) or 0) for row in agents.records],
            dtype=np.int32,
        )
        default_goal = float(self.settings.get("goal_force_factor", 1.0))
        default_social = float(self.settings.get("social_force_factor", 1.0))
        default_obs = float(self.settings.get("obstacle_force_factor", 1.0))
        self.goal_force_factor = np.array(
            [
                float((row.get("force") or {}).get("goal_force_factor", default_goal))
                for row in agents.records
            ],
            dtype=np.float64,
        )
        self.social_force_factor = np.array(
            [
                float((row.get("force") or {}).get("social_force_factor", default_social))
                for row in agents.records
            ],
            dtype=np.float64,
        )
        self.obstacle_force_factor = np.array(
            [
                float((row.get("force") or {}).get("obstacle_force_factor", default_obs))
                for row in agents.records
            ],
            dtype=np.float64,
        )
        self.other_force_factor = np.array(
            [
                float((row.get("force") or {}).get("other_force_factor", 1.0))
                for row in agents.records
            ],
            dtype=np.float64,
        )
        self.goal_lists = agents.goal_lists()
        self.goal_index = np.zeros((n,), dtype=np.int32)
        self.cyclic = np.array(
            [bool(row["cyclic_goals"]) for row in agents.records], dtype=bool
        )
        self.stuck = np.zeros((n,), dtype=np.int32)
        if n:
            self.settings["radius"] = float(self.radii[0])
            self.settings["goal_tolerance"] = float(self.goal_radii[0])
        if self.field is not None and self.arena is not None:
            self.position = self.field.relocate(
                self.arena, self.position, float(self.settings["radius"]), self.rng
            )

    def spawn_random(self) -> None:
        """Uniform arena samples with a shared reaction profile."""
        count = int(self.settings["count"])
        if count < 0:
            raise ValueError("scenario.pedestrian.count must be >= 0")
        kind = Behavior.kind_name(self.settings.get("behavior", "regular"))
        self.position = self.arena.sample(self.rng, count)
        self.velocity = np.zeros((count, 2), dtype=np.float64)
        self.goal = self.sample_goals(count)
        self.yaw = np.zeros((count,), dtype=np.float64)
        self.speed = np.full((count,), float(self.settings["speed"]), dtype=np.float64)
        self.kinds = np.array([kind] * count, dtype=object)
        self.agent_ids = np.arange(1, count + 1, dtype=np.int32)
        self.agent_names = np.array(
            [f"agent{i + 1}" for i in range(count)], dtype=object
        )
        self.group_ids = np.full((count,), -1, dtype=np.int32)
        self.skins = np.zeros((count,), dtype=np.int32)
        self.avatars = np.array([""] * count, dtype=object)
        self.radii = np.full(
            (count,), float(self.settings.get("radius", 0.2)), dtype=np.float64
        )
        self.goal_radii = np.full(
            (count,), float(self.settings.get("goal_tolerance", 0.4)), dtype=np.float64
        )
        self.beh_config = np.zeros((count,), dtype=np.int32)
        self.goal_force_factor = np.full(
            (count,), float(self.settings.get("goal_force_factor", 1.0)), dtype=np.float64
        )
        self.social_force_factor = np.full(
            (count,), float(self.settings.get("social_force_factor", 1.0)), dtype=np.float64
        )
        self.obstacle_force_factor = np.full(
            (count,),
            float(self.settings.get("obstacle_force_factor", 1.0)),
            dtype=np.float64,
        )
        self.other_force_factor = np.ones((count,), dtype=np.float64)
        self.goal_lists = [[(float(x), float(y))] for x, y in self.goal]
        self.goal_index = np.zeros((count,), dtype=np.int32)
        self.cyclic = np.ones((count,), dtype=bool)
        self.stuck = np.zeros((count,), dtype=np.int32)

    def step(
        self,
        dt: float,
        bodies: np.ndarray | None = None,
        body_radius: float = 0.0,
        robot: tuple[float, float, float] | np.ndarray | None = None,
    ) -> None:
        """Advance walkers; ``robot`` is ``(x, y, yaw)`` for social reactions.

        Args:
            dt: Time step in seconds.
            bodies: ``(M, 2)`` obstacle centers.
            body_radius: Radius of every center in ``bodies``.
            robot: Optional robot pose for social reactions.
        """
        if dt < 0.0:
            raise ValueError("dt must be >= 0")
        if self.arena is None or self.position.shape[0] == 0:
            return
        # Habitat navmesh wander: try_step along ShortestPath + social avoidance.
        if self.pathfinder is not None and bool(self.settings.get("wander", False)):
            self.step_navmesh_wander(
                float(dt),
                bodies=bodies,
                body_radius=float(body_radius),
                robot=robot,
            )
            return
        if self.social:
            self.step_social(float(dt), bodies, float(body_radius), robot)
        else:
            self.step_simple(float(dt), bodies, float(body_radius))

    def step_simple(
        self,
        dt: float,
        bodies: np.ndarray | None,
        body_radius: float,
    ) -> None:
        """Legacy goal walk with separation and wall skirting."""
        self.follow_nav_paths()
        velocity = self.limit_speed(self.desired_velocity() + self.separation())
        start = np.array(self.position, dtype=np.float64, copy=True)
        proposed = self.choose_step(start, velocity, dt)
        proposed = self.arena.leave_spawn(proposed)
        proposed = self.clear_bodies(proposed, bodies, body_radius)
        self.position = self.keep_free(start, proposed)
        self.velocity = (self.position - start) / max(dt, 1e-6)
        self.face(start)
        self.note_stuck(start)
        self.refresh_goals()

    def step_social(
        self,
        dt: float,
        bodies: np.ndarray | None,
        body_radius: float,
        robot: tuple[float, float, float] | np.ndarray | None,
    ) -> None:
        """SFM integration with robot reaction profiles."""
        self.follow_nav_paths()
        robot_xy, robot_yaw = self.robot_state(robot)
        preferred, impassive = self.behavior.preferred(
            self.kinds,
            self.position,
            self.goal,
            self.speed,
            robot_xy,
            robot_yaw,
        )
        force_bodies, force_radius = self.combine_bodies(
            bodies, body_radius, robot_xy, impassive
        )
        accel = self.force.acceleration(
            self.position,
            self.velocity,
            preferred,
            float(self.settings["radius"]),
            force_bodies,
            force_radius,
            goal_force_factor=self.goal_force_factor,
            social_force_factor=self.social_force_factor,
            obstacle_force_factor=self.obstacle_force_factor,
        )
        self.velocity = self.limit_speed(self.velocity + accel * dt, self.speed)
        start = np.array(self.position, dtype=np.float64, copy=True)
        # Integrate the SFM velocity directly. choose_step's goal-cost ranking
        # would cancel scared / curious reactions that move away from the goal.
        proposed = self.place_step(start, self.velocity, dt)
        proposed = self.arena.leave_spawn(proposed)
        if bodies is not None:
            proposed = self.clear_bodies(proposed, bodies, body_radius)
        self.position = self.keep_free(start, proposed)
        self.velocity = (self.position - start) / max(dt, 1e-6)
        self.face(start)
        self.note_stuck(start)
        self.refresh_goals()

    @staticmethod
    def robot_state(
        robot: tuple[float, float, float] | np.ndarray | None,
    ) -> tuple[np.ndarray | None, float]:
        """Unpack ``(x, y, yaw)``; missing pose → ``(None, 0)``."""
        if robot is None:
            return None, 0.0
        values = np.asarray(robot, dtype=np.float64).reshape(-1)
        if values.size < 2:
            return None, 0.0
        yaw = float(values[2]) if values.size >= 3 else 0.0
        return values[:2].copy(), yaw

    def combine_bodies(
        self,
        bodies: np.ndarray | None,
        body_radius: float,
        robot_xy: np.ndarray | None,
        impassive: np.ndarray,
    ) -> tuple[np.ndarray | None, float]:
        """Append the robot as a static body for impassive agents only.

        When any agent is impassive, the robot is merged into the obstacle
        force set. Regular agents still feel the robot through preferred
        velocity shaping and (via leave_spawn) the spawn disk.
        """
        parts = []
        radius = float(body_radius)
        if bodies is not None and np.asarray(bodies).size:
            parts.append(np.asarray(bodies, dtype=np.float64).reshape(-1, 2))
        if robot_xy is not None and np.any(impassive):
            parts.append(robot_xy.reshape(1, 2))
            radius = max(radius, float(self.settings.get("spawn_radius", 0.4)))
        if not parts:
            return None, radius
        return np.concatenate(parts, axis=0), radius

    def poses(self) -> np.ndarray:
        """Planar poses ``(x, y, yaw)``.

        Returns:
            ``(N, 3)`` float64.
        """
        if self.position.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float64)
        return np.column_stack((self.position, self.yaw))

    def people(self) -> list[dict[str, Any]]:
        """People records for logging or Autolink export."""
        rows = []
        for index in range(self.position.shape[0]):
            rows.append(
                {
                    "id": index + 1,
                    "x": float(self.position[index, 0]),
                    "y": float(self.position[index, 1]),
                    "yaw": float(self.yaw[index]),
                    "vx": float(self.velocity[index, 0]),
                    "vy": float(self.velocity[index, 1]),
                    "behavior": str(self.kinds[index]),
                }
            )
        return rows

    def surface_points(self, resolution: float = 0.15) -> np.ndarray:
        """Surface samples of every upright capsule for the global map cloud."""
        if self.position.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float32)
        radius = self.hit_radius()
        height = float(self.settings["height"])
        step = max(1e-3, float(resolution))
        chunks = [
            Pedestrian.capsule_surface(float(x), float(y), radius, height, step)
            for x, y in self.position
        ]
        return np.concatenate(chunks, axis=0).astype(np.float32)

    @staticmethod
    def capsule_surface(
        cx: float, cy: float, radius: float, height: float, resolution: float
    ) -> np.ndarray:
        """Vertical capsule: cylinder wall + hemispherical caps."""
        if height <= 2.0 * radius:
            return Pedestrian.sphere_surface(
                cx, cy, 0.5 * height, min(radius, 0.5 * height), resolution
            )
        z0 = radius
        z1 = height - radius
        n_theta = max(8, int(np.ceil(2.0 * np.pi * radius / resolution)))
        n_z = max(2, int(np.ceil((z1 - z0) / resolution)) + 1)
        n_phi = max(4, int(np.ceil(0.5 * np.pi * radius / resolution)))
        theta = np.linspace(0.0, 2.0 * np.pi, n_theta, endpoint=False)
        zs = np.linspace(z0, z1, n_z)
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        wall = np.column_stack(
            [
                np.tile(cx + radius * cos_t, n_z),
                np.tile(cy + radius * sin_t, n_z),
                np.repeat(zs, n_theta),
            ]
        )
        caps = []
        for z_c, phis in (
            (z1, np.linspace(0.0, 0.5 * np.pi, n_phi)),
            (z0, np.linspace(-0.5 * np.pi, 0.0, n_phi)),
        ):
            tt, pp = np.meshgrid(theta, phis, indexing="xy")
            caps.append(
                np.column_stack(
                    (
                        cx + radius * np.cos(pp.ravel()) * np.cos(tt.ravel()),
                        cy + radius * np.cos(pp.ravel()) * np.sin(tt.ravel()),
                        z_c + radius * np.sin(pp.ravel()),
                    )
                )
            )
        return np.concatenate([wall, *caps], axis=0)

    @staticmethod
    def sphere_surface(
        cx: float, cy: float, cz: float, radius: float, resolution: float
    ) -> np.ndarray:
        """Latitude–longitude samples on a sphere."""
        n_theta = max(8, int(np.ceil(2.0 * np.pi * radius / resolution)))
        n_phi = max(6, int(np.ceil(np.pi * radius / resolution)))
        theta = np.linspace(0.0, 2.0 * np.pi, n_theta, endpoint=False)
        phi = np.linspace(-0.5 * np.pi, 0.5 * np.pi, n_phi)
        tt, pp = np.meshgrid(theta, phi, indexing="xy")
        return np.column_stack(
            (
                cx + radius * np.cos(pp.ravel()) * np.cos(tt.ravel()),
                cy + radius * np.cos(pp.ravel()) * np.sin(tt.ravel()),
                cz + radius * np.sin(pp.ravel()),
            )
        )

    def desired_velocity(self) -> np.ndarray:
        """Preferred velocity of length ``speed`` toward each goal."""
        delta = self.goal - self.position
        distance = np.linalg.norm(delta, axis=1, keepdims=True)
        direction = np.divide(delta, np.maximum(distance, 1e-6))
        speeds = self.speed if self.speed.shape[0] else float(self.settings["speed"])
        return direction * np.asarray(speeds, dtype=np.float64).reshape(-1, 1)

    def separation(self) -> np.ndarray:
        """Repulsion from other pedestrians inside two radii."""
        delta = self.position[:, None, :] - self.position[None, :, :]
        distance = np.linalg.norm(delta, axis=-1)
        np.fill_diagonal(distance, np.inf)
        overlap = np.clip(2.0 * float(self.settings["radius"]) - distance, 0.0, None)
        direction = delta / np.maximum(distance, 1e-6)[..., None]
        return (direction * overlap[..., None]).sum(axis=1)

    def limit_speed(
        self, velocity: np.ndarray, speeds: np.ndarray | float | None = None
    ) -> np.ndarray:
        """Cap each walker at its preferred speed."""
        count = int(velocity.shape[0])
        if speeds is None:
            limit = np.full((count, 1), float(self.settings["speed"]))
        else:
            values = np.asarray(speeds, dtype=np.float64).reshape(-1)
            if values.size == 1:
                limit = np.full((count, 1), float(values[0]))
            else:
                limit = values.reshape(-1, 1)
        norm = np.linalg.norm(velocity, axis=1, keepdims=True)
        scale = np.ones_like(norm)
        fast = norm[:, 0] > limit[:, 0]
        scale[fast] = limit[fast] / np.maximum(norm[fast], 1e-6)
        return velocity * scale

    def choose_step(self, start: np.ndarray, velocity: np.ndarray, dt: float) -> np.ndarray:
        """Step forward, or along the wall when that way is blocked."""
        best = start
        best_cost = np.full(start.shape[0], np.inf)
        for option in self.step_options(velocity):
            proposed = self.place_step(start, option, dt)
            cost = self.step_cost(start, proposed)
            take = cost < best_cost
            best = np.where(take[:, None], proposed, best)
            best_cost = np.minimum(best_cost, cost)
        return best

    def step_options(self, velocity: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Forward, then the two wall tangents of the same speed."""
        left = np.stack((-velocity[:, 1], velocity[:, 0]), axis=1)
        return velocity, left, -left

    def place_step(self, start: np.ndarray, velocity: np.ndarray, dt: float) -> np.ndarray:
        """Clamp one candidate and drop any part that enters an occupied cell."""
        proposed = start + velocity * dt
        # When Habitat navmesh owns free-space, do not Arena-clamp — the default
        # 10×10 box fights spread_spawn and freezes walkers off the tiny rectangle.
        if self.pathfinder is None and self.arena is not None:
            proposed = self.arena.clamp(proposed)
        if self.field is None:
            return proposed
        radius = float(self.settings["radius"])
        proposed = self.field.slide(start, proposed, radius)
        blocked = self.field.blocked(proposed, radius)
        proposed[blocked] = start[blocked]
        return proposed

    def step_navmesh_wander(
        self,
        dt: float,
        bodies: np.ndarray | None = None,
        body_radius: float = 0.0,
        robot: tuple[float, float, float] | np.ndarray | None = None,
    ) -> None:
        """Navmesh random walk with social avoidance (peer + optional robot).

        Steering = goal direction along ShortestPath, plus Helbing-style
        repulsion from other pedestrians (and robot when ``social``), then
        Habitat ``try_step`` so avoidance stays on the navigable surface.
        """
        n = int(self.position.shape[0])
        if n == 0 or self.pathfinder is None:
            return
        self._ensure_path_slots()
        try_step = getattr(self.pathfinder, "try_step", None)
        if not callable(try_step):
            try_step = getattr(self.pathfinder, "try_step_no_sliding", None)
        tol = float(self.settings.get("goal_tolerance", 0.45))
        stuck_lim = max(8, int(self.settings.get("stuck_steps", 18)))
        social_on = bool(self.settings.get("social", True))
        # Precompute pairwise social repulsion once per tick.
        social = (
            self.force.social(self.position, float(self.settings["radius"]))
            if social_on and n >= 2
            else np.zeros((n, 2), dtype=np.float64)
        )
        # Soft disk separation (handles hard overlaps SFM exponential misses).
        if social_on and n >= 2:
            social = social + self.separation()
        robot_xy, _robot_yaw = self.robot_state(robot)
        if social_on and robot_xy is not None:
            social = social + self.force.bodies(
                self.position,
                float(self.settings["radius"]),
                robot_xy.reshape(1, 2),
                float(self.settings.get("radius", 0.22)),
            )
        if bodies is not None and np.asarray(bodies).size:
            social = social + self.force.bodies(
                self.position,
                float(self.settings["radius"]),
                bodies,
                float(body_radius),
            )

        for i in range(n):
            if not self.path_waypoints[i]:
                self.assign_wander_goal(i)
            self.follow_nav_paths_one(i, tol)
            gx = float(self.goal[i, 0])
            gy = float(self.goal[i, 1])
            x = float(self.position[i, 0])
            y = float(self.position[i, 1])
            dx, dy = gx - x, gy - y
            dist = math.hypot(dx, dy)
            if dist <= tol or int(self.stuck[i]) >= stuck_lim:
                self.assign_wander_goal(i)
                self.stuck[i] = 0
                gx = float(self.goal[i, 0])
                gy = float(self.goal[i, 1])
                dx, dy = gx - x, gy - y
                dist = math.hypot(dx, dy)
            if dist < 1e-4:
                self.assign_wander_goal(i)
                continue
            speed = float(self.speed[i]) if i < self.speed.shape[0] else float(
                self.settings.get("speed", 0.8)
            )
            speed = max(speed, 0.35)
            # Preferred velocity toward waypoint + social avoidance.
            goal_vx, goal_vy = (dx / dist) * speed, (dy / dist) * speed
            gain = float(self.settings.get("social_force_factor", 1.0))
            if i < self.social_force_factor.shape[0]:
                gain *= float(self.social_force_factor[i])
            avoid_x = float(social[i, 0]) * gain
            avoid_y = float(social[i, 1]) * gain
            vx, vy = goal_vx + avoid_x, goal_vy + avoid_y
            vnorm = math.hypot(vx, vy)
            if vnorm < 1e-4:
                vx, vy = goal_vx, goal_vy
                vnorm = math.hypot(vx, vy)
            # Cap to preferred speed (avoidance can otherwise explode).
            scale = speed / max(vnorm, 1e-6)
            if vnorm > speed:
                vx, vy = vx * scale, vy * scale
            step = speed * dt
            tx = x + vx / max(math.hypot(vx, vy), 1e-6) * min(step, max(dist, step))
            ty = y + vy / max(math.hypot(vx, vy), 1e-6) * min(step, max(dist, step))
            moved = False
            if callable(try_step):
                try:
                    hit = try_step(
                        self._habitat_point(x, y), self._habitat_point(tx, ty)
                    )
                    if hit is not None:
                        hx, hy = float(hit[0]), -float(hit[2])
                        if np.isfinite(hx) and np.isfinite(hy):
                            self.position[i, 0] = hx
                            self.position[i, 1] = hy
                            if len(hit) >= 2 and np.isfinite(float(hit[1])):
                                self.nav_floor_y = float(hit[1])
                            moved = True
                except Exception:
                    moved = False
            if not moved:
                snapped = self.snap_one(tx, ty, allow_random=False)
                if snapped is not None:
                    self.position[i, 0], self.position[i, 1] = snapped
                    moved = True
            if not moved:
                self.stuck[i] += 1
            else:
                delta = math.hypot(
                    float(self.position[i, 0]) - x, float(self.position[i, 1]) - y
                )
                if delta > 1e-4:
                    self.stuck[i] = 0
                    self.yaw[i] = math.atan2(
                        float(self.position[i, 1]) - y,
                        float(self.position[i, 0]) - x,
                    )
                    self.velocity[i, 0] = (float(self.position[i, 0]) - x) / max(dt, 1e-6)
                    self.velocity[i, 1] = (float(self.position[i, 1]) - y) / max(dt, 1e-6)
                else:
                    self.stuck[i] += 1
                    if int(self.stuck[i]) >= stuck_lim:
                        near = self.random_navigable_near(x, y, radius=2.0)
                        if near is not None:
                            self.position[i, 0], self.position[i, 1] = near
                        self.assign_wander_goal(i)
                        self.stuck[i] = 0

        # Final soft de-penetration between peers, still on navmesh.
        if social_on and n >= 2:
            self.resolve_peer_overlaps(try_step)

    def follow_nav_paths_one(self, index: int, tol: float) -> None:
        """Advance agent ``index`` along its navmesh polyline."""
        self._ensure_path_slots()
        pts = self.path_waypoints[index]
        if not pts:
            return
        px = float(self.position[index, 0])
        py = float(self.position[index, 1])
        while pts and math.hypot(pts[0][0] - px, pts[0][1] - py) <= tol:
            pts.pop(0)
        if pts:
            self.goal[index] = pts[0]
        else:
            self.assign_wander_goal(index)

    def resolve_peer_overlaps(self, try_step: Any) -> None:
        """Push overlapping pedestrians apart along navmesh when possible."""
        n = int(self.position.shape[0])
        gap = 2.0 * float(self.settings["radius"]) + float(
            self.settings.get("social_margin", 0.08)
        )
        for i in range(n):
            for j in range(i + 1, n):
                dx = float(self.position[i, 0] - self.position[j, 0])
                dy = float(self.position[i, 1] - self.position[j, 1])
                dist = math.hypot(dx, dy)
                if dist >= gap or dist < 1e-6:
                    if dist < 1e-6:
                        dx, dy, dist = 1.0, 0.0, 1.0
                    else:
                        continue
                push = 0.5 * (gap - dist)
                ux, uy = dx / dist, dy / dist
                for index, sign in ((i, 1.0), (j, -1.0)):
                    x = float(self.position[index, 0])
                    y = float(self.position[index, 1])
                    tx = x + sign * ux * push
                    ty = y + sign * uy * push
                    placed = False
                    if callable(try_step):
                        try:
                            hit = try_step(
                                self._habitat_point(x, y),
                                self._habitat_point(tx, ty),
                            )
                            if hit is not None:
                                hx, hy = float(hit[0]), -float(hit[2])
                                if np.isfinite(hx) and np.isfinite(hy):
                                    self.position[index, 0] = hx
                                    self.position[index, 1] = hy
                                    placed = True
                        except Exception:
                            placed = False
                    if not placed:
                        snapped = self.snap_one(tx, ty, allow_random=False)
                        if snapped is not None:
                            self.position[index, 0], self.position[index, 1] = snapped

    def step_cost(self, start: np.ndarray, proposed: np.ndarray) -> np.ndarray:
        """Distance left to the goal. A step that does not move sorts last."""
        distance = np.linalg.norm(self.goal - proposed, axis=1)
        stayed = np.linalg.norm(proposed - start, axis=1) <= 1e-6
        return np.where(stayed, distance + 1.0e3, distance)

    def clear_bodies(
        self,
        proposed: np.ndarray,
        bodies: np.ndarray | None,
        body_radius: float,
    ) -> np.ndarray:
        """Push centers out to ``radius + body_radius`` from each other body."""
        if bodies is None:
            return proposed
        centers = np.asarray(bodies, dtype=np.float64).reshape(-1, 2)
        gap = float(self.settings["radius"]) + float(body_radius)
        if centers.shape[0] == 0 or gap <= 0.0:
            return proposed
        placed = np.array(proposed, dtype=np.float64, copy=True)
        delta = placed[:, None, :] - centers[None, :, :]
        distance = np.linalg.norm(delta, axis=-1)
        overlap = np.clip(gap - distance, 0.0, None)
        direction = delta / np.maximum(distance, 1e-6)[..., None]
        direction[distance < 1e-6] = np.array([1.0, 0.0])
        placed += (direction * overlap[..., None]).sum(axis=1)
        return self.arena.clamp(placed)

    def keep_free(self, start: np.ndarray, proposed: np.ndarray) -> np.ndarray:
        """Reject steps into occupied cells / off the Habitat navmesh."""
        placed = np.array(proposed, dtype=np.float64, copy=True)
        start_xy = np.asarray(start, dtype=np.float64).reshape(-1, 2)
        if self.field is not None:
            blocked = self.field.blocked(placed, float(self.settings["radius"]))
            placed[blocked] = start_xy[blocked]
        if self.pathfinder is not None:
            placed = self.constrain_navmesh(start_xy, placed)
        return placed

    def bind_pathfinder(self, pathfinder: Any, floor_y: float = 0.0) -> None:
        """Attach Habitat ``pathfinder`` and snap every walker/goal onto it."""
        self.pathfinder = pathfinder
        self.nav_floor_y = float(floor_y)
        if pathfinder is None or self.position.shape[0] == 0:
            return
        if not bool(getattr(pathfinder, "is_loaded", True)):
            self.pathfinder = None
            return
        self.expand_arena_to_navmesh()
        if bool(self.settings.get("spread_spawn", False)):
            self.spread_agents_on_navmesh()
        else:
            self.position = self.snap_points(self.position, allow_random=True)
        self.goal = self.snap_points(self.goal, allow_random=True)
        for index, waypoints in enumerate(self.goal_lists):
            if not waypoints:
                continue
            points = np.asarray(waypoints, dtype=np.float64).reshape(-1, 2)
            points = self.snap_points(points, allow_random=True)
            self.goal_lists[index] = [
                (float(x), float(y)) for x, y in points.reshape(-1, 2)
            ]
            gi = int(self.goal_index[index]) if index < self.goal_index.shape[0] else 0
            if self.goal_lists[index]:
                gi = min(max(gi, 0), len(self.goal_lists[index]) - 1)
                self.goal_index[index] = gi
                self.goal[index] = self.goal_lists[index][gi]
        self._ensure_path_slots()
        if bool(self.settings.get("wander", False)):
            for index in range(int(self.position.shape[0])):
                self.assign_wander_goal(index)
        logger.info(
            "pedestrian pathfinder bound: agents=%d floor_y=%.3f wander=%s spread=%s",
            int(self.position.shape[0]),
            self.nav_floor_y,
            bool(self.settings.get("wander", False)),
            bool(self.settings.get("spread_spawn", False)),
        )

    def snap_points(
        self, points: np.ndarray, *, allow_random: bool = False
    ) -> np.ndarray:
        """Project ``(N, 2)`` map poses onto navigable navmesh cells."""
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        if pts.shape[0] == 0 or self.pathfinder is None:
            return pts
        out = np.array(pts, dtype=np.float64, copy=True)
        for i in range(out.shape[0]):
            snapped = self.snap_one(float(out[i, 0]), float(out[i, 1]), allow_random=allow_random)
            if snapped is not None:
                out[i, 0], out[i, 1] = snapped
        return out

    def snap_one(
        self, x: float, y: float, *, allow_random: bool = False
    ) -> tuple[float, float] | None:
        """Snap one map ``(x, y)`` to Habitat navmesh; ``None`` if impossible."""
        if self.pathfinder is None:
            return (float(x), float(y))
        query = self._habitat_point(x, y)
        snap = getattr(self.pathfinder, "snap_point", None)
        navigable = getattr(self.pathfinder, "is_navigable", None)
        try:
            if callable(navigable) and bool(navigable(query)):
                return (float(x), float(y))
            if callable(snap):
                hit = snap(query)
                if hit is not None:
                    sx, sy = float(hit[0]), -float(hit[2])
                    if np.isfinite(sx) and np.isfinite(sy):
                        # Reject wild snaps into a distant island.
                        if math.hypot(sx - x, sy - y) <= 3.0 or allow_random:
                            if not callable(navigable) or bool(
                                navigable(self._habitat_point(sx, sy))
                            ):
                                return (sx, sy)
            if allow_random:
                return self.random_navigable()
        except Exception:
            if allow_random:
                return self.random_navigable()
        return None

    def expand_arena_to_navmesh(self) -> None:
        """Grow the planar Arena to Habitat navmesh bounds (map frame)."""
        if self.pathfinder is None:
            return
        get_bounds = getattr(self.pathfinder, "get_bounds", None)
        if not callable(get_bounds):
            return
        try:
            lo, hi = get_bounds()
        except Exception:
            return
        x0, x1 = float(lo[0]), float(hi[0])
        # map_y = -Habitat_Z
        y0, y1 = -float(hi[2]), -float(lo[2])
        if y0 > y1:
            y0, y1 = y1, y0
        pad = 1.0
        self.settings["x_length"] = max(abs(x1 - x0) + 2.0 * pad, 4.0)
        self.settings["y_length"] = max(abs(y1 - y0) + 2.0 * pad, 4.0)
        self.settings["center_x"] = 0.5 * (x0 + x1)
        self.settings["center_y"] = 0.5 * (y0 + y1)
        self.settings["spawn_x"] = float(self.settings["center_x"])
        self.settings["spawn_y"] = float(self.settings["center_y"])
        try:
            self.arena = self.make_arena()
        except ValueError:
            pass

    def random_navigable(self) -> tuple[float, float] | None:
        """Sample one Habitat navigable point in map frame."""
        random_nav = getattr(self.pathfinder, "get_random_navigable_point", None)
        if not callable(random_nav):
            return None
        try:
            for _ in range(16):
                point = random_nav()
                if point is None:
                    continue
                sx, sy = float(point[0]), -float(point[2])
                if np.isfinite(sx) and np.isfinite(sy):
                    self.nav_floor_y = float(point[1])
                    return (sx, sy)
        except Exception:
            return None
        return None

    def random_navigable_away(
        self,
        origin: tuple[float, float],
        *,
        min_dist: float,
        avoid: np.ndarray | None = None,
        trials: int = 48,
    ) -> tuple[float, float] | None:
        """Sample a navigable point far from ``origin`` (and optional peers)."""
        best: tuple[float, float] | None = None
        best_score = -1.0
        ox, oy = float(origin[0]), float(origin[1])
        peers = (
            np.asarray(avoid, dtype=np.float64).reshape(-1, 2)
            if avoid is not None
            else np.zeros((0, 2), dtype=np.float64)
        )
        for _ in range(max(8, int(trials))):
            point = self.random_navigable()
            if point is None:
                continue
            px, py = point
            dist = math.hypot(px - ox, py - oy)
            if dist < float(min_dist):
                continue
            peer = 0.0
            if peers.shape[0]:
                peer = float(
                    np.min(np.hypot(peers[:, 0] - px, peers[:, 1] - py))
                )
            # Prefer far from self and not clustered on another agent.
            score = dist + 0.75 * peer
            if score > best_score:
                best_score = score
                best = (px, py)
        return best if best is not None else self.random_navigable()

    def spread_agents_on_navmesh(self) -> None:
        """Re-seed agents onto well-separated navigable points."""
        count = int(self.position.shape[0])
        if count == 0 or self.pathfinder is None:
            return
        min_dist = float(self.settings.get("spread_min_dist", 2.0))
        placed: list[tuple[float, float]] = []
        for _ in range(count):
            avoid = np.asarray(placed, dtype=np.float64).reshape(-1, 2)
            origin = placed[-1] if placed else (0.0, 0.0)
            point = self.random_navigable_away(
                origin, min_dist=min_dist, avoid=avoid, trials=64
            )
            if point is None:
                point = self.random_navigable()
            if point is None:
                point = (float(self.position[len(placed), 0]), float(self.position[len(placed), 1]))
            placed.append(point)
        self.position = np.asarray(placed, dtype=np.float64).reshape(-1, 2)
        self.velocity = np.zeros_like(self.position)

    def assign_wander_goal(self, index: int) -> None:
        """Pick a reachable navmesh goal (ShortestPath) so agents keep moving."""
        if index < 0 or index >= int(self.position.shape[0]):
            return
        self._ensure_path_slots()
        ox = float(self.position[index, 0])
        oy = float(self.position[index, 1])
        min_dist = float(self.settings.get("wander_min_dist", 3.5))
        # After being stuck, accept closer goals so motion resumes quickly.
        if index < self.stuck.shape[0] and int(self.stuck[index]) >= int(
            self.settings.get("stuck_steps", 25)
        ):
            min_dist = max(1.2, 0.45 * min_dist)
        avoid = (
            np.delete(self.position, index, axis=0)
            if self.position.shape[0] > 1
            else None
        )
        chosen: tuple[float, float] | None = None
        path: list[tuple[float, float]] | None = None
        for _ in range(56):
            point = self.random_navigable_away(
                (ox, oy), min_dist=min_dist, avoid=avoid, trials=8
            )
            if point is None:
                point = self.random_navigable_near(ox, oy, radius=max(2.5, min_dist))
            if point is None:
                continue
            path = self.compute_nav_path(ox, oy, point[0], point[1])
            if path is not None and len(path) >= 2:
                chosen = point
                break
            path = None
        if chosen is None:
            near = self.random_navigable_near(ox, oy, radius=3.0)
            if near is not None:
                path = self.compute_nav_path(ox, oy, near[0], near[1])
                chosen = near if path is not None else near
        if chosen is None:
            sample = self.sample_goals(1)
            if sample.shape[0] == 0:
                return
            chosen = (float(sample[0, 0]), float(sample[0, 1]))
            path = self.compute_nav_path(ox, oy, chosen[0], chosen[1])
        self.goal[index] = chosen
        self.goal_lists[index] = [chosen]
        if index < self.goal_index.shape[0]:
            self.goal_index[index] = 0
        if path is not None and len(path) >= 2:
            self.path_waypoints[index] = path[1:]  # skip current pose
            self.goal[index] = self.path_waypoints[index][0]
        else:
            self.path_waypoints[index] = [chosen]

    def _ensure_path_slots(self) -> None:
        """Pad ``path_waypoints`` to the current agent count."""
        n = int(self.position.shape[0])
        while len(self.path_waypoints) < n:
            self.path_waypoints.append([])
        if len(self.path_waypoints) > n:
            self.path_waypoints = self.path_waypoints[:n]

    def follow_nav_paths(self) -> None:
        """Advance along ShortestPath vertices; keep ``goal`` = next waypoint."""
        if self.pathfinder is None:
            return
        self._ensure_path_slots()
        tol = float(self.settings.get("goal_tolerance", 0.4))
        for index in range(int(self.position.shape[0])):
            pts = self.path_waypoints[index]
            if not pts:
                continue
            px = float(self.position[index, 0])
            py = float(self.position[index, 1])
            while pts and math.hypot(pts[0][0] - px, pts[0][1] - py) <= tol:
                pts.pop(0)
            if pts:
                self.goal[index] = pts[0]
            elif bool(self.settings.get("wander", False)):
                self.assign_wander_goal(index)

    def compute_nav_path(
        self, x0: float, y0: float, x1: float, y1: float
    ) -> list[tuple[float, float]] | None:
        """Habitat ``ShortestPath`` in map frame, or None if disconnected."""
        if self.pathfinder is None:
            return None
        find_path = getattr(self.pathfinder, "find_path", None)
        if not callable(find_path):
            return None
        try:
            import habitat_sim

            path = habitat_sim.ShortestPath()
            path.requested_start = self._habitat_point(x0, y0)
            path.requested_end = self._habitat_point(x1, y1)
            if not bool(find_path(path)):
                return None
            points = []
            for hit in list(getattr(path, "points", []) or []):
                sx, sy = float(hit[0]), -float(hit[2])
                if np.isfinite(sx) and np.isfinite(sy):
                    points.append((sx, sy))
            return points if len(points) >= 2 else None
        except Exception:
            return None

    def random_navigable_near(
        self, x: float, y: float, *, radius: float = 3.0
    ) -> tuple[float, float] | None:
        """Sample a navigable point near ``(x, y)`` when the API exists."""
        near = getattr(self.pathfinder, "get_random_navigable_point_near", None)
        if callable(near):
            try:
                for _ in range(24):
                    point = near(self._habitat_point(x, y), float(radius))
                    if point is None:
                        continue
                    sx, sy = float(point[0]), -float(point[2])
                    if np.isfinite(sx) and np.isfinite(sy):
                        if len(point) >= 2 and np.isfinite(float(point[1])):
                            self.nav_floor_y = float(point[1])
                        return (sx, sy)
            except Exception:
                pass
        # Fallback: reject random samples that are too far.
        for _ in range(24):
            point = self.random_navigable()
            if point is None:
                continue
            if math.hypot(point[0] - x, point[1] - y) <= float(radius) * 1.5:
                return point
        return self.random_navigable()

    def constrain_navmesh(self, start: np.ndarray, proposed: np.ndarray) -> np.ndarray:
        """Keep each step on Habitat navmesh via ``try_step`` / snap."""
        placed = np.array(proposed, dtype=np.float64, copy=True)
        start_xy = np.asarray(start, dtype=np.float64).reshape(-1, 2)
        # Prefer sliding so agents glide along walls instead of freezing.
        try_step = getattr(self.pathfinder, "try_step", None)
        if not callable(try_step):
            try_step = getattr(self.pathfinder, "try_step_no_sliding", None)
        for i in range(placed.shape[0]):
            sx0, sy0 = float(start_xy[i, 0]), float(start_xy[i, 1])
            px, py = float(placed[i, 0]), float(placed[i, 1])
            start_h = self._habitat_point(sx0, sy0)
            end_h = self._habitat_point(px, py)
            moved = False
            if callable(try_step):
                try:
                    hit = try_step(start_h, end_h)
                    if hit is not None:
                        hx, hy = float(hit[0]), -float(hit[2])
                        if np.isfinite(hx) and np.isfinite(hy):
                            placed[i, 0], placed[i, 1] = hx, hy
                            if len(hit) >= 2 and np.isfinite(float(hit[1])):
                                self.nav_floor_y = float(hit[1])
                            moved = True
                except Exception:
                    moved = False
            if not moved:
                snapped = self.snap_one(px, py, allow_random=False)
                if snapped is None:
                    back = self.snap_one(sx0, sy0, allow_random=True)
                    placed[i, 0], placed[i, 1] = (
                        back if back is not None else (sx0, sy0)
                    )
                else:
                    # Only accept if still near the attempted step (no teleport).
                    if math.hypot(snapped[0] - sx0, snapped[1] - sy0) <= max(
                        0.75, math.hypot(px - sx0, py - sy0) + 0.25
                    ):
                        placed[i, 0], placed[i, 1] = snapped
                    else:
                        back = self.snap_one(sx0, sy0, allow_random=True)
                        placed[i, 0], placed[i, 1] = (
                            back if back is not None else (sx0, sy0)
                        )
        return placed

    def _habitat_point(self, x: float, y: float) -> list[float]:
        """Map ``(x, y)`` → Habitat ``[X, Y, Z]`` with ``map_y = -Z``."""
        return [float(x), float(self.nav_floor_y), -float(y)]

    def face(self, start: np.ndarray) -> None:
        """Aim yaw along the step that was actually taken."""
        delta = self.position - start
        moving = np.linalg.norm(delta, axis=1) > 1e-6
        self.yaw[moving] = np.arctan2(delta[moving, 1], delta[moving, 0])

    def note_stuck(self, start: np.ndarray) -> None:
        """Count consecutive steps that do not move the walker."""
        moved = np.linalg.norm(self.position - start, axis=1) > 1e-3
        self.stuck[moved] = 0
        self.stuck[~moved] += 1

    def sample_goals(self, count: int) -> np.ndarray:
        """Goals on Habitat navmesh when bound; else arena / occupancy free space."""
        if count == 0:
            return np.zeros((0, 2), dtype=np.float64)
        if self.pathfinder is not None:
            samples: list[tuple[float, float]] = []
            for _ in range(max(count * 4, count)):
                point = self.random_navigable()
                if point is None:
                    break
                samples.append(point)
                if len(samples) >= count:
                    return np.asarray(samples[:count], dtype=np.float64)
        if self.arena is None:
            return np.zeros((0, 2), dtype=np.float64)
        goals = self.arena.sample(self.rng, count)
        if self.field is None:
            return goals
        return self.field.relocate(
            self.arena, goals, float(self.settings["radius"]), self.rng
        )

    def refresh_goals(self) -> None:
        """Advance waypoints, or resample when stuck without a waypoint list."""
        if self.arena is None or self.position.shape[0] == 0:
            return
        if self.goal_radii.shape[0] == self.position.shape[0]:
            tolerance = self.goal_radii
        else:
            tolerance = float(self.settings["goal_tolerance"])
        reached = np.linalg.norm(self.goal - self.position, axis=1) <= tolerance
        stalled = self.stuck >= int(self.settings["stuck_steps"])
        for index in np.flatnonzero(reached | stalled):
            self.advance_goal(int(index), force=bool(stalled[index]))
            self.stuck[index] = 0

    def advance_goal(self, index: int, force: bool = False) -> None:
        """Next waypoint, wrap if cyclic, wander on navmesh, or sample free space."""
        if bool(self.settings.get("wander", False)) and self.pathfinder is not None:
            self.assign_wander_goal(index)
            return
        waypoints = self.goal_lists[index] if index < len(self.goal_lists) else []
        if len(waypoints) > 1:
            nxt = int(self.goal_index[index]) + 1
            if nxt >= len(waypoints):
                if self.cyclic[index]:
                    nxt = 0
                elif not force:
                    return
                else:
                    nxt = 0
            self.goal_index[index] = nxt
            self.goal[index] = waypoints[nxt]
            return
        sample = self.sample_goals(1)
        self.goal[index] = sample[0]
        self.goal_lists[index] = [(float(sample[0, 0]), float(sample[0, 1]))]

    def hit_radius(self) -> float:
        """Radius used for laser / map cloud (defaults to body radius)."""
        return float(self.settings.get("laser_radius", self.settings["radius"]))

    def bind_field(self, field: Field) -> None:
        """Slide along this grid, and move anyone who spawned on a wall."""
        self.field = field
        if self.arena is None or self.position.shape[0] == 0:
            return
        rng = np.random.default_rng(int(self.settings["seed"]) + 17)
        radius = float(self.settings["radius"])
        before = np.array(self.position, dtype=np.float64, copy=True)
        self.position = field.relocate(
            self.arena, self.position, radius, rng, tries=48
        )
        self.goal = field.relocate(self.arena, self.goal, radius, rng, tries=48)
        for index, waypoints in enumerate(self.goal_lists):
            if not waypoints:
                continue
            points = np.asarray(waypoints, dtype=np.float64).reshape(-1, 2)
            points = field.relocate(self.arena, points, radius, rng, tries=48)
            self.goal_lists[index] = [
                (float(x), float(y)) for x, y in points.reshape(-1, 2)
            ]
            goal_index = int(self.goal_index[index]) if index < self.goal_index.shape[0] else 0
            if 0 <= goal_index < len(self.goal_lists[index]):
                self.goal[index] = self.goal_lists[index][goal_index]
        moved = int(np.count_nonzero(np.linalg.norm(self.position - before, axis=1) > 1e-3))
        if moved:
            logger.info(
                "relocated %d / %d pedestrians off occupied cells",
                moved,
                int(self.position.shape[0]),
            )

    def make_arena(self) -> Arena:
        """Rectangle these pedestrians are allowed to walk in."""
        radius = float(self.settings["radius"])
        if radius <= 0.0:
            raise ValueError("scenario.pedestrian.radius must be > 0")
        center = (
            float(self.settings.get("center_x", 0.0)),
            float(self.settings.get("center_y", 0.0)),
        )
        spawn = (
            float(self.settings.get("spawn_x", center[0])),
            float(self.settings.get("spawn_y", center[1])),
        )
        return Arena(
            float(self.settings["x_length"]),
            float(self.settings["y_length"]),
            radius,
            float(self.settings["spawn_radius"]),
            "scenario map is smaller than a pedestrian",
            center=center,
            spawn_center=spawn,
        )
