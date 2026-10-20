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
        proposed = self.arena.clamp(start + velocity * dt)
        if self.field is None:
            return proposed
        radius = float(self.settings["radius"])
        proposed = self.field.slide(start, proposed, radius)
        blocked = self.field.blocked(proposed, radius)
        proposed[blocked] = start[blocked]
        return proposed

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
        """Reject a clearance step that lands back inside an occupied cell."""
        if self.field is None:
            return proposed
        placed = np.array(proposed, dtype=np.float64, copy=True)
        blocked = self.field.blocked(placed, float(self.settings["radius"]))
        placed[blocked] = start[blocked]
        return placed

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
        """Goals inside the rectangle and, when a map exists, off the walls."""
        if self.arena is None or count == 0:
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
        """Next waypoint, wrap if cyclic, or sample a free-space goal."""
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
