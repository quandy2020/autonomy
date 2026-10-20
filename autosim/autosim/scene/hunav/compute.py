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

"""HuNav ``/compute_agents`` body: one SFM step over an Agents snapshot."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from autosim.scene.agents.pedestrian import Pedestrian
from autosim.scene.hunav.codec import AgentCodec


class ComputeAgentsEngine:
    """Stateful SFM engine exposed as HuNav ComputeAgents.

    Holds a :class:`Pedestrian` that is overwritten from each request, stepped,
    then packed back into an ``Agents`` response — the same contract as
    HuNavSim's ``/compute_agents`` service, but implemented with autosim Force.
    """

    def __init__(
        self,
        settings: Mapping[str, Any] | None = None,
        *,
        default_dt: float = 0.02,
    ) -> None:
        cfg = dict(Pedestrian.DEFAULTS)
        cfg.update(
            {
                "enabled": True,
                "social": True,
                "count": 0,
            }
        )
        if settings:
            cfg.update(dict(settings))
        self.settings = cfg
        self.default_dt = float(default_dt)
        self.codec = AgentCodec()
        self.pedestrian = Pedestrian({**cfg, "enabled": False})
        self.pedestrian.settings.update(cfg)
        self.pedestrian.arena = self.pedestrian.make_arena()

    def ensure_size(self, count: int) -> None:
        """Grow / shrink internal Pedestrian buffers to ``count`` agents."""
        n = max(0, int(count))
        ped = self.pedestrian
        if ped.position.shape[0] == n:
            return
        ped.position = np.zeros((n, 2), dtype=np.float64)
        ped.velocity = np.zeros((n, 2), dtype=np.float64)
        ped.goal = np.zeros((n, 2), dtype=np.float64)
        ped.yaw = np.zeros((n,), dtype=np.float64)
        ped.speed = np.full((n,), float(ped.settings.get("speed", 1.2)), dtype=np.float64)
        ped.stuck = np.zeros((n,), dtype=np.int32)
        ped.kinds = np.array(["regular"] * n, dtype=object)
        ped.agent_ids = np.arange(1, n + 1, dtype=np.int32)
        ped.agent_names = np.array([f"agent{i + 1}" for i in range(n)], dtype=object)
        ped.group_ids = np.full((n,), -1, dtype=np.int32)
        ped.skins = np.zeros((n,), dtype=np.int32)
        ped.radii = np.full(
            (n,), float(ped.settings.get("radius", 0.2)), dtype=np.float64
        )
        ped.goal_radii = np.full(
            (n,), float(ped.settings.get("goal_tolerance", 0.4)), dtype=np.float64
        )
        ped.beh_config = np.zeros((n,), dtype=np.int32)
        ped.goal_force_factor = np.full(
            (n,), float(ped.settings.get("goal_force_factor", 1.0)), dtype=np.float64
        )
        ped.social_force_factor = np.full(
            (n,), float(ped.settings.get("social_force_factor", 1.0)), dtype=np.float64
        )
        ped.obstacle_force_factor = np.full(
            (n,), float(ped.settings.get("obstacle_force_factor", 1.0)), dtype=np.float64
        )
        ped.other_force_factor = np.ones((n,), dtype=np.float64)
        ped.goal_lists = [[(0.0, 0.0)] for _ in range(n)]
        ped.goal_index = np.zeros((n,), dtype=np.int32)
        ped.cyclic = np.ones((n,), dtype=bool)
        ped.settings["count"] = n
        ped.settings["social"] = True

    def ingest(self, agents_msg: Any) -> None:
        """Load request Agents into the Pedestrian plant."""
        rows = list(agents_msg.agents)
        self.ensure_size(len(rows))
        ped = self.pedestrian
        for i, agent in enumerate(rows):
            ped.agent_ids[i] = int(agent.id) if agent.id else (i + 1)
            ped.agent_names[i] = str(agent.name or f"agent{i + 1}")
            ped.group_ids[i] = int(agent.group_id) if agent.group_id else -1
            ped.skins[i] = int(agent.skin)
            if agent.radius:
                ped.radii[i] = float(agent.radius)
            if agent.goal_radius:
                ped.goal_radii[i] = float(agent.goal_radius)
            ped.position[i, 0] = float(agent.position.position.x)
            ped.position[i, 1] = float(agent.position.position.y)
            ped.yaw[i] = float(agent.yaw)
            ped.velocity[i, 0] = float(agent.velocity.linear.x)
            ped.velocity[i, 1] = float(agent.velocity.linear.y)
            ped.speed[i] = float(agent.desired_velocity or ped.settings.get("speed", 1.2))
            ped.cyclic[i] = bool(agent.cyclic_goals)
            if agent.behavior and agent.behavior.type:
                ped.kinds[i] = self.codec.behavior_name(int(agent.behavior.type))
                ped.beh_config[i] = int(agent.behavior.configuration or 0)
                if agent.behavior.goal_force_factor:
                    ped.goal_force_factor[i] = float(agent.behavior.goal_force_factor)
                if agent.behavior.social_force_factor:
                    ped.social_force_factor[i] = float(agent.behavior.social_force_factor)
                if agent.behavior.obstacle_force_factor:
                    ped.obstacle_force_factor[i] = float(
                        agent.behavior.obstacle_force_factor
                    )
                if agent.behavior.other_force_factor:
                    ped.other_force_factor[i] = float(agent.behavior.other_force_factor)
            if agent.goals:
                goals = [
                    (float(g.position.x), float(g.position.y)) for g in agent.goals
                ]
                ped.goal_lists[i] = goals
                ped.goal_index[i] = 0
                ped.goal[i, 0] = goals[0][0]
                ped.goal[i, 1] = goals[0][1]
            else:
                ped.goal[i] = ped.position[i]

    def step(
        self,
        agents_msg: Any,
        robot_msg: Any,
        *,
        dt: float | None = None,
        stamp: tuple[int, int] | None = None,
        frame_id: str = "map",
    ) -> Any:
        """Run one ComputeAgents cycle; return updated ``Agents`` protobuf."""
        if not self.codec.available:
            raise ImportError("automsgs hunav_msgs Python stubs missing; rebuild automsgs")
        self.ingest(agents_msg)
        rx, ry, ryaw = self.codec.robot_xy_yaw(robot_msg)
        step_dt = float(dt) if dt and float(dt) > 0.0 else self.default_dt
        self.pedestrian.step(step_dt, bodies=None, body_radius=0.0, robot=(rx, ry, ryaw))
        header = getattr(agents_msg, "header", None)
        if stamp is None and header is not None and header.stamp.sec:
            stamp = (int(header.stamp.sec), int(header.stamp.nanosec))
        if stamp is None:
            stamp = (0, 0)
        fid = frame_id
        if header is not None and header.frame_id:
            fid = str(header.frame_id)
        return self.codec.pack_agents(self.pedestrian, stamp=stamp, frame_id=fid)

    def step_one(
        self,
        agents_msg: Any,
        robot_msg: Any,
        agent_id: int,
        *,
        dt: float | None = None,
    ) -> Any:
        """ComputeAgents then return the agent with ``agent_id``."""
        updated = self.step(agents_msg, robot_msg, dt=dt)
        for agent in updated.agents:
            if int(agent.id) == int(agent_id):
                return agent
        raise KeyError(f"agent id {agent_id} not in updated Agents")
