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

"""Encode / decode ``automsgs.msgs.hunav_msgs`` Agents ↔ Pedestrian state."""

from __future__ import annotations

import math
from typing import Any

from autosim.scene.agents.behavior import Behavior

# Mirror hunav_msgs enum values (also in agent_behavior.proto).
BEH_NAME_TO_TYPE = {
    "regular": 1,
    "impassive": 2,
    "surprised": 3,
    "scared": 4,
    "curious": 5,
    "threatening": 6,
}
BEH_TYPE_TO_NAME = {v: k for k, v in BEH_NAME_TO_TYPE.items()}

# HuNav far-obstacle sentinel when no ray hit (see Hunav_isaac_wrapper).
OBS_FAR = 10000.0


def _try_import_pb2() -> tuple[Any, Any, Any] | None:
    try:
        from automsgs.msgs.hunav_msgs import agent_pb2, agents_pb2, agent_behavior_pb2

        return agent_pb2, agents_pb2, agent_behavior_pb2
    except Exception:
        return None


class AgentCodec:
    """Bidirectional conversion between protobuf Agents and Pedestrian arrays."""

    PERSON = 1
    ROBOT = 2

    def __init__(self) -> None:
        self._pb = _try_import_pb2()

    @property
    def available(self) -> bool:
        """True when generated ``automsgs`` hunav_msgs Python stubs exist."""
        return self._pb is not None

    def yaw_to_quat(self, yaw: float) -> tuple[float, float, float, float]:
        """Planar yaw → quaternion ``(x, y, z, w)``."""
        half = 0.5 * float(yaw)
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def quat_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        """Extract Z-up yaw from quaternion."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle: float) -> float:
        """Wrap to ``(-pi, pi]`` (HuNav helper)."""
        value = float(angle)
        while value <= -math.pi:
            value += 2.0 * math.pi
        while value > math.pi:
            value -= 2.0 * math.pi
        return value

    def pack_agents(
        self,
        pedestrian: Any,
        *,
        stamp: tuple[int, int] = (0, 0),
        frame_id: str = "map",
    ) -> Any:
        """Build an ``Agents`` protobuf from a :class:`Pedestrian`."""
        if self._pb is None:
            raise ImportError(
                "automsgs hunav_msgs not generated; rebuild automsgs "
                "(Agent / Agents / AgentBehavior)"
            )
        agent_pb2, agents_pb2, agent_behavior_pb2 = self._pb
        msg = agents_pb2.Agents()
        msg.header.stamp.sec = int(stamp[0])
        msg.header.stamp.nanosec = int(stamp[1])
        msg.header.frame_id = str(frame_id)
        n = int(pedestrian.position.shape[0])
        ids = getattr(pedestrian, "agent_ids", None)
        names = getattr(pedestrian, "agent_names", None)
        group_ids = getattr(pedestrian, "group_ids", None)
        skins = getattr(pedestrian, "skins", None)
        radii = getattr(pedestrian, "radii", None)
        goal_radii = getattr(pedestrian, "goal_radii", None)
        beh_config = getattr(pedestrian, "beh_config", None)
        for i in range(n):
            agent = msg.agents.add()
            if ids is not None and i < len(ids):
                agent.id = int(ids[i])
            else:
                agent.id = int(i + 1)
            agent.type = agent_pb2.Agent.PERSON
            if names is not None and i < len(names):
                agent.name = str(names[i])
            else:
                agent.name = f"agent{i + 1}"
            if group_ids is not None and i < len(group_ids):
                agent.group_id = int(group_ids[i])
            else:
                agent.group_id = -1
            if skins is not None and i < len(skins):
                agent.skin = int(skins[i])
            if radii is not None and i < len(radii):
                agent.radius = float(radii[i])
            else:
                agent.radius = float(pedestrian.settings.get("radius", 0.2))
            agent.desired_velocity = float(pedestrian.speed[i])
            x = float(pedestrian.position[i, 0])
            y = float(pedestrian.position[i, 1])
            yaw = self.normalize_angle(float(pedestrian.yaw[i]))
            qx, qy, qz, qw = self.yaw_to_quat(yaw)
            agent.position.position.x = x
            agent.position.position.y = y
            agent.position.position.z = 0.0
            agent.position.orientation.x = qx
            agent.position.orientation.y = qy
            agent.position.orientation.z = qz
            agent.position.orientation.w = qw
            agent.yaw = yaw
            vx = float(pedestrian.velocity[i, 0])
            vy = float(pedestrian.velocity[i, 1])
            agent.velocity.linear.x = vx
            agent.velocity.linear.y = vy
            agent.linear_vel = float(math.hypot(vx, vy))
            agent.angular_vel = 0.0
            kind = str(pedestrian.kinds[i])
            beh = agent.behavior
            beh.type = BEH_NAME_TO_TYPE.get(kind, 1)
            beh.state = agent_behavior_pb2.AgentBehavior.BEH_ACTIVE_1
            cfg = (
                int(beh_config[i])
                if beh_config is not None and i < len(beh_config)
                else 0
            )
            if cfg == 0:
                beh.configuration = agent_behavior_pb2.AgentBehavior.BEH_CONF_DEFAULT
            elif cfg == 1:
                beh.configuration = agent_behavior_pb2.AgentBehavior.BEH_CONF_CUSTOM
            elif cfg == 2:
                beh.configuration = (
                    agent_behavior_pb2.AgentBehavior.BEH_CONF_RANDOM_NORMAL
                )
            else:
                beh.configuration = (
                    agent_behavior_pb2.AgentBehavior.BEH_CONF_RANDOM_UNIFORM
                )
            gff = getattr(pedestrian, "goal_force_factor", None)
            sff = getattr(pedestrian, "social_force_factor", None)
            off = getattr(pedestrian, "obstacle_force_factor", None)
            oth = getattr(pedestrian, "other_force_factor", None)
            beh.goal_force_factor = float(
                gff[i]
                if gff is not None and i < len(gff)
                else pedestrian.settings.get("goal_force_factor", 1.0)
            )
            beh.social_force_factor = float(
                sff[i]
                if sff is not None and i < len(sff)
                else pedestrian.settings.get("social_force_factor", 1.0)
            )
            beh.obstacle_force_factor = float(
                off[i]
                if off is not None and i < len(off)
                else pedestrian.settings.get("obstacle_force_factor", 1.0)
            )
            beh.other_force_factor = float(
                oth[i] if oth is not None and i < len(oth) else 1.0
            )
            agent.cyclic_goals = (
                bool(pedestrian.cyclic[i]) if i < len(pedestrian.cyclic) else True
            )
            if goal_radii is not None and i < len(goal_radii):
                agent.goal_radius = float(goal_radii[i])
            else:
                agent.goal_radius = float(
                    pedestrian.settings.get("goal_tolerance", 0.4)
                )
            waypoints = []
            goal_lists = getattr(pedestrian, "goal_lists", None) or []
            if i < len(goal_lists) and goal_lists[i]:
                waypoints = list(goal_lists[i])
            else:
                waypoints = [
                    (float(pedestrian.goal[i, 0]), float(pedestrian.goal[i, 1]))
                ]
            # Rotate so current goal is first (HuNav consumers expect that).
            gi = int(pedestrian.goal_index[i]) if i < len(pedestrian.goal_index) else 0
            if waypoints and 0 <= gi < len(waypoints):
                waypoints = waypoints[gi:] + waypoints[:gi]
            for gx, gy in waypoints:
                g = agent.goals.add()
                g.position.x = float(gx)
                g.position.y = float(gy)
                g.position.z = 0.0
                g.orientation.w = 1.0
            # Empty closest_obs → consumers treat as "no sample"; far sentinel
            # would be filled by Isaac PhysX raycasts in the wrapper path.
        return msg

    def pack_robot(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        vx: float = 0.0,
        vy: float = 0.0,
        wz: float = 0.0,
        radius: float = 0.3,
        name: str = "robot",
        skin: int = 1,
    ) -> Any:
        """Build a robot ``Agent`` protobuf (HuNav ``Agent.ROBOT``)."""
        if self._pb is None:
            raise ImportError("automsgs hunav_msgs not generated")
        agent_pb2, _, _ = self._pb
        robot = agent_pb2.Agent()
        robot.id = 0
        robot.type = agent_pb2.Agent.ROBOT
        robot.skin = int(skin)
        robot.name = name
        robot.group_id = 0
        robot.radius = float(radius)
        robot.desired_velocity = max(float(math.hypot(vx, vy)), 1.0)
        yaw = self.normalize_angle(float(yaw))
        qx, qy, qz, qw = self.yaw_to_quat(yaw)
        robot.position.position.x = float(x)
        robot.position.position.y = float(y)
        robot.position.orientation.x = qx
        robot.position.orientation.y = qy
        robot.position.orientation.z = qz
        robot.position.orientation.w = qw
        robot.yaw = yaw
        robot.velocity.linear.x = float(vx)
        robot.velocity.linear.y = float(vy)
        robot.velocity.angular.z = float(wz)
        robot.linear_vel = float(math.hypot(vx, vy))
        robot.angular_vel = float(wz)
        robot.cyclic_goals = True
        robot.goal_radius = 0.5
        return robot

    def apply_agents(self, pedestrian: Any, agents_msg: Any) -> None:
        """Write updated ``Agents`` poses back into a :class:`Pedestrian`."""
        rows = list(agents_msg.agents)
        if not rows:
            return
        by_id = {int(a.id): a for a in rows}
        n = int(pedestrian.position.shape[0])
        ids = getattr(pedestrian, "agent_ids", None)
        for i in range(n):
            want = int(ids[i]) if ids is not None and i < len(ids) else (i + 1)
            agent = by_id.get(want) or (rows[i] if i < len(rows) else None)
            if agent is None:
                continue
            pedestrian.position[i, 0] = float(agent.position.position.x)
            pedestrian.position[i, 1] = float(agent.position.position.y)
            pedestrian.yaw[i] = self.normalize_angle(float(agent.yaw))
            pedestrian.velocity[i, 0] = float(agent.velocity.linear.x)
            pedestrian.velocity[i, 1] = float(agent.velocity.linear.y)
            if agent.behavior and agent.behavior.type:
                name = BEH_TYPE_TO_NAME.get(int(agent.behavior.type))
                if name:
                    pedestrian.kinds[i] = name
            gff = getattr(pedestrian, "goal_force_factor", None)
            if gff is not None and i < len(gff) and agent.behavior:
                if agent.behavior.goal_force_factor:
                    gff[i] = float(agent.behavior.goal_force_factor)
                if agent.behavior.social_force_factor:
                    pedestrian.social_force_factor[i] = float(
                        agent.behavior.social_force_factor
                    )
                if agent.behavior.obstacle_force_factor:
                    pedestrian.obstacle_force_factor[i] = float(
                        agent.behavior.obstacle_force_factor
                    )
                if agent.behavior.other_force_factor:
                    pedestrian.other_force_factor[i] = float(
                        agent.behavior.other_force_factor
                    )

    def robot_xy_yaw(self, robot_msg: Any) -> tuple[float, float, float]:
        """Extract planar robot pose from an ``Agent`` message."""
        return (
            float(robot_msg.position.position.x),
            float(robot_msg.position.position.y),
            float(robot_msg.yaw),
        )

    def behavior_name(self, raw: Any) -> str:
        """Map protobuf / int / string behavior to Pedestrian kind name."""
        if isinstance(raw, str):
            return Behavior.kind_name(raw)
        return BEH_TYPE_TO_NAME.get(int(raw), "regular")

    def agents_snapshot(self, agents_msg: Any) -> list[dict[str, Any]]:
        """Lightweight dict view for logging / tests (no protobuf required)."""
        out = []
        for agent in agents_msg.agents:
            out.append(
                {
                    "id": int(agent.id),
                    "name": str(agent.name),
                    "skin": int(agent.skin),
                    "group_id": int(agent.group_id),
                    "x": float(agent.position.position.x),
                    "y": float(agent.position.position.y),
                    "yaw": float(agent.yaw),
                    "vx": float(agent.velocity.linear.x),
                    "vy": float(agent.velocity.linear.y),
                    "behavior": self.behavior_name(
                        int(agent.behavior.type) if agent.behavior else 1
                    ),
                }
            )
        return out
