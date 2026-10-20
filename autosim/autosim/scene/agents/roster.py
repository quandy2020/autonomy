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

"""Load pedestrian agent YAML for autosim (no ROS).

Native layout::

    agents:
      - pose: {x: 0, y: 0, yaw: 0}
        speed: 1.0
        behavior: regular
        goals: [{x: 1, y: 0}, {x: 0, y: 1}]
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from autosim.scene.agents.behavior import Behavior


class Agents:
    """Parsed per-agent pose, goals, speeds, and reaction profiles."""

    def __init__(self, records: list[dict[str, Any]]) -> None:
        """Store normalized agent records.

        Args:
            records: Output of :meth:`normalize`.
        """
        self.records = list(records)

    @classmethod
    def load(cls, path: str | Path) -> "Agents":
        """Load agents from a YAML file.

        Raises:
            FileNotFoundError: Path missing.
            ValueError: File has no agents.
        """
        file = Path(path).expanduser()
        if not file.is_file():
            raise FileNotFoundError(f"scenario pedestrian agents file missing: {file}")
        with file.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        return cls.from_mapping(data)

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any] | list[Any]) -> "Agents":
        """Parse a mapping or list into agent records."""
        if isinstance(data, list):
            raw_agents = data
        else:
            raw_agents = list(data.get("agents") or data.get("people") or [])
            if raw_agents and all(isinstance(name, str) for name in raw_agents):
                named = []
                for name in raw_agents:
                    entry = dict(data.get(name) or {})
                    entry.setdefault("name", name)
                    named.append(entry)
                raw_agents = named
        records = [cls.normalize(item, index) for index, item in enumerate(raw_agents)]
        if not records:
            raise ValueError("scenario pedestrian agents list is empty")
        return cls(records)

    @classmethod
    def normalize(cls, raw: Any, index: int) -> dict[str, Any]:
        """One agent dict → spawn fields used by :class:`Pedestrian`."""
        if not isinstance(raw, Mapping):
            raise ValueError(f"agent[{index}] must be a mapping")
        pose = raw.get("pose") or raw.get("init_pose") or {}
        if not isinstance(pose, Mapping):
            raise ValueError(f"agent[{index}].pose must be a mapping")
        behavior_raw = raw.get("behavior", "regular")
        if isinstance(behavior_raw, Mapping):
            kind = Behavior.kind_name(behavior_raw.get("type", "regular"))
            force = {
                key: float(behavior_raw[key])
                for key in (
                    "goal_force_factor",
                    "obstacle_force_factor",
                    "social_force_factor",
                )
                if key in behavior_raw
            }
            speed_override = behavior_raw.get("vel")
        else:
            kind = Behavior.kind_name(behavior_raw)
            force = {}
            speed_override = None
        goals = cls.goal_list(raw, pose)
        speed = float(
            speed_override
            if speed_override is not None
            else raw.get("speed", raw.get("max_vel", 1.2))
        )
        return {
            "id": int(raw.get("id", index + 1)),
            "name": str(raw.get("name") or f"agent{index + 1}"),
            "x": float(pose.get("x", 0.0)),
            "y": float(pose.get("y", 0.0)),
            "yaw": float(pose.get("yaw", pose.get("h", 0.0))),
            "radius": float(raw.get("radius", 0.2)),
            "height": float(raw.get("height", 1.7)),
            "speed": speed,
            "behavior": kind,
            "goal_tolerance": float(
                raw.get("goal_tolerance", raw.get("goal_radius", 0.4))
            ),
            "cyclic_goals": bool(raw.get("cyclic_goals", True)),
            "goals": goals,
            "force": force,
        }

    @classmethod
    def goal_list(
        cls, raw: Mapping[str, Any], pose: Mapping[str, Any]
    ) -> list[tuple[float, float]]:
        """Read ``[{x,y}, …]``, ``[[x,y], …]``, or named ``g0`` entries."""
        names = raw.get("goals") or []
        points: list[tuple[float, float]] = []
        if names and isinstance(names[0], (list, tuple)):
            for item in names:
                points.append((float(item[0]), float(item[1])))
            return points
        for name in names:
            if isinstance(name, Mapping):
                points.append((float(name["x"]), float(name["y"])))
                continue
            entry = raw.get(name)
            if not isinstance(entry, Mapping):
                raise ValueError(f"agent goal {name!r} is missing")
            points.append((float(entry["x"]), float(entry["y"])))
        if not points:
            points.append((float(pose.get("x", 0.0)), float(pose.get("y", 0.0))))
        return points

    @property
    def count(self) -> int:
        """Number of agents."""
        return len(self.records)

    def positions(self) -> np.ndarray:
        """``(N, 2)`` initial xy."""
        return np.array([[row["x"], row["y"]] for row in self.records], dtype=np.float64)

    def yaws(self) -> np.ndarray:
        """``(N,)`` initial headings."""
        return np.array([row["yaw"] for row in self.records], dtype=np.float64)

    def speeds(self) -> np.ndarray:
        """``(N,)`` preferred speeds."""
        return np.array([row["speed"] for row in self.records], dtype=np.float64)

    def behaviors(self) -> np.ndarray:
        """Object array of behavior kind strings."""
        return np.array([row["behavior"] for row in self.records], dtype=object)

    def first_goals(self) -> np.ndarray:
        """``(N, 2)`` first waypoint of each agent."""
        return np.array([row["goals"][0] for row in self.records], dtype=np.float64)

    def goal_lists(self) -> list[list[tuple[float, float]]]:
        """Per-agent waypoint lists."""
        return [list(row["goals"]) for row in self.records]
