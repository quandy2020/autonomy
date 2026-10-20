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

"""Pedestrian reactions to the robot.

Kinds: regular, impassive, surprised, scared, curious, threatening
(also accepted as integers 1–6).
"""

from __future__ import annotations

from typing import Any

import numpy as np


class Behavior:
    """Shape preferred velocities from a reaction profile."""

    KINDS = {
        1: "regular",
        2: "impassive",
        3: "surprised",
        4: "scared",
        5: "curious",
        6: "threatening",
        "1": "regular",
        "2": "impassive",
        "3": "surprised",
        "4": "scared",
        "5": "curious",
        "6": "threatening",
        "regular": "regular",
        "impassive": "impassive",
        "surprised": "surprised",
        "scared": "scared",
        "curious": "curious",
        "threatening": "threatening",
        # HuNavSim / hunav_msgs enum spellings
        "beh_regular": "regular",
        "beh_impassive": "impassive",
        "beh_surprised": "surprised",
        "beh_scared": "scared",
        "beh_curious": "curious",
        "beh_threatening": "threatening",
    }

    def __init__(
        self,
        notice: float = 5.0,
        approach: float = 0.55,
        scare: float = 2.5,
        block: float = 1.2,
    ) -> None:
        """Store reaction distances.

        Args:
            notice: Range where surprised / curious / scared activate (m).
            approach: Curious approach speed scale in ``(0, 1]``.
            scare: Distance under which scared agents flee (m).
            block: How far ahead of the robot threatening agents aim (m).
        """
        self.notice = float(notice)
        self.approach = float(approach)
        self.scare = float(scare)
        self.block = float(block)

    @classmethod
    def kind_name(cls, raw: Any) -> str:
        """Map an id or name to a reaction key.

        Raises:
            ValueError: Unknown behavior.
        """
        key: Any = raw.strip().lower() if isinstance(raw, str) else raw
        if isinstance(key, dict):
            key = key.get("type", "regular")
            if isinstance(key, str):
                key = key.strip().lower()
        try:
            return cls.KINDS[key]
        except KeyError as exc:
            raise ValueError(
                "pedestrian behavior must be regular, impassive, surprised, "
                "scared, curious, threatening, or 1–6"
            ) from exc

    def preferred(
        self,
        kinds: np.ndarray,
        position: np.ndarray,
        goal: np.ndarray,
        speed: np.ndarray,
        robot: np.ndarray | None,
        robot_yaw: float = 0.0,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Preferred velocities and a mask of agents that treat the robot as a body."""
        count = position.shape[0]
        preferred = np.zeros((count, 2), dtype=np.float64)
        impassive = np.zeros((count,), dtype=bool)
        if count == 0:
            return preferred, impassive
        delta_goal = goal - position
        dist_goal = np.linalg.norm(delta_goal, axis=1, keepdims=True)
        to_goal = np.divide(delta_goal, np.maximum(dist_goal, 1e-6))
        preferred = to_goal * np.asarray(speed, dtype=np.float64).reshape(-1, 1)
        if robot is None:
            return preferred, impassive
        robot_xy = np.asarray(robot, dtype=np.float64).reshape(2)
        to_robot = robot_xy - position
        dist = np.linalg.norm(to_robot, axis=1)
        near = dist <= self.notice
        for index in range(count):
            kind = str(kinds[index])
            if kind == "impassive":
                impassive[index] = True
                continue
            if not near[index]:
                continue
            if kind == "surprised":
                preferred[index] = 0.0
            elif kind == "curious":
                direction = to_robot[index] / max(dist[index], 1e-6)
                preferred[index] = direction * float(speed[index]) * self.approach
            elif kind == "scared" and dist[index] <= self.scare:
                direction = -to_robot[index] / max(dist[index], 1e-6)
                preferred[index] = direction * float(speed[index])
            elif kind == "threatening":
                ahead = robot_xy + self.block * np.array(
                    [np.cos(robot_yaw), np.sin(robot_yaw)]
                )
                offset = ahead - position[index]
                preferred[index] = (
                    offset / max(float(np.linalg.norm(offset)), 1e-6)
                ) * float(speed[index])
        return preferred, impassive
