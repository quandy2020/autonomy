# Copyright 2026 The Navigation Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""VNT-PA discrete actions and primitive-to-velocity conversion."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

import numpy as np


class Action(IntEnum):
    STOP = 0
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3


@dataclass(frozen=True)
class ActionPrimitive:
    action: Action
    distance: float = 0.0
    angle: float = 0.0


@dataclass(frozen=True)
class Twist:
    linear_x: float
    linear_y: float
    angular_z: float


def decode(logits: np.ndarray, chunk: int, action_count: int = 4) -> list[Action]:
    """(chunk, action_count) or (1, chunk, action_count) logits -> actions."""
    logits = np.asarray(logits, dtype=np.float32)
    if logits.ndim == 3:
        logits = logits[0]
    if logits.shape[0] < chunk or logits.shape[1] < action_count:
        raise ValueError(f"logits shape {logits.shape} < ({chunk}, {action_count})")
    return [Action(int(np.argmax(logits[i, :action_count]))) for i in range(chunk)]


def primitive(
    action: Action, forward_step: float = 0.25, turn_step_rad: float = 0.5235987756
) -> ActionPrimitive:
    if action == Action.FORWARD:
        return ActionPrimitive(action, distance=forward_step)
    if action in (Action.TURN_LEFT, Action.TURN_RIGHT):
        angle = turn_step_rad if action == Action.TURN_LEFT else -turn_step_rad
        return ActionPrimitive(action, angle=angle)
    return ActionPrimitive(action)


def twist(item: ActionPrimitive, dt_s: float = 0.5) -> Twist:
    """Map a discrete step to a velocity held for ``dt_s``."""
    if dt_s <= 0:
        raise ValueError("dt_s must be positive")
    if item.action == Action.FORWARD:
        return Twist(item.distance / dt_s, 0.0, 0.0)
    if item.action in (Action.TURN_LEFT, Action.TURN_RIGHT):
        return Twist(0.0, 0.0, item.angle / dt_s)
    return Twist(0.0, 0.0, 0.0)
