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

"""Prediction result object."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import List

import numpy as np

from vntpa.simulator.action import Action, ActionPrimitive


@dataclass
class VNTResult:
    """One prediction result."""

    logits: np.ndarray
    actions: List[Action]
    primitives: List[ActionPrimitive]
    memory_size: int = 0

    def dump(self) -> dict:
        return {
            "actions": [int(action) for action in self.actions],
            "primitives": [
                {"action": int(item.action), "distance": item.distance, "angle": item.angle}
                for item in self.primitives
            ],
            "memory_size": self.memory_size,
        }

    def serialize(self, indent: int = 2) -> str:
        return json.dumps(self.dump(), indent=indent)

    def save(self, path: str) -> None:
        from pathlib import Path

        Path(path).write_text(self.serialize(), encoding="utf-8")

    def summary(self) -> str:
        return f"actions={[action.name for action in self.actions]} memory={self.memory_size}"
