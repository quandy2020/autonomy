from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass
class AgentState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def as_tuple(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.yaw
