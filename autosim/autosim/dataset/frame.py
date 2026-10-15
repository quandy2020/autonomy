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

"""Single imitation-learning frame (observation + action)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np


@dataclass
class Frame:
    """One timestep for LeRobot export."""

    timestamp: float
    state: np.ndarray  # float32 [5] = x,y,yaw,vx,wz
    action: np.ndarray  # float32 [2] = vx,wz (command)
    rgb: Optional[np.ndarray] = None  # HxWx3 uint8
    laser: Optional[np.ndarray] = None  # float32 [N]
    done: bool = False
    meta: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.state = np.asarray(self.state, dtype=np.float32).reshape(-1)
        self.action = np.asarray(self.action, dtype=np.float32).reshape(-1)
        if self.rgb is not None:
            self.rgb = np.asarray(self.rgb, dtype=np.uint8)
        if self.laser is not None:
            self.laser = np.asarray(self.laser, dtype=np.float32).reshape(-1)
