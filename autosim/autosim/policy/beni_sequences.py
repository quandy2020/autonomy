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

"""Named Beni teleop maneuvers: ``(label, vx, wz, duration_s)``."""

from __future__ import annotations

from typing import Dict, List, Tuple

# (label, linear_x, angular_z, duration_seconds)
SequenceStep = Tuple[str, float, float, float]

BENI_SEQUENCES: Dict[str, List[SequenceStep]] = {
    "idle": [("hold_zero", 0.0, 0.0, 0.20)],
    "forward": [("fwd", 0.6, 0.0, 0.50), ("stop", 0.0, 0.0, 0.10)],
    "reverse": [("rev", -0.5, 0.0, 0.40), ("stop", 0.0, 0.0, 0.10)],
    "turn_left": [("yaw+", 0.0, 1.2, 0.40), ("stop", 0.0, 0.0, 0.10)],
    "turn_right": [("yaw-", 0.0, -1.2, 0.40), ("stop", 0.0, 0.0, 0.10)],
    "arc_left": [("arc", 0.5, 0.8, 0.50), ("stop", 0.0, 0.0, 0.10)],
    "arc_right": [("arc", 0.5, -0.8, 0.50), ("stop", 0.0, 0.0, 0.10)],
    "stop_go_stop": [
        ("fwd", 0.8, 0.0, 0.30),
        ("stop", 0.0, 0.0, 0.15),
        ("rev", -0.6, 0.0, 0.30),
        ("stop", 0.0, 0.0, 0.15),
        ("spin", 0.0, 1.5, 0.25),
        ("stop", 0.0, 0.0, 0.15),
    ],
}
