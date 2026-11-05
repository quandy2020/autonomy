# Copyright 2026 The Openbot Authors (duyongquan)
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

"""Optional third-person chase RGB camera for watching the robot move."""

from __future__ import annotations

import math
from typing import Any, Mapping

CHASE_UUID = "chase_rgb"


def chase_spec(sensors: Mapping[str, Any] | None) -> dict[str, Any] | None:
    """Parse ``habitat.sensors.chase``; return None when disabled.

    Camera sits behind and above the agent (Habitat local ``+Z`` / ``+Y``),
    pitched down toward the chassis so the robot stays in frame while moving.
    """
    if not isinstance(sensors, Mapping):
        return None
    block = sensors.get("chase")
    if not isinstance(block, Mapping) or not block.get("enabled", False):
        return None
    width = max(1, int(block.get("width", 640)))
    height = max(1, int(block.get("height", 480)))
    # Tight default for TurtleBot-scale platforms; override in YAML if needed.
    distance_m = float(block.get("distance_m", 1.0))
    height_m = float(block.get("height_m", 0.85))
    look_at_height_m = float(block.get("look_at_height_m", 0.15))
    pitch = block.get("pitch_deg")
    if pitch is None:
        # Look from (0, height, distance) toward (0, look_at, 0) in agent frame.
        pitch_rad = -math.atan2(
            max(1e-3, height_m - look_at_height_m),
            max(1e-3, distance_m),
        )
        pitch_deg = math.degrees(pitch_rad)
    else:
        pitch_deg = float(pitch)
    return {
        "uuid": str(block.get("uuid") or CHASE_UUID),
        "width": width,
        "height": height,
        "hfov_deg": float(block.get("hfov_deg", 70.0)),
        "distance_m": distance_m,
        "height_m": height_m,
        "look_at_height_m": look_at_height_m,
        "pitch_deg": pitch_deg,
        "rate_hz": float(block.get("rate_hz", 10.0)),
        "frame": str(block.get("frame") or "chase_link").strip() or "chase_link",
        "channel": str(block.get("channel") or "/camera/chase/image_raw").strip()
        or "/camera/chase/image_raw",
        "info_channel": str(
            block.get("info_channel") or "/camera/chase/camera_info"
        ).strip()
        or "/camera/chase/camera_info",
    }
