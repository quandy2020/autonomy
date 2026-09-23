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

"""Settings for the equirectangular surround panorama.

Habitat renders this as one 360°×180° image. The center column is forward
and image-left is the robot's left.
"""

from __future__ import annotations

from typing import Any, Mapping, Optional, Tuple

PANORAMA_UUID = "surround_panorama"


def panorama_spec(sensors: Mapping[str, Any] | None) -> Optional[dict]:
    """Return the panorama publisher settings, or ``None`` when disabled."""
    if not isinstance(sensors, Mapping):
        return None
    block = sensors.get("panorama")
    if not isinstance(block, Mapping) or not bool(block.get("enabled", False)):
        return None
    channel = str(block.get("channel") or "/surround/panorama").strip()
    if not channel:
        raise ValueError("habitat.sensors.panorama.channel is empty")
    width = int(block.get("width", 1920))
    height = int(block.get("height", 960))
    if width < 2 or height < 2:
        raise ValueError("habitat.sensors.panorama width and height must be >= 2")
    return {
        "channel": channel,
        "frame": str(block.get("frame") or "base_link"),
        "width": width,
        "height": height,
        "xyz": _xyz(block.get("xyz", (0.0, 0.0, 1.2))),
        "uuid": PANORAMA_UUID,
    }


def _xyz(value: Any) -> Tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("habitat.sensors.panorama.xyz must be [x, y, z]")
    return float(value[0]), float(value[1]), float(value[2])
