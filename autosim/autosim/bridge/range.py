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

"""URDF-mounted range sensors (ToF and cliff) resolved to single ray casts."""

from __future__ import annotations

import math
from typing import Any, Mapping

# A single-zone ToF emitter reports one spot; 2° keeps the reported field of
# view honest without pretending the beam is a cone.
DEFAULT_FOV = 0.0349
INFRARED = 1
ULTRASOUND = 0


def _check_limits(name: str, spec: Mapping[str, Any]) -> None:
    """Reject limits that would make a beam meaningless.

    Args:
        name: Sensor frame (or ``habitat.sensors.range`` for block defaults).
        spec: Dict carrying ``min_range`` / ``max_range`` / ``fov`` /
            ``rate_hz`` / ``radiation_type``.

    Raises:
        ValueError: Non-finite, inverted or out-of-range value.
    """
    if not math.isfinite(spec["min_range"]) or spec["min_range"] < 0.0:
        raise ValueError(f"range sensor {name} min_range must be finite and >= 0")
    if not math.isfinite(spec["max_range"]) or spec["max_range"] <= spec["min_range"]:
        raise ValueError(f"range sensor {name} max_range must exceed min_range")
    if not math.isfinite(spec["fov"]) or not 0.0 < spec["fov"] < math.pi:
        raise ValueError(f"range sensor {name} fov must be in (0, pi)")
    if not math.isfinite(spec["rate_hz"]) or spec["rate_hz"] <= 0.0:
        raise ValueError(f"range sensor {name} rate_hz must be finite and positive")
    if spec["radiation_type"] not in (INFRARED, ULTRASOUND):
        raise ValueError(
            f"range sensor {name} radiation_type must be 0 (ultrasound) or 1 (infrared)"
        )


def range_sensors(sensors: Mapping[str, Any]) -> list[dict]:
    """Parse ``habitat.sensors.range`` into one ray spec per frame.

    Args:
        sensors: ``habitat.sensors`` mapping.

    Returns:
        Per-sensor dicts with ``frame``, ``channel``, ``min_range``,
        ``max_range``, ``fov``, ``rate_hz`` and ``radiation_type``. Disabled
        blocks yield an empty list.

    Raises:
        ValueError: Malformed block, empty frame or out-of-range limits.
    """
    block = sensors.get("range") or {}
    if not isinstance(block, Mapping):
        raise ValueError("habitat.sensors.range must be a mapping")
    if not block.get("enabled", False):
        return []
    defaults = {
        "min_range": float(block.get("min_range", 0.02)),
        "max_range": float(block.get("max_range", 4.0)),
        "fov": float(block.get("fov", DEFAULT_FOV)),
        "rate_hz": float(block.get("rate_hz", 10.0)),
        "radiation_type": int(block.get("radiation_type", INFRARED)),
    }
    _check_limits("habitat.sensors.range", defaults)
    entries = block.get("sensors")
    if entries is None:
        entries = {}
    if isinstance(entries, Mapping):
        items = list(entries.items())
    elif isinstance(entries, (list, tuple)):
        items = []
        for entry in entries:
            if not isinstance(entry, Mapping) or not entry.get("frame"):
                raise ValueError("habitat.sensors.range.sensors entries need a frame")
            items.append((str(entry["frame"]), entry))
    else:
        raise ValueError("habitat.sensors.range.sensors must be a mapping or list")

    specs: list[dict] = []
    seen: set[str] = set()
    for frame, value in items:
        name = str(frame).strip()
        if not name:
            raise ValueError("habitat.sensors.range.sensors has an empty frame")
        if name in seen:
            raise ValueError(f"duplicate range sensor frame: {name}")
        seen.add(name)
        if isinstance(value, str):
            options: Mapping[str, Any] = {"channel": value}
        elif isinstance(value, Mapping):
            options = value
        elif value is None:
            options = {}
        else:
            raise ValueError(
                f"habitat.sensors.range.sensors.{name} must be a channel or mapping"
            )
        spec = {
            "frame": name,
            "channel": str(options.get("channel") or f"/range/{name}"),
            "min_range": float(options.get("min_range", defaults["min_range"])),
            "max_range": float(options.get("max_range", defaults["max_range"])),
            "fov": float(options.get("fov", defaults["fov"])),
            "rate_hz": float(options.get("rate_hz", defaults["rate_hz"])),
            "radiation_type": int(
                options.get("radiation_type", defaults["radiation_type"])
            ),
        }
        _check_limits(name, spec)
        specs.append(spec)
    return specs
