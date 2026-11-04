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

"""Optional overhead (top-down) RGB camera for global scene debugging."""

from __future__ import annotations

from typing import Any, Mapping

TOPDOWN_UUID = "topdown_rgb"


def topdown_spec(sensors: Mapping[str, Any] | None) -> dict[str, Any] | None:
    """Parse ``habitat.sensors.topdown``; return None when disabled.

    Default mode is orthographic + ``fit_scene`` so the whole navmesh AABB
    fits in one frame (agent-mounted pinhole FOV usually crops the apartment).
    """
    if not isinstance(sensors, Mapping):
        return None
    block = sensors.get("topdown")
    if not isinstance(block, Mapping) or not block.get("enabled", False):
        return None
    width = max(1, int(block.get("width", 2048)))
    height = max(1, int(block.get("height", 2048)))
    mode = str(block.get("mode") or "orthographic").strip().lower()
    if mode not in ("orthographic", "pinhole"):
        mode = "orthographic"
    ortho_scale = block.get("ortho_scale")
    return {
        "uuid": TOPDOWN_UUID,
        "width": width,
        "height": height,
        "mode": mode,
        "hfov_deg": float(block.get("hfov_deg", 90.0)),
        # World height above floor when fit_scene; else local +Y on the agent.
        "height_m": float(block.get("height_m", 12.0)),
        "rate_hz": float(block.get("rate_hz", 5.0)),
        # Center + size from navigable cells (tight floor plan, not full AABB).
        "fit_scene": bool(block.get("fit_scene", True)),
        # Rotate about +Y so navmesh principal axis is image-horizontal (untilted).
        "align_navmesh": bool(block.get("align_navmesh", True)),
        "ortho_margin": float(block.get("ortho_margin", 1.05)),
        "ortho_fill": float(block.get("ortho_fill", 0.92)),
        # Habitat ortho_scale: visible meters ≈ 1/scale (larger → zoom in).
        "ortho_scale": None if ortho_scale is None else float(ortho_scale),
        "frame": str(block.get("frame") or "topdown_link").strip() or "topdown_link",
        "channel": str(block.get("channel") or "/camera/topdown/image_raw").strip()
        or "/camera/topdown/image_raw",
        "info_channel": str(
            block.get("info_channel") or "/camera/topdown/camera_info"
        ).strip()
        or "/camera/topdown/camera_info",
    }
