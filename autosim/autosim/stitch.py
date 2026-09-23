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

"""Cylindrical 360° stitch of a surround rig.

Each pinhole is projected with its known yaw and FOV. The panorama center
is forward; image left is the robot's left. Overlaps blend by how face-on
the camera is and by distance from that image's border.
"""

from __future__ import annotations

import math
from typing import Any, Mapping, Optional, Sequence, Tuple

import numpy as np

from autosim.cameras import SurroundCamera


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
    height = int(block.get("height", 480))
    if width < 2 or height < 2:
        raise ValueError("habitat.sensors.panorama width and height must be >= 2")
    return {
        "channel": channel,
        "frame": str(block.get("frame") or "base_link"),
        "width": width,
        "height": height,
    }


def stitch_cylindrical(
    images: Mapping[str, np.ndarray],
    cameras: Sequence[SurroundCamera],
    width: int,
    height: int,
) -> np.ndarray:
    """Project ``cameras`` onto one cylindrical RGB image.

    Args:
        images: ``camera.uuid → HxWx3`` uint8 (or float) views.
        cameras: Rig used to render ``images``.
        width: Panorama columns. Column 0 is rear-left; the center is forward.
        height: Panorama rows. Row 0 is the top of the cameras' vertical FOV.

    Returns:
        ``height × width × 3`` uint8. Unseen pixels stay black.
    """
    canvas = np.zeros((int(height), int(width), 3), dtype=np.float32)
    weights = np.zeros((int(height), int(width)), dtype=np.float32)
    if width < 2 or height < 2 or not cameras:
        return canvas.astype(np.uint8)

    half_v = math.radians(max(cam.vfov_deg for cam in cameras)) * 0.5
    columns = np.arange(width, dtype=np.float64) + 0.5
    rows = np.arange(height, dtype=np.float64) + 0.5
    azimuth = math.pi - columns * (2.0 * math.pi / float(width))
    elevation = half_v - rows * (2.0 * half_v / float(height))
    azimuth, elevation = np.meshgrid(azimuth, elevation)
    cos_el = np.cos(elevation)
    sin_el = np.sin(elevation)

    for camera in cameras:
        image = images.get(camera.uuid)
        if image is None:
            continue
        view = np.asarray(image)
        if view.ndim != 3 or view.shape[2] < 3 or view.shape[0] < 2 or view.shape[1] < 2:
            continue
        color, weight = _project(view[..., :3], camera, azimuth, elevation, cos_el, sin_el)
        canvas += color * weight[..., None]
        weights += weight

    seen = weights > 1e-6
    canvas[seen] /= weights[seen, None]
    return np.clip(canvas, 0.0, 255.0).astype(np.uint8)


def _project(
    image: np.ndarray,
    camera: SurroundCamera,
    azimuth: np.ndarray,
    elevation: np.ndarray,
    cos_el: np.ndarray,
    sin_el: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Sample ``image`` for each panorama ray. Weight is 0 outside the FOV."""
    src_h, src_w = image.shape[:2]
    fx = (src_w * 0.5) / math.tan(math.radians(camera.hfov_deg) * 0.5)
    fy = (src_h * 0.5) / math.tan(math.radians(camera.vfov_deg) * 0.5)
    cx = (src_w - 1) * 0.5
    cy = (src_h - 1) * 0.5
    delta = _wrap(azimuth - camera.yaw)
    z = cos_el * np.cos(delta)
    x = -cos_el * np.sin(delta)
    y = -sin_el
    safe_z = np.maximum(z, 1e-6)
    u = cx + fx * (x / safe_z)
    v = cy + fy * (y / safe_z)
    inside = (z > 1e-3) & (u >= 0.0) & (u <= src_w - 1) & (v >= 0.0) & (v <= src_h - 1)
    margin = max(2.0, 0.04 * min(src_w, src_h))
    border = np.minimum(np.minimum(u, (src_w - 1) - u), np.minimum(v, (src_h - 1) - v))
    feather = np.clip(border / margin, 0.0, 1.0)
    weight = np.where(inside, z * feather, 0.0).astype(np.float32)
    return _bilinear(image, u, v), weight


def _bilinear(image: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Sample ``image`` at floating pixel coords. Out-of-range coords are clipped."""
    src_h, src_w = image.shape[:2]
    u = np.clip(u, 0.0, src_w - 1.001)
    v = np.clip(v, 0.0, src_h - 1.001)
    u0 = np.floor(u).astype(np.int32)
    v0 = np.floor(v).astype(np.int32)
    u1 = np.minimum(u0 + 1, src_w - 1)
    v1 = np.minimum(v0 + 1, src_h - 1)
    du = (u - u0).astype(np.float32)[..., None]
    dv = (v - v0).astype(np.float32)[..., None]
    view = image.astype(np.float32, copy=False)
    top = view[v0, u0] * (1.0 - du) + view[v0, u1] * du
    bot = view[v1, u0] * (1.0 - du) + view[v1, u1] * du
    return top * (1.0 - dv) + bot * dv


def _wrap(angle: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(angle), np.cos(angle))
