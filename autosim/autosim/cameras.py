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

"""Surround-camera rigs for BEV / occupancy (Fast-BEV, FlashOcc).

A rig is either 4 or 6 pinhole cameras. Each camera carries its own
FOV, image size, rate, and ``base_link`` pose. The union of the
horizontal FOVs must cover 360°.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Sequence, Tuple

# Off-the-shelf automotive wide camera (dashcam / ADAS pinhole), degrees.
COMMODITY_HFOV_DEG = 120.0

# Habitat ``SensorSubType`` names. ``type`` on each camera selects one.
CAMERA_TYPES = ("pinhole", "fisheye", "equirectangular")


@dataclass(frozen=True)
class SurroundCamera:
    """One pinhole in the ego surround rig."""

    name: str
    model: str
    yaw: float
    hfov_deg: float
    vfov_deg: float
    width: int
    height: int
    x: float
    y: float
    z: float
    rate_hz: float
    frame: str
    rgb_channel: str
    info_channel: str
    uuid: str

    @property
    def rgb_key(self) -> str:
        return f"surround_rgb_{self.name}"

    @property
    def info_key(self) -> str:
        return f"surround_info_{self.name}"


def layout_names(count: int) -> Tuple[Tuple[str, float], ...]:
    """Even azimuth layout for ``count`` cameras.

    Yaw is REP-103 (about +Z, 0 = forward, positive toward +Y / left).

    Args:
        count: ``4`` (front/left/rear/right) or ``6`` (Fast-BEV names).

    Returns:
        ``(name, yaw_rad)`` pairs.
    """
    if count == 4:
        names = ("front", "left", "rear", "right")
    elif count == 6:
        names = (
            "front",
            "front_left",
            "rear_left",
            "rear",
            "rear_right",
            "front_right",
        )
    else:
        raise ValueError("surround.count must be 4 or 6")
    step = 2.0 * math.pi / float(count)
    return tuple((name, _wrap(index * step)) for index, name in enumerate(names))


def camera_rig(sensors: Mapping[str, Any] | None) -> Mapping[str, Any] | None:
    """Return the 4/6-camera table from ``habitat.sensors``.

    ``sensors.cameras`` is the rig. ``sensors.surround`` remains accepted.
    """
    if not isinstance(sensors, Mapping):
        return None
    cameras = sensors.get("cameras")
    if isinstance(cameras, Mapping) and cameras:
        return cameras
    surround = sensors.get("surround")
    if isinstance(surround, Mapping):
        return surround
    return None


def resolve_surround(
    block: Mapping[str, Any] | None, urdf: Any = None
) -> Tuple[SurroundCamera, ...]:
    """Build the rig from a camera table or a legacy ``surround`` block.

    A camera table is keyed by view name (the 4-camera or 6-camera set) and
    is enabled by being present. A legacy block needs ``enabled: true`` and
    nests that table under ``cameras``. Each camera may set ``type``
    (``pinhole``, ``fisheye``, ``equirectangular``), ``hfov_deg``, ``width``,
    ``height``, ``rate_hz``, ``frame``, channels, and a ``base_link`` pose
    (``xyz`` plus ``yaw_deg`` or ``yaw``).

    Args:
        block: Camera table, surround mapping, or ``None``.

    Returns:
        Empty when disabled; otherwise one :class:`SurroundCamera` per view.
    """
    block = _normalize_rig(block)
    if block is None:
        return ()
    table = block.get("cameras")
    if isinstance(table, Mapping):
        table = _active_cameras(table)
        block = dict(block)
        block["cameras"] = table
    radius, eye = _mount(block)
    cameras = []
    for name, default_yaw in layout_names(rig_count(block)):
        spec = {}
        if isinstance(table, Mapping) and isinstance(table.get(name), Mapping):
            spec = table[name]
        hfov = float(spec.get("hfov_deg", block.get("hfov_deg", COMMODITY_HFOV_DEG)))
        width = int(spec.get("width", block.get("width", 960)))
        height = int(spec.get("height", block.get("height", 540)))
        vfov = _vertical_fov(spec, block, hfov, width, height)
        rate = float(spec.get("rate_hz", block.get("rate_hz", 10.0)))
        frame = str(spec.get("frame") or f"cam_{name}")
        x, y, z, yaw = _pose_of(spec, default_yaw, radius, eye, frame, urdf)
        rgb_channel, info_channel = _camera_channels(block, name)
        cameras.append(
            SurroundCamera(
                name=name,
                model=_camera_model(spec, block),
                yaw=yaw,
                hfov_deg=hfov,
                vfov_deg=vfov,
                width=width,
                height=height,
                x=x,
                y=y,
                z=z,
                rate_hz=rate,
                frame=frame,
                rgb_channel=rgb_channel,
                info_channel=info_channel,
                uuid=f"surround_{name}",
            )
        )
    return tuple(cameras)


def _normalize_rig(block: Mapping[str, Any] | None) -> Mapping[str, Any] | None:
    """Accept a bare camera table or a legacy ``{enabled, cameras}`` block."""
    if not isinstance(block, Mapping):
        return None
    names = {str(key) for key in block}
    known = {name for count in (4, 6) for name, _ in layout_names(count)}
    if names and names <= known:
        active = _active_cameras(block)
        active_names = set(active)
        for count in (4, 6):
            expected = {name for name, _ in layout_names(count)}
            if active_names == expected:
                return {"enabled": True, "cameras": active}
        raise ValueError(
            "enabled cameras must be exactly "
            "front/left/rear/right or "
            "front/front_left/rear_left/rear/rear_right/front_right"
        )
    if not block.get("enabled", False):
        return None
    return block


def _active_cameras(table: Mapping[str, Any]) -> Dict[str, Any]:
    """Cameras with ``enabled`` true (default true)."""
    active: Dict[str, Any] = {}
    for name, spec in table.items():
        if not isinstance(spec, Mapping):
            raise ValueError(f"habitat.sensors.cameras.{name} must be a mapping")
        if bool(spec.get("enabled", True)):
            active[str(name)] = spec
    return active


def _pose_of(
    spec: Mapping[str, Any],
    default_yaw: float,
    radius: float,
    eye: float,
    frame: str,
    urdf: Any,
) -> Tuple[float, float, float, float]:
    """Pose in ``base_link``. URDF joint origin wins over any yaml xyz/yaw."""
    if urdf is not None:
        try:
            return urdf.link_pose(frame)
        except ValueError as exc:
            raise ValueError(f"camera frame {frame!r} is missing from the URDF") from exc
    return _pose(spec, default_yaw, radius, eye)


def _vertical_fov(
    spec: Mapping[str, Any],
    block: Mapping[str, Any],
    hfov_deg: float,
    width: int,
    height: int,
) -> float:
    """Vertical FOV in degrees. Explicit ``vfov_deg``, else square-pixel from HFOV."""
    if "vfov_deg" in spec:
        return float(spec["vfov_deg"])
    if "vfov_deg" in block:
        return float(block["vfov_deg"])
    half_h = math.radians(hfov_deg) * 0.5
    aspect = float(height) / max(float(width), 1.0)
    return math.degrees(2.0 * math.atan(math.tan(half_h) * aspect))


def projection_height(width: int, height: int, hfov_deg: float, vfov_deg: float) -> int:
    """Render height whose square-pixel vertical FOV equals ``vfov_deg``.

    Habitat only accepts a horizontal FOV and derives the vertical one from
    the image aspect. Rendering at this height, then scaling to ``height``,
    makes the published image cover the configured vertical FOV.
    """
    half_h = math.radians(hfov_deg) * 0.5
    half_v = math.radians(vfov_deg) * 0.5
    ratio = math.tan(half_v) / max(math.tan(half_h), 1e-6)
    return max(1, int(round(float(width) * ratio)))


def _camera_model(spec: Mapping[str, Any], block: Mapping[str, Any]) -> str:
    """Projection model for one camera. Default is pinhole."""
    model = str(spec.get("type", block.get("type", "pinhole"))).strip().lower()
    if model not in CAMERA_TYPES:
        raise ValueError(
            "camera type must be one of " + ", ".join(CAMERA_TYPES) + f", got {model!r}"
        )
    return model


def rig_count(block: Mapping[str, Any]) -> int:
    """4 or 6, from the camera-name set, otherwise from ``count``."""
    table = block.get("cameras")
    if table is None:
        count = int(block.get("count", 6))
        if count not in (4, 6):
            raise ValueError("habitat.sensors.surround.count must be 4 or 6")
        return count
    names = _camera_names(table)
    for count in (4, 6):
        expected = {name for name, _ in layout_names(count)}
        if names == expected:
            stated = block.get("count")
            if stated is not None and int(stated) != count:
                raise ValueError(
                    f"habitat.sensors.surround.count is {stated} but cameras is the {count}-camera set"
                )
            return count
    raise ValueError(
        "habitat.sensors.surround.cameras must be exactly "
        "front/left/rear/right or "
        "front/front_left/rear_left/rear/rear_right/front_right"
    )


def _pose(
    spec: Mapping[str, Any], default_yaw: float, radius: float, eye: float
) -> Tuple[float, float, float, float]:
    """Camera origin and yaw in ``base_link`` (x forward, y left, z up)."""
    if "yaw_deg" in spec:
        yaw = math.radians(float(spec["yaw_deg"]))
    elif "yaw" in spec:
        yaw = float(spec["yaw"])
    else:
        yaw = default_yaw
    yaw = _wrap(yaw)
    xyz = spec.get("xyz")
    if isinstance(xyz, Sequence) and not isinstance(xyz, (str, bytes)) and len(xyz) == 3:
        return float(xyz[0]), float(xyz[1]), float(xyz[2]), yaw
    if xyz is not None:
        raise ValueError("surround camera xyz must be [x, y, z] in base_link")
    return radius * math.cos(yaw), radius * math.sin(yaw), eye, yaw


def _mount(block: Mapping[str, Any]) -> Tuple[float, float]:
    """Ring radius and camera height in the base frame."""
    mount = block.get("mount")
    if not isinstance(mount, Mapping):
        mount = {}
    radius = mount.get("radius", block.get("radius_m", 0.4))
    height = mount.get("height", block.get("sensor_height", 1.2))
    return float(radius), float(height)


def _camera_names(table: Any) -> set[str]:
    if isinstance(table, Mapping):
        return {str(name) for name in table}
    raise ValueError("habitat.sensors.surround.cameras must be a mapping")


def _camera_channels(block: Mapping[str, Any], name: str) -> Tuple[str, str]:
    """Image and camera_info channels for one surround camera."""
    table = block.get("cameras")
    if table is None:
        prefix = str(block.get("topic_prefix", "/surround")).rstrip("/")
        return f"{prefix}/{name}/image_raw", f"{prefix}/{name}/camera_info"
    spec = table.get(name) if isinstance(table, Mapping) else None
    if not isinstance(spec, Mapping):
        raise ValueError(f"habitat.sensors.surround.cameras.{name} must be a mapping")
    rgb = str(spec.get("rgb_channel") or "").strip()
    info = str(spec.get("info_channel") or "").strip()
    if not rgb or not info:
        raise ValueError(
            f"habitat.sensors.surround.cameras.{name} needs rgb_channel and info_channel"
        )
    return rgb, info


def covers_full_circle(
    cameras: Sequence[SurroundCamera], *, margin_rad: float = 1e-6
) -> bool:
    """True when every azimuth lies inside some camera's horizontal FOV."""
    if not cameras:
        return False
    # Sample finer than the smallest FOV.
    half = [math.radians(cam.hfov_deg) * 0.5 for cam in cameras]
    yaws = [cam.yaw for cam in cameras]
    steps = 720
    for index in range(steps):
        azimuth = -math.pi + (2.0 * math.pi) * index / steps
        if not any(_abs_delta(azimuth, yaw) <= half_fov + margin_rad for yaw, half_fov in zip(yaws, half)):
            return False
    return True


def ros_to_habitat_mount(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """REP-103 base offset → Habitat agent-local ``(x, y, z)`` (Y-up).

    Agent ``-Z`` is ROS ``+X``, agent ``-X`` is ROS ``+Y``, agent ``+Y`` is ROS ``+Z``.
    """
    return (-float(y), float(z), -float(x))


def _wrap(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def _abs_delta(a: float, b: float) -> float:
    return abs(_wrap(a - b))
