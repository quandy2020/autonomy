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

"""Which sensors each robot and backend can actually provide.

``SENSOR_KEYS`` lists every sensor block a config may contain.  Not every
block makes sense for every robot: a differential-drive Husky has no joints to
publish, and a stereo pair needs two calibrated frames in the URDF.  This
module answers "can this combination work?" so the selector can explain a
refusal instead of failing later inside the renderer.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from autosim.robot.profiles import apply_profile, get_profile


@dataclass(frozen=True)
class SensorSupport:
    """Whether one sensor block is usable, and why not when it is not."""

    supported: bool
    reason: str = ""

    def __bool__(self) -> bool:
        """Allow ``if support:`` checks."""
        return self.supported


def _ok() -> SensorSupport:
    return SensorSupport(True)


def _no(reason: str) -> SensorSupport:
    return SensorSupport(False, reason)


def _frames(urdf_model: Any) -> set[str]:
    """Link names available in a loaded URDF model."""
    if urdf_model is None:
        return set()
    return {str(name) for name in urdf_model.parents} | {str(urdf_model.root)}


def support_matrix(
    robot_cfg: Mapping[str, Any],
    backend: str,
    urdf_model: Any = None,
) -> dict[str, SensorSupport]:
    """Return ``sensor key -> support`` for one robot/backend pair.

    Args:
        robot_cfg: ``habitat.robot`` mapping (profile defaults are applied).
        backend: ``habitat``, ``fake``, or ``isaac``.
        urdf_model: Loaded :class:`~autosim.robot.urdf.UrdfModel`, if any.

    Returns:
        One entry per key in :data:`~autosim.runtime.presets.SENSOR_KEYS`.
    """
    robot = apply_profile(robot_cfg)
    profile_name = str(robot.get("profile") or "").strip()
    articulated = False
    if profile_name:
        try:
            articulated = get_profile(profile_name).articulated
        except KeyError:
            articulated = False
    has_urdf = urdf_model is not None
    links = _frames(urdf_model)
    render_backend = backend in ("habitat", "isaac")

    matrix: dict[str, SensorSupport] = {
        "camera": _ok(),
        "lidar_2d": _ok(),
        "lidar_3d": _ok(),
        "imu": _ok(),
        "odom": _ok(),
    }

    if articulated:
        matrix["joint_states"] = _ok()
    else:
        matrix["joint_states"] = _no(
            f"robot {profile_name or '(unnamed)'} has no articulated joints"
        )

    if not render_backend:
        for key in ("stereo", "range", "cameras", "topdown", "chase", "panorama"):
            matrix[key] = _no(f"backend {backend} does not render sensor images")
        return matrix

    if not has_urdf:
        for key in ("stereo", "range", "cameras"):
            matrix[key] = _no("a URDF is required to mount this sensor")
    else:
        matrix["stereo"] = _ok()
        matrix["range"] = _ok()
        matrix["cameras"] = _ok()
        if not links:
            for key in ("stereo", "range", "cameras"):
                matrix[key] = _no("the URDF exposes no links")

    matrix["topdown"] = _ok()
    matrix["chase"] = _ok()
    matrix["panorama"] = _ok()
    return matrix


def describe(matrix: Mapping[str, SensorSupport]) -> str:
    """Format a support matrix as one line per sensor.

    Args:
        matrix: Result of :func:`support_matrix`.

    Returns:
        ``sensor: yes`` / ``sensor: no (reason)`` lines.
    """
    lines = []
    for key in sorted(matrix):
        support = matrix[key]
        if support.supported:
            lines.append(f"{key}: yes")
        else:
            lines.append(f"{key}: no ({support.reason})")
    return "\n".join(lines)
