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

"""Differential-drive wheel joints for turtlebot / husky style robots."""

from __future__ import annotations

import logging
import math
from typing import Any, Sequence

logger = logging.getLogger(__name__)

# Common URDF wheel joint name pairs (left, right).
WHEEL_NAME_PAIRS = (
    ("wheel_left_joint", "wheel_right_joint"),
    ("left_wheel_joint", "right_wheel_joint"),
    ("LW", "RW"),
    ("joint_wheel_left", "joint_wheel_right"),
)


def twist_to_wheel_speeds(
    vx: float,
    wz: float,
    *,
    track_width: float,
    wheel_radius: float,
) -> tuple[float, float]:
    """Map planar twist to left/right wheel angular velocities (rad/s)."""
    track = max(1e-4, float(track_width))
    radius = max(1e-4, float(wheel_radius))
    left = (float(vx) - 0.5 * float(wz) * track) / radius
    right = (float(vx) + 0.5 * float(wz) * track) / radius
    return float(left), float(right)


class WheeledDrive:
    """Apply ``(vx, wz)`` to a pair of continuous/revolute wheel joints."""

    def __init__(
        self,
        stage: Any,
        articulation_root: str,
        *,
        left_joint: str,
        right_joint: str,
        track_width: float = 0.16,
        wheel_radius: float = 0.033,
    ) -> None:
        self.stage = stage
        self.articulation_root = str(articulation_root)
        self.left_joint = str(left_joint)
        self.right_joint = str(right_joint)
        self.track_width = float(track_width)
        self.wheel_radius = float(wheel_radius)
        self._drives: dict[str, Any] = {}
        self._bind_drives()

    @classmethod
    def try_create(
        cls,
        stage: Any,
        under: str,
        *,
        track_width: float = 0.16,
        wheel_radius: float = 0.033,
        joint_names: Sequence[str] | None = None,
    ) -> "WheeledDrive | None":
        """Find a left/right wheel pair under ``under`` and bind drives."""
        if stage is None:
            return None
        names = cls._discover_wheels(stage, under, joint_names=joint_names)
        if names is None:
            return None
        left, right = names
        try:
            drive = cls(
                stage,
                under,
                left_joint=left,
                right_joint=right,
                track_width=track_width,
                wheel_radius=wheel_radius,
            )
        except Exception as exc:
            logger.warning("WheeledDrive bind failed: %s", exc)
            return None
        if not drive._drives:
            return None
        logger.info(
            "WheeledDrive ready under %s (%s / %s)", under, left, right
        )
        return drive

    @staticmethod
    def _discover_wheels(
        stage: Any,
        under: str,
        *,
        joint_names: Sequence[str] | None = None,
    ) -> tuple[str, str] | None:
        from pxr import Usd, UsdPhysics

        root = stage.GetPrimAtPath(under)
        if not root or not root.IsValid():
            return None
        found: dict[str, str] = {}
        for prim in Usd.PrimRange(root):
            name = prim.GetName()
            type_name = prim.GetTypeName()
            if "Joint" not in type_name and "Revolute" not in type_name:
                if not prim.IsA(UsdPhysics.Joint):
                    continue
            found[name] = str(prim.GetPath())
        if joint_names and len(joint_names) >= 2:
            left, right = str(joint_names[0]), str(joint_names[1])
            if left in found and right in found:
                return left, right
        for left, right in WHEEL_NAME_PAIRS:
            if left in found and right in found:
                return left, right
        # Fuzzy: any *left*wheel* / *right*wheel*
        lefts = [n for n in found if "left" in n.lower() and "wheel" in n.lower()]
        rights = [n for n in found if "right" in n.lower() and "wheel" in n.lower()]
        if lefts and rights:
            return lefts[0], rights[0]
        return None

    def _bind_drives(self) -> None:
        from pxr import UsdPhysics

        for name in (self.left_joint, self.right_joint):
            # Prefer path under articulation root.
            candidates = (
                f"{self.articulation_root.rstrip('/')}/{name}",
                f"{self.articulation_root.rstrip('/')}/joints/{name}",
            )
            prim = None
            for path in candidates:
                prim = self.stage.GetPrimAtPath(path)
                if prim and prim.IsValid():
                    break
            if prim is None or not prim.IsValid():
                # Stage-wide name search.
                from pxr import Usd

                root = self.stage.GetPrimAtPath(self.articulation_root)
                if root and root.IsValid():
                    for child in Usd.PrimRange(root):
                        if child.GetName() == name:
                            prim = child
                            break
            if prim is None or not prim.IsValid():
                continue
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if not drive:
                try:
                    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                except Exception:
                    continue
            try:
                drive.GetStiffnessAttr().Set(0.0)
                drive.GetDampingAttr().Set(1e3)
            except Exception:
                pass
            self._drives[name] = drive

    def apply(self, vx: float, wz: float) -> bool:
        """Set wheel target velocities from planar twist. Returns success."""
        if len(self._drives) < 2:
            return False
        left_w, right_w = twist_to_wheel_speeds(
            vx,
            wz,
            track_width=self.track_width,
            wheel_radius=self.wheel_radius,
        )
        mapping = {
            self.left_joint: left_w,
            self.right_joint: right_w,
        }
        ok = True
        for name, speed in mapping.items():
            drive = self._drives.get(name)
            if drive is None:
                ok = False
                continue
            try:
                drive.GetTargetVelocityAttr().Set(float(speed))
            except Exception:
                try:
                    drive.CreateTargetVelocityAttr(float(speed))
                except Exception:
                    ok = False
        return ok
