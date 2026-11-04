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

"""Single kinematic SMPL-X humanoid in a Habitat-Sim session.

Transform math follows TrackVLA / Habitat ``KinematicHumanoid.set_joint_transform``:
base (feet on floor, Y-up) × ``offset_transform_base⁻¹`` × gait offset.
"""

from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Any

from autosim.backends.habitat.humanoids.assets import HumanoidAsset
from autosim.backends.habitat.humanoids.controller import WalkGaitController

logger = logging.getLogger(__name__)

# TrackVLA MobileManipulatorParams.base_offset = (0, -0.9, 0):
# feet at ``p`` ⇒ root translation ``p - base_offset`` = ``p + (0, 0.9, 0)``.
DEFAULT_BASE_OFFSET_Y = 0.9
# KinematicHumanoid.offset_rot
_OFFSET_ROT = -0.5 * math.pi


def _offset_transform_base(mn: Any) -> Any:
    """Same as TrackVLA ``KinematicHumanoid.offset_transform_base``."""
    add_rot = mn.Matrix4.rotation(mn.Rad(_OFFSET_ROT), mn.Vector3(0, 1.0, 0))
    perm = mn.Matrix4.rotation(mn.Rad(_OFFSET_ROT), mn.Vector3(0, 0, 1.0))
    return perm @ add_rot


class HumanoidActor:
    """Kinematic articulated object + walk gait for one pedestrian."""

    def __init__(
        self,
        session: Any,
        asset: HumanoidAsset,
        *,
        habitat_sim: Any,
        mn: Any,
        base_offset_y: float = DEFAULT_BASE_OFFSET_Y,
        motion_fps: float = 30.0,
    ) -> None:
        self.session = session
        self.asset = asset
        self.habitat_sim = habitat_sim
        self.mn = mn
        self.base_offset_y = float(base_offset_y)
        self.sim_obj: Any = None
        self._offset_base = _offset_transform_base(mn)
        self._offset_base_inv = self._offset_base.inverted()
        self.controller = WalkGaitController(
            asset.motion, motion_fps=motion_fps, mn=mn
        )
        self._spawn(habitat_sim)

    def _spawn(self, habitat_sim: Any) -> None:
        manager = self.session.get_articulated_object_manager()
        ao = manager.add_articulated_object_from_urdf(
            filepath=str(self.asset.urdf),
            fixed_base=False,
            maintain_link_order=True,
        )
        ao.motion_type = habitat_sim.physics.MotionType.KINEMATIC
        try:
            for motor_id in list(ao.existing_joint_motor_ids):
                ao.remove_joint_motor(motor_id)
        except Exception as exc:  # noqa: BLE001
            logger.debug("remove joint motors (%s): %s", self.asset.name, exc)
        self.sim_obj = ao
        self.controller.calculate_stop_pose()
        self.apply_pose(0.0, 0.0, 0.0, floor_y=0.0)

    def apply_pose(
        self,
        map_x: float,
        map_y: float,
        yaw: float,
        *,
        floor_y: float,
        speed_mps: float = 0.0,
        dt: float = 0.0,
    ) -> None:
        """Write Traffic planar pose into the AO (map z-up → Habitat Y-up)."""
        if self.sim_obj is None:
            return
        mn = self.mn
        if dt > 0.0:
            self.controller.advance(speed_mps, dt)
        joints, offset = self.controller.joints_and_offset()
        try:
            self.sim_obj.joint_positions = joints
        except Exception as exc:  # noqa: BLE001
            logger.debug("set joint_positions (%s): %s", self.asset.name, exc)

        # Feet on Habitat floor. map_y = -Habitat_Z.
        feet = mn.Vector3(float(map_x), float(floor_y), -float(map_y))
        # params.base_offset = (0, -base_offset_y, 0)
        root = feet - mn.Vector3(0.0, -self.base_offset_y, 0.0)

        # TrackVLA walk controller: look_at (yaw about Y) then −90° about X so
        # SMPL-X (Z-up) stands in Habitat (Y-up). Do NOT apply an extra Z-up
        # rotation after the gait offset — that laid characters on their side.
        translate = mn.Matrix4.translation(root)
        rotate_y = mn.Matrix4.rotation(mn.Rad(float(yaw)), mn.Vector3(0, 1, 0))
        rot_x = mn.Matrix4.rotation(mn.Rad(-0.5 * math.pi), mn.Vector3(1, 0, 0))
        base_transform = translate @ rotate_y @ rot_x

        # set_joint_transform: (base @ offset_transform_base⁻¹) @ offset
        final = (base_transform @ self._offset_base_inv) @ offset
        try:
            self.sim_obj.transformation = final
        except Exception:
            try:
                self.sim_obj.translation = final.translation
                self.sim_obj.rotation = mn.Quaternion.from_matrix(final.rotation())
            except Exception as exc:  # noqa: BLE001
                logger.debug("set transform (%s): %s", self.asset.name, exc)

    def remove(self) -> None:
        """Remove the articulated object from the scene."""
        if self.sim_obj is None or self.session is None:
            self.sim_obj = None
            return
        try:
            manager = self.session.get_articulated_object_manager()
            handle = getattr(self.sim_obj, "handle", None)
            if handle is not None:
                manager.remove_object_by_handle(handle)
            else:
                obj_id = getattr(self.sim_obj, "object_id", None)
                if obj_id is not None:
                    manager.remove_object_by_id(obj_id)
        except Exception as exc:  # noqa: BLE001
            logger.debug("remove humanoid %s: %s", self.asset.name, exc)
        self.sim_obj = None


def load_actor(
    session: Any,
    asset: HumanoidAsset,
    *,
    habitat_sim: Any | None = None,
    mn: Any | None = None,
    base_offset_y: float = DEFAULT_BASE_OFFSET_Y,
    motion_fps: float = 30.0,
) -> HumanoidActor:
    """Factory that imports habitat_sim / magnum when omitted."""
    hs = habitat_sim
    mag = mn
    if hs is None:
        import habitat_sim as _hs  # type: ignore

        hs = _hs
    if mag is None:
        import magnum as _mn  # type: ignore

        mag = _mn
    if not Path(asset.urdf).is_file():
        raise FileNotFoundError(f"humanoid URDF missing: {asset.urdf}")
    return HumanoidActor(
        session,
        asset,
        habitat_sim=hs,
        mn=mag,
        base_offset_y=base_offset_y,
        motion_fps=motion_fps,
    )
