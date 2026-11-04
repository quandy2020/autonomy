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

"""SMPL-X walk/stop gait from TrackVLA-style motion pickles.

Adapted from Meta Habitat ``HumanoidRearrangeController`` / TrackVLA
``evt_bench/humanoid_controller.py`` (MIT). Planar root motion is owned by
autosim Traffic; this class only advances joint + gait offset poses.
"""

from __future__ import annotations

import pickle as pkl
from pathlib import Path
from typing import Any, Sequence

import numpy as np

# Stop gait when planar speed is below this (m/s).
STOP_SPEED_MPS = 0.05


class Pose:
    """One SMPL-X frame: joint quaternions + root transform."""

    def __init__(self, joints_quat: Sequence[float], root_transform: Any) -> None:
        self.joints = list(joints_quat)
        self.root_transform = root_transform


class Motion:
    """Sequential walk mocap (joints + cumulative displacement)."""

    def __init__(
        self,
        joints_quat_array: np.ndarray,
        transform_array: np.ndarray,
        displacement: np.ndarray,
        fps: float,
        *,
        matrix4_cls: Any,
    ) -> None:
        self.num_poses = int(joints_quat_array.shape[0])
        self.poses = [
            Pose(
                joints_quat_array[index].reshape(-1),
                matrix4_cls(transform_array[index]),
            )
            for index in range(self.num_poses)
        ]
        self.fps = float(fps)
        self.displacement = np.asarray(displacement, dtype=np.float64)


class WalkGaitController:
    """Advance walk/stop poses from distance traveled (Traffic-driven root)."""

    def __init__(
        self,
        walk_pose_path: str | Path,
        *,
        motion_fps: float = 30.0,
        stop_speed: float = STOP_SPEED_MPS,
        mn: Any = None,
    ) -> None:
        if mn is None:
            import magnum as mn  # type: ignore

        path = Path(walk_pose_path)
        if not path.is_file():
            raise FileNotFoundError(f"humanoid motion pickle not found: {path}")
        with path.open("rb") as handle:
            walk_data = pkl.load(handle)
        walk_info = walk_data["walk_motion"]
        self.mn = mn
        self.walk_motion = Motion(
            walk_info["joints_array"],
            walk_info["transform_array"],
            walk_info["displacement"],
            walk_info["fps"],
            matrix4_cls=mn.Matrix4,
        )
        self.stop_pose = Pose(
            walk_data["stop_pose"]["joints"].reshape(-1),
            mn.Matrix4(walk_data["stop_pose"]["transform"]),
        )
        self.motion_fps = float(motion_fps)
        self.stop_speed = float(stop_speed)
        self.dist_per_step_size = float(
            self.walk_motion.displacement[-1] / max(1, self.walk_motion.num_poses)
        )
        self.walk_mocap_frame = 0
        self.joint_pose: list[float] = list(self.stop_pose.joints)
        # TrackVLA set_rest_position uses identity offset (joints only).
        self.obj_transform_offset = mn.Matrix4()

    def _gait_offset_from_pose(self, pose: Pose) -> Any:
        mn = self.mn
        obj_transform = mn.Matrix4(pose.root_transform)
        add_rot = mn.Matrix4.rotation(mn.Rad(np.pi), mn.Vector3(0, 1, 0))
        obj_transform = add_rot @ obj_transform
        # Strip forward (Z) translation from mocap; keep lateral/vertical bob.
        obj_transform.translation *= mn.Vector3.x_axis() + mn.Vector3.y_axis()
        return obj_transform

    def calculate_stop_pose(self) -> None:
        """Standing rest pose: rest joints + identity offset (TrackVLA)."""
        self.joint_pose = list(self.stop_pose.joints)
        self.obj_transform_offset = self.mn.Matrix4()

    def advance(self, speed_mps: float, dt: float) -> None:
        """Advance walk mocap by ``|speed| * dt`` meters, or stand if slow."""
        speed = abs(float(speed_mps))
        if speed < self.stop_speed or float(dt) <= 0.0:
            self.calculate_stop_pose()
            return
        distance_to_walk = speed * float(dt)
        # Frames to advance from mocap fps vs controller fps, capped by distance.
        step_size = max(1, int(self.walk_motion.fps / max(1e-6, self.motion_fps)))
        step_size = max(
            1,
            min(step_size, int(distance_to_walk / max(1e-9, self.dist_per_step_size))),
        )
        prev = self.walk_mocap_frame
        self.walk_mocap_frame = (
            self.walk_mocap_frame + step_size
        ) % self.walk_motion.num_poses
        pose = self.walk_motion.poses[self.walk_mocap_frame]
        self.joint_pose = list(pose.joints)
        self.obj_transform_offset = self._gait_offset_from_pose(pose)
        # Keep displacement bookkeeping consistent when looping.
        if self.walk_mocap_frame < prev:
            pass  # looped; nothing else required for Traffic-driven root

    def joints_and_offset(self) -> tuple[list[float], Any]:
        """Return ``(joint_positions, offset_Matrix4)`` for the AO."""
        return list(self.joint_pose), self.obj_transform_offset
