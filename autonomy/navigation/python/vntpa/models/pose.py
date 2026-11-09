# Copyright 2026 The Navigation Authors (duyongquan)
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

"""Pose utilities kept independent of the ONNX graph."""

from __future__ import annotations

import numpy as np


def to_habitat(pose: np.ndarray) -> np.ndarray:
    """ROS ``(x, y, yaw)`` to Habitat ``(x, z, yaw)`` for pose RoPE."""
    pose = np.asarray(pose, dtype=np.float32).reshape(-1)
    return np.array([-pose[1], -pose[0], pose[2]], dtype=np.float32)


def goal_to_habitat(goal: np.ndarray) -> np.ndarray:
    """ROS ``(x, y)`` goal to Habitat ``(x, z)``."""
    goal = np.asarray(goal, dtype=np.float32).reshape(-1)
    return np.array([-goal[1], -goal[0]], dtype=np.float32)


def relative(pose: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Return planar relative pose ``(dx, dy, dyaw)``."""
    pose = np.asarray(pose, dtype=np.float32)
    reference = np.asarray(reference, dtype=np.float32)
    delta = pose[:2] - reference[:2]
    yaw = float(pose[2] - reference[2])
    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            cos_yaw * delta[0] + sin_yaw * delta[1],
            -sin_yaw * delta[0] + cos_yaw * delta[1],
            yaw,
        ],
        dtype=np.float32,
    )
