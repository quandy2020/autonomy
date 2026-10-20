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

"""Map-frame pose ``(x, y, yaw)`` ↔ Isaac / USD world transforms (z-up).

autosim map: z-up, yaw about +z, +x forward at yaw=0.
Isaac stages are typically z-up; keep all axis remaps here.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def map_to_world(
    x: float, y: float, yaw: float, *, z: float = 0.0
) -> tuple[np.ndarray, np.ndarray]:
    """Planar map pose → world translation (3,) and quaternion wxyz."""
    half = 0.5 * float(yaw)
    quat_wxyz = np.array(
        [math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64
    )
    translation = np.array([float(x), float(y), float(z)], dtype=np.float64)
    return translation, quat_wxyz


def world_to_map(
    translation: np.ndarray, quat_wxyz: np.ndarray
) -> tuple[float, float, float]:
    """World pose → planar map ``(x, y, yaw)``."""
    t = np.asarray(translation, dtype=np.float64).reshape(3)
    q = np.asarray(quat_wxyz, dtype=np.float64).reshape(4)
    # yaw from wxyz quaternion (rotation about +z).
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return float(t[0]), float(t[1]), float(yaw)


def quat_wxyz_to_matrix(quat_wxyz: np.ndarray) -> np.ndarray:
    """Unit quaternion wxyz → 3×3 rotation matrix."""
    w, x, y, z = np.asarray(quat_wxyz, dtype=np.float64).reshape(4)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
