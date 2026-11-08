"""Frame conventions and the transforms the pipeline uses.

World frame (habitat): right-handed, +x right, +y up, +z back. Yaw is the
counter-clockwise rotation about +y; an agent at yaw 0 faces -z, so its forward
direction on the ground is (-sin yaw, -cos yaw). Vocabulary, used across the repo:

    point     (x, y, z) world, e.g. a navmesh point
    position  (x, z) on the ground plane
    pose      (x, z, yaw)

Camera frame: x right, y up, z back. A depth pixel (u, v) at depth d unprojects to
((u - CX) d / FX, -(v - CY) d / FY, -d).
"""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np
import quaternion as npq            # numpy-quaternion (a habitat-sim dependency)

from thirdparty.pan.util.constants import SENSOR_HFOV_DEG, SENSOR_RES


# ---------------------------------------------------------------- sensor model
_H_IM, _W_IM = SENSOR_RES
FX = _W_IM / 2.0 / math.tan(math.radians(SENSOR_HFOV_DEG / 2.0))
FY = FX                             # square pixels
CX = _W_IM / 2.0 - 0.5
CY = _H_IM / 2.0 - 0.5
DEPTH_MIN = 0.1                     # metres; habitat renders no-hit as 0


# ---------------------------------------------------------------- angles
def wrap_angle(a):
    """Wrap to [-pi, pi). Works on scalars and arrays."""
    return np.remainder(a + np.pi, 2.0 * np.pi) - np.pi


def heading_of(dx: float, dz: float) -> float:
    """Yaw that faces the ground direction (dx, dz)."""
    return math.atan2(-dx, -dz)


def R_ground(theta: float, dtype=np.float64) -> np.ndarray:
    """(x, z) block of R_y(theta), for column vectors: rotating by theta about +y maps a
    position p to R_ground(theta) @ p and a yaw psi to psi + theta."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, s],
                     [-s, c]], dtype)


# ---------------------------------------------------------------- rotations
def heading_from_quaternion(rot: npq.quaternion) -> float:
    """Yaw of an upright rotation, from its world-frame forward (-z) axis."""
    f = npq.rotate_vectors(rot, np.array([0.0, 0.0, -1.0]))
    return heading_of(f[0], f[2])


def heading_from_rotation(R: np.ndarray) -> float:
    """Yaw of a 3x3 rotation or 4x4 camera-to-world, from its forward (-z) column."""
    R = np.asarray(R, dtype=np.float64)
    return float(np.arctan2(R[0, 2], R[2, 2]))


# ---------------------------------------------------------------- top-down raster
def raster_cell(position, origin_position, mpp: float) -> Tuple[int, int]:
    """(x, z) `position` -> (row, col) in a raster of `mpp` metres per cell whose cell (0, 0)
    starts at `origin_position`; indices truncate toward zero."""
    return (int((position[1] - origin_position[1]) / mpp),
            int((position[0] - origin_position[0]) / mpp))
