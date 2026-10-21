# Copyright 2026 The Openbot Authors (duyongquan)
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

"""TUM trajectory helpers (timestamp tx ty tz qx qy qz qw)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class TumPose:
    t: float
    xyz: tuple[float, float, float]
    quat_xyzw: tuple[float, float, float, float]


def pose_to_mat44(xyz: tuple[float, float, float],
                    quat_xyzw: tuple[float, float, float, float]) -> np.ndarray:
    """SE(3) as 4x4 from translation + quaternion (xyzw)."""
    x, y, z, w = quat_xyzw
    n = float(np.sqrt(x * x + y * y + z * z + w * w))
    if n < 1e-12:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    else:
        x, y, z, w = x / n, y / n, z / n, w / n
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    r = np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )
    t = np.eye(4, dtype=np.float64)
    t[:3, :3] = r
    t[0, 3] = xyz[0]
    t[1, 3] = xyz[1]
    t[2, 3] = xyz[2]
    return t


def mat44_to_pose(mat: np.ndarray) -> tuple[tuple[float, float, float],
                                              tuple[float, float, float, float]]:
    """Extract translation + quaternion (xyzw) from SE(3) 4x4."""
    r = mat[:3, :3]
    trace = float(r[0, 0] + r[1, 1] + r[2, 2])
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2, 1] - r[1, 2]) * s
        y = (r[0, 2] - r[2, 0]) * s
        z = (r[1, 0] - r[0, 1]) * s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    xyz = (float(mat[0, 3]), float(mat[1, 3]), float(mat[2, 3]))
    return xyz, (float(x), float(y), float(z), float(w))


def relative_to_origin(
    pose: TumPose, origin: TumPose
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Express ``pose`` in the frame of ``origin`` (origin → identity)."""
    t_origin = pose_to_mat44(origin.xyz, origin.quat_xyzw)
    t_pose = pose_to_mat44(pose.xyz, pose.quat_xyzw)
    return mat44_to_pose(np.linalg.inv(t_origin) @ t_pose)


def load_tum(path: str | Path) -> list[TumPose]:
    """Load a TUM pose file; skip blank / ``#`` comment lines."""
    poses: list[TumPose] = []
    text = Path(path).read_text()
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        t = float(parts[0])
        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
        qx, qy, qz, qw = (
            float(parts[4]),
            float(parts[5]),
            float(parts[6]),
            float(parts[7]),
        )
        poses.append(TumPose(t=t, xyz=(x, y, z), quat_xyzw=(qx, qy, qz, qw)))
    if len(poses) < 2:
        raise ValueError(f"need ≥2 poses in {path}, got {len(poses)}")
    return poses


def resolve_spires_gt(seq_root: Path, override: Path | None = None) -> Path | None:
    """Pick a ground-truth TUM file under an Oxford Spires sequence.

    Preference (eval-aligned first):
    1. ``--gt-file`` override
    2. ``processed/vilens-slam/gt.tum``
    3. ``processed/trajectory/gt-tum.txt``
    """
    if override is not None:
        p = Path(override).expanduser().resolve()
        return p if p.is_file() else None
    root = Path(seq_root).resolve()
    candidates = (
        root / "processed" / "vilens-slam" / "gt.tum",
        root / "processed" / "trajectory" / "gt-tum.txt",
    )
    for c in candidates:
        if c.is_file() and c.stat().st_size > 0:
            return c
    return None


class TumIndex:
    """Nearest-neighbor pose lookup by timestamp."""

    def __init__(self, poses: list[TumPose]) -> None:
        self.poses = poses
        self.times = np.asarray([p.t for p in poses], dtype=np.float64)

    def __len__(self) -> int:
        return len(self.poses)

    def nearest(self, t: float, *, max_dt: float | None = None) -> TumPose | None:
        i = int(np.searchsorted(self.times, t))
        cands: list[int] = []
        if 0 <= i < len(self.times):
            cands.append(i)
        if i - 1 >= 0:
            cands.append(i - 1)
        if not cands:
            return None
        best = min(cands, key=lambda j: abs(float(self.times[j]) - t))
        dt = abs(float(self.times[best]) - t)
        if max_dt is not None and dt > max_dt:
            return None
        return self.poses[best]

    def end_index_at(self, t: float) -> int:
        """Exclusive end index: poses with ``pose.t <= t`` (empty → 0)."""
        return int(np.searchsorted(self.times, t, side="right"))
