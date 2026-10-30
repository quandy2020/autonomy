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

"""KITTI odometry stereo reader."""

from __future__ import annotations

import math
from collections.abc import Iterator
from pathlib import Path

from autonomy.tools.datasets.readers.visual import (
    ImageSample,
    ImuSample,
    PoseSample,
    VisualFrame,
    duration_seconds,
    frame_indices,
    load_image,
    seconds_text_to_ns,
    validate_strictly_increasing,
)


def _rotation_matrix_to_quaternion_xyzw(
    values: list[float], *, source: str
) -> tuple[float, float, float, float]:
    """Convert a row-major 3x3 rotation matrix to a unit quaternion."""
    r00, r01, r02, r10, r11, r12, r20, r21, r22 = values
    trace = r00 + r11 + r22
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        qw = 0.25 * scale
        qx = (r21 - r12) / scale
        qy = (r02 - r20) / scale
        qz = (r10 - r01) / scale
    elif r00 > r11 and r00 > r22:
        scale = 2.0 * math.sqrt(max(0.0, 1.0 + r00 - r11 - r22))
        if scale <= 1e-12:
            raise ValueError(f"{source}: invalid rotation matrix")
        qw = (r21 - r12) / scale
        qx = 0.25 * scale
        qy = (r01 + r10) / scale
        qz = (r02 + r20) / scale
    elif r11 > r22:
        scale = 2.0 * math.sqrt(max(0.0, 1.0 + r11 - r00 - r22))
        if scale <= 1e-12:
            raise ValueError(f"{source}: invalid rotation matrix")
        qw = (r02 - r20) / scale
        qx = (r01 + r10) / scale
        qy = 0.25 * scale
        qz = (r12 + r21) / scale
    else:
        scale = 2.0 * math.sqrt(max(0.0, 1.0 + r22 - r00 - r11))
        if scale <= 1e-12:
            raise ValueError(f"{source}: invalid rotation matrix")
        qw = (r10 - r01) / scale
        qx = (r02 + r20) / scale
        qy = (r12 + r21) / scale
        qz = 0.25 * scale

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError(f"{source}: invalid rotation quaternion")
    quaternion = (qx / norm, qy / norm, qz / norm, qw / norm)
    if quaternion[3] < 0.0:
        quaternion = tuple(-value for value in quaternion)
    return quaternion  # type: ignore[return-value]


def _default_ground_truth_path(sequence_root: Path) -> Path | None:
    candidates = (
        sequence_root / "poses.txt",
        sequence_root.parent.parent / "poses" / f"{sequence_root.name}.txt",
    )
    return next((candidate for candidate in candidates if candidate.is_file()), None)


def _load_ground_truth(
    path: Path, timestamps_ns: list[int]
) -> list[PoseSample]:
    samples: list[PoseSample] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            if len(fields) != 12:
                raise ValueError(
                    f"{path}:{line_number}: expected a 3x4 KITTI pose"
                )
            try:
                matrix = [float(field) for field in fields]
            except ValueError as error:
                raise ValueError(
                    f"{path}:{line_number}: invalid KITTI pose"
                ) from error
            if not all(math.isfinite(value) for value in matrix):
                raise ValueError(
                    f"{path}:{line_number}: KITTI pose must be finite"
                )
            index = len(samples)
            if index >= len(timestamps_ns):
                raise ValueError(f"{path}: more poses than image timestamps")
            rotation = [
                matrix[0],
                matrix[1],
                matrix[2],
                matrix[4],
                matrix[5],
                matrix[6],
                matrix[8],
                matrix[9],
                matrix[10],
            ]
            samples.append(
                PoseSample(
                    timestamp_ns=timestamps_ns[index],
                    position=(matrix[3], matrix[7], matrix[11]),
                    orientation_xyzw=_rotation_matrix_to_quaternion_xyzw(
                        rotation, source=f"{path}:{line_number}"
                    ),
                )
            )
    if len(samples) != len(timestamps_ns):
        raise ValueError(
            f"{path}: pose count {len(samples)} does not match image count "
            f"{len(timestamps_ns)}"
        )
    return samples


class KittiReader:
    """Lazy reader for ``times.txt`` plus ``image_0`` / ``image_1``."""

    def __init__(
        self,
        root: str | Path,
        *,
        load_images: bool = True,
        ground_truth_path: str | Path | None = None,
    ) -> None:
        self.seq_root = Path(root).resolve()
        if not self.seq_root.is_dir():
            raise FileNotFoundError(f"missing KITTI sequence: {self.seq_root}")
        self.left_directory = self.seq_root / "image_0"
        self.right_directory = self.seq_root / "image_1"
        if not self.left_directory.is_dir() or not self.right_directory.is_dir():
            raise FileNotFoundError(
                f"KITTI sequence requires image_0 and image_1: {self.seq_root}"
            )
        times_path = self.seq_root / "times.txt"
        if not times_path.is_file():
            raise FileNotFoundError(f"missing KITTI timestamps: {times_path}")

        self.frame_timestamps_ns: list[int] = []
        with times_path.open("r", encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, start=1):
                stripped = line.strip()
                if not stripped:
                    continue
                self.frame_timestamps_ns.append(
                    seconds_text_to_ns(
                        stripped, source=f"{times_path}:{line_number}"
                    )
                )
        validate_strictly_increasing(
            self.frame_timestamps_ns, source=str(times_path)
        )
        self.sensor_mode = "stereo"
        self.dataset_name = "kitti"
        self.load_images = bool(load_images)
        self.imu: list[ImuSample] = []
        if ground_truth_path is None:
            self.ground_truth_path = _default_ground_truth_path(self.seq_root)
        else:
            self.ground_truth_path = Path(ground_truth_path).expanduser().resolve()
            if not self.ground_truth_path.is_file():
                raise FileNotFoundError(
                    f"missing KITTI ground truth: {self.ground_truth_path}"
                )
        self.ground_truth = (
            _load_ground_truth(self.ground_truth_path, self.frame_timestamps_ns)
            if self.ground_truth_path is not None
            else []
        )

    def __len__(self) -> int:
        return len(self.frame_timestamps_ns)

    def info(self) -> dict:
        return {
            "dataset": self.dataset_name,
            "path": str(self.seq_root),
            "sensor_mode": self.sensor_mode,
            "frames": len(self),
            "imu_samples": 0,
            "ground_truth_poses": len(self.ground_truth),
            "ground_truth_file": (
                str(self.ground_truth_path)
                if self.ground_truth_path is not None
                else ""
            ),
            "duration_sec": duration_seconds(self.frame_timestamps_ns),
            "load_images": self.load_images,
        }

    def read(self, index: int) -> VisualFrame:
        if index < 0 or index >= len(self):
            raise IndexError(index)
        filename = f"{index:06d}.png"
        timestamp_ns = self.frame_timestamps_ns[index]
        left_path = self.left_directory / filename
        right_path = self.right_directory / filename
        return VisualFrame(
            index=index,
            primary=ImageSample(
                timestamp_ns=timestamp_ns,
                pixels=(
                    load_image(left_path, "mono8") if self.load_images else None
                ),
                encoding="mono8",
                frame_id="image_0",
                path=left_path,
            ),
            secondary=ImageSample(
                timestamp_ns=timestamp_ns,
                pixels=(
                    load_image(right_path, "mono8") if self.load_images else None
                ),
                encoding="mono8",
                frame_id="image_1",
                path=right_path,
            ),
            secondary_role="right",
            ground_truth=(
                self.ground_truth[index] if self.ground_truth else None
            ),
        )

    def iter_frames(
        self, *, start: int = 0, max_frames: int | None = None
    ) -> Iterator[VisualFrame]:
        for index in frame_indices(len(self), start=start, max_frames=max_frames):
            yield self.read(index)
