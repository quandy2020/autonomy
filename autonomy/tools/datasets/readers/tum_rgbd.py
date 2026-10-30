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

"""TUM RGB-D reader with deterministic timestamp association."""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

from autonomy.tools.datasets.readers.visual import (
    ImageSample,
    ImuSample,
    TimestampedPath,
    VisualFrame,
    associate_nearest,
    duration_seconds,
    frame_indices,
    load_image,
    seconds_text_to_ns,
    validate_strictly_increasing,
)


def _read_timestamp_path_file(path: Path, root: Path) -> list[TimestampedPath]:
    if not path.is_file():
        raise FileNotFoundError(f"missing timestamp/path file: {path}")
    samples: list[TimestampedPath] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            if len(fields) < 2:
                raise ValueError(f"{path}:{line_number}: expected timestamp path")
            timestamp_ns = seconds_text_to_ns(
                fields[0], source=f"{path}:{line_number}"
            )
            samples.append(TimestampedPath(timestamp_ns, root / fields[1]))
    validate_strictly_increasing(
        [sample.timestamp_ns for sample in samples], source=str(path)
    )
    return samples


def _read_associations(
    path: Path, root: Path
) -> list[tuple[TimestampedPath, TimestampedPath]]:
    if not path.is_file():
        raise FileNotFoundError(f"missing TUM association file: {path}")
    pairs: list[tuple[TimestampedPath, TimestampedPath]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            if len(fields) < 4:
                raise ValueError(
                    f"{path}:{line_number}: expected rgb_time rgb_path "
                    "depth_time depth_path"
                )
            source = f"{path}:{line_number}"
            pairs.append(
                (
                    TimestampedPath(
                        seconds_text_to_ns(fields[0], source=source),
                        root / fields[1],
                    ),
                    TimestampedPath(
                        seconds_text_to_ns(fields[2], source=source),
                        root / fields[3],
                    ),
                )
            )
    if not pairs:
        raise ValueError(f"{path}: no RGB-D associations found")
    validate_strictly_increasing(
        [pair[0].timestamp_ns for pair in pairs], source=f"{path} RGB"
    )
    validate_strictly_increasing(
        [pair[1].timestamp_ns for pair in pairs], source=f"{path} depth"
    )
    return pairs


class TumRgbdReader:
    """Lazy reader for an associated TUM RGB-D sequence."""

    def __init__(
        self,
        root: str | Path,
        *,
        association_file: str | Path | None = None,
        load_images: bool = True,
        max_pair_dt_ns: int = 20_000_000,
    ) -> None:
        if max_pair_dt_ns < 0:
            raise ValueError("max_pair_dt_ns must be nonnegative")
        self.seq_root = Path(root).resolve()
        if not self.seq_root.is_dir():
            raise FileNotFoundError(f"missing TUM RGB-D sequence: {self.seq_root}")
        self.sensor_mode = "rgbd"
        self.dataset_name = "tum-rgbd"
        self.load_images = bool(load_images)
        self.max_pair_dt_ns = int(max_pair_dt_ns)
        self.imu: list[ImuSample] = []

        self.association_file: Path | None = None
        if association_file is not None:
            self.association_file = Path(association_file).resolve()
            self._pairs = _read_associations(
                self.association_file, self.seq_root
            )
        else:
            rgb = _read_timestamp_path_file(self.seq_root / "rgb.txt", self.seq_root)
            depth = _read_timestamp_path_file(
                self.seq_root / "depth.txt", self.seq_root
            )
            self._pairs = associate_nearest(
                rgb, depth, max_difference_ns=self.max_pair_dt_ns
            )
            if not self._pairs:
                raise ValueError(
                    "no RGB-D pairs satisfy the configured timestamp tolerance"
                )
        self.frame_timestamps_ns = [pair[0].timestamp_ns for pair in self._pairs]

    def __len__(self) -> int:
        return len(self._pairs)

    def info(self) -> dict:
        return {
            "dataset": self.dataset_name,
            "path": str(self.seq_root),
            "sensor_mode": self.sensor_mode,
            "frames": len(self),
            "imu_samples": 0,
            "duration_sec": duration_seconds(self.frame_timestamps_ns),
            "association_file": (
                str(self.association_file) if self.association_file else "auto"
            ),
            "max_pair_dt_ns": self.max_pair_dt_ns,
            "load_images": self.load_images,
        }

    def read(self, index: int) -> VisualFrame:
        if index < 0 or index >= len(self):
            raise IndexError(index)
        rgb, depth = self._pairs[index]
        return VisualFrame(
            index=index,
            primary=ImageSample(
                timestamp_ns=rgb.timestamp_ns,
                pixels=(load_image(rgb.path, "rgb8") if self.load_images else None),
                encoding="rgb8",
                frame_id="rgb",
                path=rgb.path,
            ),
            secondary=ImageSample(
                timestamp_ns=depth.timestamp_ns,
                pixels=(
                    load_image(depth.path, "mono16") if self.load_images else None
                ),
                encoding="mono16",
                frame_id="depth",
                path=depth.path,
            ),
            secondary_role="depth",
        )

    def iter_frames(
        self, *, start: int = 0, max_frames: int | None = None
    ) -> Iterator[VisualFrame]:
        for index in frame_indices(len(self), start=start, max_frames=max_frames):
            yield self.read(index)
