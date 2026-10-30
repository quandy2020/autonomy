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

"""Readers for EuRoC and the compatible TUM-VI ``mav0`` layout."""

from __future__ import annotations

import csv
from collections.abc import Iterator
from pathlib import Path

from autonomy.tools.datasets.readers.visual import (
    ImageSample,
    ImuSample,
    TimestampedPath,
    VisualFrame,
    associate_nearest,
    duration_seconds,
    finite_vector3,
    frame_indices,
    load_image,
    nanoseconds_text_to_ns,
    validate_strictly_increasing,
)

_EUROC_SENSOR_MODES = {
    "monocular",
    "stereo",
    "monocular_inertial",
    "stereo_inertial",
}


def _read_camera_csv(path: Path, image_directory: Path) -> list[TimestampedPath]:
    if not path.is_file():
        raise FileNotFoundError(f"missing camera CSV: {path}")
    samples: list[TimestampedPath] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for line_number, row in enumerate(csv.reader(stream), start=1):
            if not row or row[0].strip().startswith("#"):
                continue
            if len(row) < 2 or not row[1].strip():
                raise ValueError(f"{path}:{line_number}: expected timestamp,filename")
            timestamp_ns = nanoseconds_text_to_ns(
                row[0], source=f"{path}:{line_number}"
            )
            samples.append(
                TimestampedPath(timestamp_ns, image_directory / row[1].strip())
            )
    validate_strictly_increasing(
        [sample.timestamp_ns for sample in samples], source=str(path)
    )
    return samples


def _read_imu_csv(path: Path) -> list[ImuSample]:
    if not path.is_file():
        raise FileNotFoundError(f"missing IMU CSV: {path}")
    samples: list[ImuSample] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for line_number, row in enumerate(csv.reader(stream), start=1):
            if not row or row[0].strip().startswith("#"):
                continue
            if len(row) < 7:
                raise ValueError(
                    f"{path}:{line_number}: expected timestamp,gyro_xyz,accel_xyz"
                )
            source = f"{path}:{line_number}"
            samples.append(
                ImuSample(
                    timestamp_ns=nanoseconds_text_to_ns(row[0], source=source),
                    angular_velocity=finite_vector3(row[1:4], source=source),
                    linear_acceleration=finite_vector3(row[4:7], source=source),
                )
            )
    validate_strictly_increasing(
        [sample.timestamp_ns for sample in samples], source=str(path)
    )
    return samples


class EurocReader:
    """Lazy image reader for EuRoC camera and optional IMU streams."""

    def __init__(
        self,
        root: str | Path,
        *,
        sensor_mode: str = "monocular",
        load_images: bool = True,
        max_stereo_dt_ns: int = 10_000_000,
    ) -> None:
        if sensor_mode not in _EUROC_SENSOR_MODES:
            raise ValueError(
                f"unsupported EuRoC sensor mode {sensor_mode!r}; "
                f"expected one of {sorted(_EUROC_SENSOR_MODES)}"
            )
        if max_stereo_dt_ns < 0:
            raise ValueError("max_stereo_dt_ns must be nonnegative")
        self.seq_root = Path(root).resolve()
        self.sensor_mode = sensor_mode
        self.load_images = bool(load_images)
        self.max_stereo_dt_ns = int(max_stereo_dt_ns)
        self.dataset_name = "euroc"

        mav0 = self.seq_root / "mav0"
        if not mav0.is_dir():
            raise FileNotFoundError(f"missing mav0 directory: {mav0}")
        left = _read_camera_csv(
            mav0 / "cam0" / "data.csv", mav0 / "cam0" / "data"
        )
        self._pairs: list[tuple[TimestampedPath, TimestampedPath | None]] = []
        if sensor_mode in ("stereo", "stereo_inertial"):
            right = _read_camera_csv(
                mav0 / "cam1" / "data.csv", mav0 / "cam1" / "data"
            )
            matches = associate_nearest(
                left, right, max_difference_ns=self.max_stereo_dt_ns
            )
            if len(matches) != len(left) or len(matches) != len(right):
                raise ValueError(
                    "camera streams are not fully one-to-one synchronized: "
                    f"cam0={len(left)} cam1={len(right)} matched={len(matches)}"
                )
            self._pairs = [(primary, secondary) for primary, secondary in matches]
        else:
            self._pairs = [(primary, None) for primary in left]

        self.frame_timestamps_ns = [pair[0].timestamp_ns for pair in self._pairs]
        self.imu: list[ImuSample] = []
        if sensor_mode in ("monocular_inertial", "stereo_inertial"):
            self.imu = _read_imu_csv(mav0 / "imu0" / "data.csv")

    def __len__(self) -> int:
        return len(self._pairs)

    def info(self) -> dict:
        return {
            "dataset": self.dataset_name,
            "path": str(self.seq_root),
            "sensor_mode": self.sensor_mode,
            "frames": len(self),
            "imu_samples": len(self.imu),
            "duration_sec": duration_seconds(self.frame_timestamps_ns),
            "load_images": self.load_images,
        }

    def _image_sample(
        self, sample: TimestampedPath, *, frame_id: str
    ) -> ImageSample:
        pixels = load_image(sample.path, "mono8") if self.load_images else None
        return ImageSample(
            timestamp_ns=sample.timestamp_ns,
            pixels=pixels,
            encoding="mono8",
            frame_id=frame_id,
            path=sample.path,
        )

    def read(self, index: int) -> VisualFrame:
        if index < 0 or index >= len(self):
            raise IndexError(index)
        primary, secondary = self._pairs[index]
        primary_image = self._image_sample(primary, frame_id="cam0")
        if secondary is None:
            return VisualFrame(index=index, primary=primary_image)
        return VisualFrame(
            index=index,
            primary=primary_image,
            secondary=self._image_sample(secondary, frame_id="cam1"),
            secondary_role="right",
        )

    def iter_frames(
        self, *, start: int = 0, max_frames: int | None = None
    ) -> Iterator[VisualFrame]:
        for index in frame_indices(len(self), start=start, max_frames=max_frames):
            yield self.read(index)


class TumViReader(EurocReader):
    """TUM-VI 512-pixel stereo-inertial reader (EuRoC-compatible layout)."""

    def __init__(
        self,
        root: str | Path,
        *,
        load_images: bool = True,
        max_stereo_dt_ns: int = 10_000_000,
    ) -> None:
        super().__init__(
            root,
            sensor_mode="stereo_inertial",
            load_images=load_images,
            max_stereo_dt_ns=max_stereo_dt_ns,
        )
        self.dataset_name = "tum-vi"
