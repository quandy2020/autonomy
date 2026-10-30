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

"""Shared types and image helpers for Atlas visual dataset readers."""

from __future__ import annotations

import bisect
import math
import sys
from collections.abc import Iterator, Sequence
from dataclasses import dataclass
from decimal import ROUND_HALF_UP, Decimal, InvalidOperation
from pathlib import Path
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class ImuSample:
    """One timestamped IMU measurement in SI units."""

    timestamp_ns: int
    angular_velocity: tuple[float, float, float]
    linear_acceleration: tuple[float, float, float]


@dataclass(frozen=True)
class ImageSample:
    """One image message payload and its original dataset path."""

    timestamp_ns: int
    pixels: np.ndarray | None
    encoding: str
    frame_id: str
    path: Path


@dataclass(frozen=True)
class PoseSample:
    """One timestamped ground-truth camera pose in a world frame."""

    timestamp_ns: int
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class VisualFrame:
    """One primary image and an optional synchronized right/depth image."""

    index: int
    primary: ImageSample
    secondary: ImageSample | None = None
    secondary_role: str | None = None
    ground_truth: PoseSample | None = None


class VisualDatasetReader(Protocol):
    """Structural interface consumed by the visual dataset publisher."""

    sensor_mode: str
    imu: list[ImuSample]
    frame_timestamps_ns: list[int]
    load_images: bool

    def __len__(self) -> int: ...

    def info(self) -> dict: ...

    def read(self, index: int) -> VisualFrame: ...

    def iter_frames(
        self, *, start: int = 0, max_frames: int | None = None
    ) -> Iterator[VisualFrame]: ...


@dataclass(frozen=True)
class TimestampedPath:
    """A timestamp/path pair used while indexing a sequence."""

    timestamp_ns: int
    path: Path


def nanoseconds_text_to_ns(text: str, *, source: str = "timestamp") -> int:
    """Parse an integer nanosecond timestamp without a float round trip."""

    stripped = text.strip()
    if not stripped:
        raise ValueError(f"{source}: empty nanosecond timestamp")
    try:
        value = int(stripped, 10)
    except ValueError as error:
        raise ValueError(f"{source}: invalid nanosecond timestamp {text!r}") from error
    return value


def seconds_text_to_ns(text: str, *, source: str = "timestamp") -> int:
    """Parse decimal seconds and round once to the nearest nanosecond."""

    stripped = text.strip()
    try:
        seconds = Decimal(stripped)
    except InvalidOperation as error:
        raise ValueError(f"{source}: invalid seconds timestamp {text!r}") from error
    if not seconds.is_finite():
        raise ValueError(f"{source}: timestamp must be finite")
    return int(
        (seconds * Decimal(1_000_000_000)).to_integral_value(
            rounding=ROUND_HALF_UP
        )
    )


def validate_strictly_increasing(
    timestamps_ns: Sequence[int], *, source: str
) -> None:
    if not timestamps_ns:
        raise ValueError(f"{source}: no timestamps found")
    for index in range(1, len(timestamps_ns)):
        if timestamps_ns[index] <= timestamps_ns[index - 1]:
            raise ValueError(
                f"{source}: timestamps must be strictly increasing at index {index}"
            )


def associate_nearest(
    primary: Sequence[TimestampedPath],
    secondary: Sequence[TimestampedPath],
    *,
    max_difference_ns: int,
) -> list[tuple[TimestampedPath, TimestampedPath]]:
    """Return deterministic one-to-one nearest-timestamp associations."""

    if max_difference_ns < 0:
        raise ValueError("max_difference_ns must be nonnegative")
    if not primary or not secondary:
        return []

    secondary_times = [sample.timestamp_ns for sample in secondary]
    candidates: list[tuple[int, int, int]] = []
    for primary_index, sample in enumerate(primary):
        begin = bisect.bisect_left(
            secondary_times, sample.timestamp_ns - max_difference_ns
        )
        end = bisect.bisect_right(
            secondary_times, sample.timestamp_ns + max_difference_ns
        )
        for secondary_index in range(begin, end):
            difference = abs(
                sample.timestamp_ns - secondary[secondary_index].timestamp_ns
            )
            candidates.append((difference, primary_index, secondary_index))

    candidates.sort()
    used_primary: set[int] = set()
    used_secondary: set[int] = set()
    matches: list[tuple[TimestampedPath, TimestampedPath]] = []
    for _, primary_index, secondary_index in candidates:
        if primary_index in used_primary or secondary_index in used_secondary:
            continue
        used_primary.add(primary_index)
        used_secondary.add(secondary_index)
        matches.append((primary[primary_index], secondary[secondary_index]))
    matches.sort(key=lambda pair: pair[0].timestamp_ns)
    return matches


def _load_with_opencv(path: Path, encoding: str) -> np.ndarray | None:
    try:
        import cv2
    except ImportError:
        return None

    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f"failed to decode image: {path}")
    if encoding == "rgb8":
        if image.ndim != 3 or image.shape[2] not in (3, 4):
            raise ValueError(f"expected a color image: {path}")
        conversion = cv2.COLOR_BGRA2RGB if image.shape[2] == 4 else cv2.COLOR_BGR2RGB
        image = cv2.cvtColor(image, conversion)
    elif encoding == "mono8":
        if image.ndim == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if image.ndim != 2 or image.dtype != np.uint8:
            raise ValueError(f"expected an 8-bit grayscale image: {path}")
    elif encoding == "mono16":
        if image.ndim != 2 or image.dtype != np.uint16:
            raise ValueError(f"expected a 16-bit single-channel image: {path}")
    else:
        raise ValueError(f"unsupported visual dataset encoding: {encoding}")
    return np.asarray(image)


def _load_with_pillow(path: Path, encoding: str) -> np.ndarray | None:
    try:
        from PIL import Image as PilImage
    except ImportError:
        return None

    with PilImage.open(path) as image:
        if encoding == "rgb8":
            pixels = np.asarray(image.convert("RGB"), dtype=np.uint8)
        elif encoding == "mono8":
            pixels = np.asarray(image.convert("L"), dtype=np.uint8)
        elif encoding == "mono16":
            pixels = np.asarray(image)
            if pixels.ndim != 2 or not np.issubdtype(pixels.dtype, np.integer):
                raise ValueError(f"expected a 16-bit single-channel image: {path}")
            if pixels.size and (int(pixels.min()) < 0 or int(pixels.max()) > 65535):
                raise ValueError(f"depth values exceed uint16 range: {path}")
            pixels = pixels.astype(np.uint16, copy=False)
        else:
            raise ValueError(f"unsupported visual dataset encoding: {encoding}")
    return pixels


def load_image(path: Path, encoding: str) -> np.ndarray:
    """Load an image with the byte layout advertised in AutoMsgs."""

    if not path.is_file():
        raise FileNotFoundError(f"missing image: {path}")
    # Prefer Pillow: the runtime image can contain an OpenCV extension built
    # against a different NumPy ABI, while Pillow preserves the required PNG
    # and JPEG pixel formats without importing that extension.
    pixels = _load_with_pillow(path, encoding)
    if pixels is None:
        pixels = _load_with_opencv(path, encoding)
    if pixels is None:
        raise ImportError("Pillow or OpenCV is required to load dataset images")
    if pixels.size == 0:
        raise ValueError(f"decoded image is empty: {path}")

    if pixels.dtype.itemsize > 1:
        big_endian = pixels.dtype.byteorder == ">" or (
            pixels.dtype.byteorder == "=" and sys.byteorder == "big"
        )
        if big_endian:
            little_dtype = pixels.dtype.newbyteorder("<")
            pixels = pixels.byteswap().view(little_dtype)
    if encoding == "rgb8" and pixels.dtype != np.uint8:
        raise ValueError(f"expected uint8 RGB pixels: {path}")
    return np.ascontiguousarray(pixels)


def frame_indices(
    length: int, *, start: int = 0, max_frames: int | None = None
) -> range:
    if start < 0:
        raise ValueError("start must be nonnegative")
    end = length
    if max_frames is not None:
        if max_frames < 0:
            raise ValueError("max_frames must be nonnegative")
        end = min(end, start + max_frames)
    return range(min(start, length), end)


def duration_seconds(timestamps_ns: Sequence[int]) -> float:
    if len(timestamps_ns) < 2:
        return 0.0
    return float(timestamps_ns[-1] - timestamps_ns[0]) * 1e-9


def finite_vector3(values: Sequence[str], *, source: str) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f"{source}: expected three values")
    try:
        parsed = tuple(float(value) for value in values)
    except ValueError as error:
        raise ValueError(f"{source}: invalid floating-point vector") from error
    if not all(math.isfinite(value) for value in parsed):
        raise ValueError(f"{source}: vector must be finite")
    return parsed  # type: ignore[return-value]
