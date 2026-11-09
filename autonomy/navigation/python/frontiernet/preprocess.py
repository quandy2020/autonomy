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

"""Input preprocessing for the two model families this package supports.

Two preprocessing contracts live here and they are deliberately kept apart:

``defm_preprocess``
    Metric-aware log-depth stacking used by DeFM and by the C++
    ``navigation::perception::DepthPreprocessor``. It is absolute (a 3 m wall
    always maps to the same value) and it resizes with a separable triangle
    filter when shrinking and bilinear when growing.

``frontiernet_preprocess``
    The official FrontierNet contract: RGB/depth are scaled, center cropped and
    the depth channel is min-max normalized *per image*. It is relative, so the
    same wall maps differently depending on the closest/farthest pixel.

Keeping both explicit is what makes the "official checkpoint baseline" and the
"DeFM encoder" runs comparable instead of silently mixing contracts.
"""

from __future__ import annotations

from typing import Optional, Sequence, Tuple

import numpy as np

DEFM_MEAN: Tuple[float, float, float] = (0.248880, 0.495620, 0.492858)
DEFM_STD: Tuple[float, float, float] = (0.139357, 0.271314, 0.297177)
DEFM_MAX_DEPTH_C1 = 100.0
DEFM_MAX_DEPTH_C2 = 9.0
DEFM_INPUT_SIZE = 224

__all__ = [
    "DEFM_INPUT_SIZE",
    "DEFM_MAX_DEPTH_C1",
    "DEFM_MAX_DEPTH_C2",
    "DEFM_MEAN",
    "DEFM_STD",
    "defm_depth_channels",
    "defm_preprocess",
    "frontiernet_preprocess",
    "frontiernet_preprocess_aligned",
    "resize",
    "resize_center_crop",
]


def _axis_filter(source: int, target: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build the 1-D separable filter used by the C++ depth preprocessor.

    Returns ``(indices, weights, taps)`` with shape ``(target, max_taps)`` for
    the first two arrays and ``(target,)`` for the tap count.
    """
    if source <= 0 or target <= 0:
        raise ValueError("source and target sizes must be positive")
    scale = float(source) / float(target)
    radius = max(1.0, scale)
    max_taps = int(np.ceil(2.0 * radius)) + 2
    indices = np.zeros((target, max_taps), dtype=np.int32)
    weights = np.zeros((target, max_taps), dtype=np.float64)
    taps = np.zeros(target, dtype=np.int32)
    for out in range(target):
        center = (out + 0.5) * scale - 0.5
        begin = int(np.floor(center - radius))
        end = int(np.ceil(center + radius))
        local_indices = []
        local_weights = []
        total = 0.0
        for index in range(begin, end + 1):
            weight = max(0.0, 1.0 - abs(index - center) / radius)
            if weight <= 0.0:
                continue
            local_indices.append(min(max(index, 0), source - 1))
            local_weights.append(weight)
            total += weight
        if total <= 0.0:
            local_indices = [min(max(int(round(center)), 0), source - 1)]
            local_weights = [1.0]
            total = 1.0
        count = len(local_indices)
        indices[out, :count] = local_indices
        weights[out, :count] = np.asarray(local_weights, dtype=np.float64) / total
        taps[out] = count
    return indices, weights, taps


def _resize_axis(
    image: np.ndarray,
    indices: np.ndarray,
    weights: np.ndarray,
    taps: np.ndarray,
    axis: int,
) -> np.ndarray:
    moved = np.moveaxis(np.asarray(image, dtype=np.float64), axis, 0)
    out = np.zeros((indices.shape[0],) + moved.shape[1:], dtype=np.float64)
    tail = (1,) * (moved.ndim - 1)
    for tap in range(indices.shape[1]):
        weight = weights[:, tap].reshape((-1,) + tail)
        out += moved[indices[:, tap]] * weight
    return np.moveaxis(out, 0, axis)


def resize(image: np.ndarray, size: int) -> np.ndarray:
    """Resize a 2-D image to ``size x size`` with the C++ filter contract."""
    array = np.asarray(image, dtype=np.float64)
    if array.ndim != 2:
        raise ValueError(f"expected a 2-D image, got shape {array.shape}")
    height, width = array.shape
    if height == size and width == size:
        return array.astype(np.float32)
    y_idx, y_w, y_taps = _axis_filter(height, size)
    x_idx, x_w, x_taps = _axis_filter(width, size)
    out = _resize_axis(array, y_idx, y_w, y_taps, axis=0)
    out = _resize_axis(out, x_idx, x_w, x_taps, axis=1)
    return out.astype(np.float32)


def defm_depth_channels(
    depth: np.ndarray,
    *,
    max_depth_c1: float = DEFM_MAX_DEPTH_C1,
    max_depth_c2: float = DEFM_MAX_DEPTH_C2,
) -> np.ndarray:
    """Stack the three DeFM log-depth channels ``(3, H, W)`` in ``[0, 1]``.

    Mirrors ``DepthPreprocessor::Process``: non-finite values become 0, depths
    are clamped to ``[0, max_depth_c1]`` and the C3 min/max are taken over the
    whole frame (invalid pixels included), so a fully invalid frame yields a
    constant C3 instead of a division by zero.
    """
    array = np.asarray(depth, dtype=np.float64)
    if array.ndim != 2:
        raise ValueError(f"expected a 2-D depth image, got shape {array.shape}")
    array = np.where(np.isfinite(array), array, 0.0)
    array = np.clip(array, 0.0, float(max_depth_c1))
    log_depth = np.log1p(array)
    min_log = float(log_depth.min())
    max_log = float(log_depth.max())
    c1 = log_depth / np.log1p(float(max_depth_c1))
    c2 = np.clip(log_depth / np.log1p(float(max_depth_c2)), 0.0, 1.0)
    if max_log > min_log:
        c3 = (log_depth - min_log) / (max_log - min_log)
    else:
        c3 = np.zeros_like(log_depth)
    return np.stack([c1, c2, c3], axis=0).astype(np.float32)


def defm_preprocess(
    depth: np.ndarray,
    size: int = DEFM_INPUT_SIZE,
    mean: Sequence[float] = DEFM_MEAN,
    std: Sequence[float] = DEFM_STD,
) -> np.ndarray:
    """Return the ``(3, size, size)`` DeFM input for a metric depth frame."""
    channels = defm_depth_channels(depth)
    out = np.empty((3, size, size), dtype=np.float32)
    for index in range(3):
        out[index] = (resize(channels[index], size) - float(mean[index])) / float(std[index])
    return out


def resize_center_crop(image: np.ndarray, scale: float, size: int) -> np.ndarray:
    """Official FrontierNet resize-then-center-crop, including zero padding."""
    from PIL import Image

    array = np.asarray(image)
    if array.dtype != np.uint8:
        array = np.clip(array, 0, 255).astype(np.uint8)
    pil = Image.fromarray(array)
    resized = pil.resize(
        (max(1, int(pil.width * scale)), max(1, int(pil.height * scale))),
        Image.BICUBIC,
    )
    canvas = Image.new(resized.mode, (size, size), 0)
    width, height = resized.size
    crop_width = min(size, width)
    crop_height = min(size, height)
    left = max(0, (width - size) // 2)
    top = max(0, (height - size) // 2)
    paste_x = max(0, (size - width) // 2)
    paste_y = max(0, (size - height) // 2)
    canvas.paste(
        resized.crop((left, top, left + crop_width, top + crop_height)),
        (paste_x, paste_y),
    )
    return np.asarray(canvas)


def _frontiernet_channel(
    image: np.ndarray,
    scale: float,
    size: int,
    *,
    is_depth: bool,
    normalize_depth: bool,
) -> np.ndarray:
    cropped = resize_center_crop(image, scale, size).astype(np.float32)
    if cropped.ndim == 2:
        cropped = cropped[np.newaxis, ...]
    else:
        cropped = cropped.transpose((2, 0, 1))
    if is_depth:
        cropped = cropped[0:1, ...]
        if normalize_depth:
            low = float(cropped.min())
            high = float(cropped.max())
            cropped = (cropped - low) / (high - low + 1e-6)
    elif (cropped > 1.0).any():
        cropped = cropped / 255.0
    return cropped.astype(np.float32)


def frontiernet_preprocess(
    rgb: np.ndarray,
    depth: Optional[np.ndarray] = None,
    *,
    scale: float = 0.5,
    size: int = 224,
    normalize_depth: bool = True,
) -> np.ndarray:
    """Return the official ``(4, size, size)`` RGB-D input (or ``(3, ...)``)."""
    channels = [_frontiernet_channel(rgb, scale, size, is_depth=False, normalize_depth=False)]
    if depth is not None:
        channels.append(
            _frontiernet_channel(
                depth, scale, size, is_depth=True, normalize_depth=normalize_depth
            )
        )
    return np.concatenate(channels, axis=0)


def frontiernet_preprocess_aligned(
    rgb: np.ndarray,
    depth: Optional[np.ndarray] = None,
    *,
    size: int = 224,
    normalize_depth: bool = True,
) -> np.ndarray:
    """Official 4-channel input on the *DeFM* pixel grid.

    ``frontiernet_preprocess`` reproduces the upstream letterbox (scale 0.5,
    then a 224x224 centre crop), which covers only the middle 448x448 pixels of
    a 640x480 frame. The DeFM branch instead stretches the whole frame to
    ``size x size``. Teacher labels therefore have to be produced on the same
    grid as the student input, otherwise the two disagree by a fixed warp and
    the adapter can never fit.

    This variant applies the identical ``resize`` used by
    :func:`defm_preprocess` to the colour and depth channels, so a teacher
    prediction lands on exactly the pixels the student sees.
    """
    colour = np.asarray(rgb, dtype=np.float64)
    if colour.ndim != 3 or colour.shape[2] != 3:
        raise ValueError(f"expected an (H, W, 3) rgb image, got {colour.shape}")
    if colour.max() > 1.0:
        colour = colour / 255.0
    channels = [resize(colour[:, :, index], size)[None, ...] for index in range(3)]
    if depth is not None:
        metric = resize(np.asarray(depth, dtype=np.float64), size)
        if normalize_depth:
            low = float(metric.min())
            high = float(metric.max())
            metric = (metric - low) / (high - low + 1e-6)
        channels.append(metric[None, ...])
    return np.concatenate(channels, axis=0).astype(np.float32)
