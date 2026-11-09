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

"""Preprocessing contract tests (numpy only)."""

import numpy as np
import pytest

from frontiernet.preprocess import (
    DEFM_MEAN,
    DEFM_STD,
    defm_depth_channels,
    defm_preprocess,
    resize,
)


def test_defm_channels_are_bounded_and_monotonic():
    depth = np.array([[0.0, 1.0], [5.0, 40.0]], dtype=np.float32)
    channels = defm_depth_channels(depth)
    assert channels.shape == (3, 2, 2)
    assert channels.dtype == np.float32
    assert np.all(channels >= 0.0)
    assert np.all(channels <= 1.0)
    # farther pixels must be brighter in every channel
    assert channels[0, 1, 1] > channels[0, 0, 1]
    assert channels[1, 1, 1] > channels[1, 0, 1]


def test_defm_channels_handle_invalid_and_constant_frames():
    invalid = np.full((4, 4), np.nan, dtype=np.float32)
    channels = defm_depth_channels(invalid)
    assert np.allclose(channels, 0.0)

    constant = np.full((4, 4), 3.0, dtype=np.float32)
    channels = defm_depth_channels(constant)
    assert np.allclose(channels[2], 0.0)  # C3 has no range -> zeros


def test_defm_channels_clamp_far_depth():
    depth = np.array([[500.0]], dtype=np.float32)
    channels = defm_depth_channels(depth)
    assert channels[0, 0, 0] == pytest.approx(1.0, abs=1e-6)


def test_resize_preserves_constant_images():
    image = np.full((37, 91), 0.25, dtype=np.float32)
    assert np.allclose(resize(image, 224), 0.25, atol=1e-6)
    assert np.allclose(resize(image, 16), 0.25, atol=1e-6)


def test_resize_matches_target_shape():
    image = np.random.default_rng(0).random((120, 160)).astype(np.float32)
    assert resize(image, 224).shape == (224, 224)


def test_defm_preprocess_normalizes_with_official_stats():
    depth = np.linspace(0.5, 20.0, 64 * 48, dtype=np.float32).reshape(64, 48)
    tensor = defm_preprocess(depth, 224)
    assert tensor.shape == (3, 224, 224)
    assert tensor.dtype == np.float32
    # channel 0 must be the C1 map standardized with the published mean/std
    raw = resize(defm_depth_channels(depth)[0], 224)
    expected = (raw - DEFM_MEAN[0]) / DEFM_STD[0]
    assert np.allclose(tensor[0], expected, atol=1e-5)


def test_frontiernet_preprocess_matches_official_contract():
    pytest.importorskip("PIL")
    from frontiernet.preprocess import frontiernet_preprocess

    rgb = np.full((240, 320, 3), 128, dtype=np.uint8)
    depth = np.linspace(0.5, 5.0, 240 * 320, dtype=np.float32).reshape(240, 320)
    tensor = frontiernet_preprocess(rgb, depth, scale=1.0, size=224)
    assert tensor.shape == (4, 224, 224)
    assert tensor.dtype == np.float32
    assert np.allclose(tensor[0], 128.0 / 255.0, atol=1e-6)
    assert 0.0 <= float(tensor[3].min()) and float(tensor[3].max()) <= 1.0


def test_frontiernet_preprocess_pads_when_scaled_image_is_smaller():
    pytest.importorskip("PIL")
    from frontiernet.preprocess import frontiernet_preprocess

    rgb = np.full((240, 320, 3), 128, dtype=np.uint8)
    depth = np.full((240, 320), 2.0, dtype=np.float32)
    tensor = frontiernet_preprocess(rgb, depth, scale=0.25, size=224)
    # 320x240 * 0.25 = 80x60, so the official center crop zero-pads the border
    assert tensor.shape == (4, 224, 224)
    assert tensor[0, 0, 0] == 0.0
    assert tensor[0, 112, 112] == pytest.approx(128.0 / 255.0, abs=1e-6)
