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

"""Head decoding tests (numpy only)."""

import numpy as np
import pytest

from frontiernet.postprocess import (
    FrontierMap,
    cls2gainmap,
    cls_bin_edges,
    denormalize_df,
    df2region,
    logits_to_labels,
    prediction2frontiermap,
    region_iou,
)


def test_bin_edges_are_log_gain_values():
    edges = cls_bin_edges(11)
    assert edges.shape == (11,)
    assert edges[0] == pytest.approx(np.log1p(1))
    assert edges[-1] == pytest.approx(np.log1p(3500))
    with pytest.raises(ValueError):
        cls_bin_edges(5)


def test_gainmap_uses_bin_centres():
    edges = cls_bin_edges(11)
    labels = np.array([[0, 1, 10]], dtype=np.int16)
    gain = cls2gainmap(labels, 11)
    assert gain[0, 0] == 0.0
    assert gain[0, 1] == pytest.approx(np.expm1(0.5 * (edges[0] + edges[1])))
    assert gain[0, 2] == pytest.approx(np.expm1(0.5 * (edges[9] + edges[10])))


def test_distance_field_denormalization():
    df = np.array([[-np.log(0.1 / 10.0)]], dtype=np.float32)
    assert denormalize_df(df, 10.0)[0, 0] == pytest.approx(0.1, abs=1e-6)


def test_region_threshold_is_in_metres():
    near = np.array([[-np.log(0.1 / 10.0)]], dtype=np.float32)
    far = np.array([[-np.log(5.0 / 10.0)]], dtype=np.float32)
    assert df2region(near, 10.0, 0.5)[0, 0] == 1
    assert df2region(far, 10.0, 0.5)[0, 0] == 0


def test_logits_to_labels_accepts_batch_dimension():
    logits = np.zeros((1, 11, 4, 4), dtype=np.float32)
    logits[0, 3] = 5.0
    labels = logits_to_labels(logits)
    assert labels.shape == (4, 4)
    assert np.all(labels == 3)


def test_prediction2frontiermap_zeroes_gain_outside_region():
    df = np.full((1, 1, 8, 8), -np.log(0.1 / 10.0), dtype=np.float32)
    df[:, :, :, 4:] = -np.log(5.0 / 10.0)
    logits = np.zeros((1, 11, 8, 8), dtype=np.float32)
    logits[:, 5] = 1.0
    result = prediction2frontiermap(df, logits, n_classes=11)
    assert isinstance(result, FrontierMap)
    assert result.region.shape == (8, 8)
    assert np.all(result.gain[:, 4:] == 0.0)
    assert np.all(result.gain[:, :4] > 0.0)
    assert result.summary()["frontier_pixels"] == 32


def test_frontier_map_roundtrip(tmp_path):
    df = np.zeros((1, 1, 4, 4), dtype=np.float32)
    logits = np.zeros((1, 11, 4, 4), dtype=np.float32)
    result = prediction2frontiermap(df, logits, n_classes=11)
    path = tmp_path / "map.npz"
    result.save(str(path))
    restored = FrontierMap.load(str(path))
    assert np.array_equal(restored.region, result.region)
    assert np.allclose(restored.gain, result.gain)


def test_region_iou_edges():
    mask = np.array([[1, 0], [0, 1]], dtype=np.uint8)
    assert region_iou(mask, mask) == 1.0
    assert region_iou(mask, np.zeros_like(mask)) == 0.0
    assert region_iou(np.zeros_like(mask), np.zeros_like(mask)) == 1.0
