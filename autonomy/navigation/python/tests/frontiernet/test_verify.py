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

"""Acceptance-report tests for the three-way frame check (numpy only)."""

import numpy as np
import pytest

from frontiernet.postprocess import FrontierMap
from frontiernet.verify import compare_predictions, load_frame


def _map(df: np.ndarray, region: np.ndarray) -> FrontierMap:
    return FrontierMap(
        region=region.astype(np.uint8),
        gain=np.zeros_like(region, dtype=np.float32),
        labels=np.zeros_like(region, dtype=np.int16),
        df=df.astype(np.float32),
        logits=np.zeros((11,) + region.shape, dtype=np.float32),
    )


def _frame(pixels: int, df_max: float = 3.0):
    region = np.zeros((4, 4), dtype=np.uint8)
    region.flat[:pixels] = 1
    df = np.full((4, 4), df_max, dtype=np.float32)
    return _map(df, region)


def test_student_only_reports_pixels_and_ok():
    report = compare_predictions(_frame(3))
    assert report["student_pixels"] == 3
    assert report["ok"] is True
    assert "onnx_pixels" not in report


def test_empty_student_is_not_ok():
    report = compare_predictions(_frame(0))
    assert report["student_pixels"] == 0
    assert report["ok"] is False


def test_onnx_must_match_pixels_and_stay_within_tolerance():
    student = _frame(3)
    assert compare_predictions(student, _frame(3))["ok"] is True

    drifted = _frame(3)
    drifted.df[0, 0] += 5e-3
    report = compare_predictions(student, drifted)
    assert report["onnx_pixels_match"] is True
    assert report["onnx_df_max_abs_delta"] > 1e-3
    assert report["ok"] is False

    fewer = compare_predictions(student, _frame(2))
    assert fewer["onnx_pixels_match"] is False
    assert fewer["ok"] is False


def test_teacher_reports_iou_without_gating():
    student = _frame(3)
    teacher = _frame(2)
    report = compare_predictions(student, None, teacher)
    assert report["teacher_pixels"] == 2
    assert report["teacher_iou"] == pytest.approx(2.0 / 3.0)
    assert report["ok"] is True


def test_load_frame_reads_depth_and_rgb(tmp_path):
    path = tmp_path / "00000.npz"
    depth = np.full((2, 3), 1.5, dtype=np.float32)
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    np.savez(path, depth=depth, rgb=rgb)
    frame = load_frame(str(path))
    assert frame["depth"].dtype == np.float32
    assert frame["rgb"].shape == (2, 3, 3)


def test_load_frame_rejects_missing_depth(tmp_path):
    path = tmp_path / "bad.npz"
    np.savez(path, rgb=np.zeros((2, 3, 3), dtype=np.uint8))
    with pytest.raises(ValueError):
        load_frame(str(path))
