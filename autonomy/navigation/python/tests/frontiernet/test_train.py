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

"""Retraining pipeline tests: loss, dataset, freeze policy, one real step."""

import json
import math
import os

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from frontiernet.dataset import FrontierFrameDataset, split_indices  # noqa: E402
from frontiernet.preprocess import (  # noqa: E402
    defm_preprocess,
    frontiernet_preprocess,
    frontiernet_preprocess_aligned,
    resize,
)
from frontiernet.loss import (  # noqa: E402
    FrontierLoss,
    LossWeights,
    class_weights_from_histogram,
    df_band_mask,
    df_to_metres,
)
from frontiernet.train import TrainSettings, train  # noqa: E402


def _write_frame(root, name, size=64, seed=0):
    rng = np.random.default_rng(seed)
    frames = os.path.join(root, "frames")
    labels = os.path.join(root, "labels")
    os.makedirs(frames, exist_ok=True)
    os.makedirs(labels, exist_ok=True)
    depth = rng.uniform(0.5, 4.0, size=(size, size)).astype(np.float32)
    rgb = rng.integers(0, 255, size=(size, size, 3), dtype=np.uint8)
    np.savez_compressed(
        os.path.join(frames, f"{name}.npz"),
        depth=depth,
        rgb=rgb,
        K=np.eye(3).reshape(-1),
        stamp=np.float64(0.0),
    )
    df = rng.uniform(0.0, 8.0, size=(size, size)).astype(np.float16)
    df[: size // 4, : size // 4] = 12.0  # a frontier patch
    classes = np.zeros((size, size), dtype=np.uint8)
    classes[size // 4 :, size // 4 :] = 1
    np.savez_compressed(os.path.join(labels, f"{name}.npz"), df=df, labels=classes)
    return frames, labels


def test_df_helpers_match_the_postprocess_contract():
    df = torch.tensor([[[0.0, math.log(10.0), 4.0]]])
    metres = df_to_metres(df)
    assert metres[0, 0, 0].item() == pytest.approx(10.0, rel=1e-5)
    assert metres[0, 0, 1].item() == pytest.approx(1.0, rel=1e-5)
    # band_m=1.0 keeps targets closer than 1 m, i.e. df > log(10) = 2.303
    band = df_band_mask(df, band_m=1.0)
    assert band.tolist() == [[[False, False, True]]]
    assert df_band_mask(torch.tensor([[[2.4]]]), band_m=1.0).item() is True


def test_class_weights_penalise_the_majority_class():
    weights = class_weights_from_histogram([100000, 1000, 10], power=0.5)
    assert weights.shape == (3,)
    assert weights[0] < weights[1] < weights[2]
    assert float(weights.mean()) == pytest.approx(1.0, rel=0.25)


def test_loss_is_near_zero_for_a_perfect_prediction():
    torch.manual_seed(0)
    criterion = FrontierLoss(LossWeights(label_smoothing=0.0, frontier_boost=0.0))
    df = torch.rand(2, 1, 32, 32)
    labels = torch.randint(0, 11, (2, 32, 32))
    logits = torch.full((2, 11, 32, 32), -20.0)
    logits.scatter_(1, labels.unsqueeze(1), 20.0)
    output = criterion(df, logits, df, labels)
    assert float(output["df"]) == pytest.approx(0.0, abs=1e-6)
    assert float(output["seg"]) == pytest.approx(0.0, abs=1e-4)
    assert float(output["metrics"]["label_acc"]) == pytest.approx(1.0)


def test_frontier_boost_increases_the_regression_loss():
    torch.manual_seed(0)
    df_target = torch.full((1, 1, 16, 16), 12.0)
    df_pred = torch.zeros_like(df_target)
    labels = torch.zeros((1, 16, 16), dtype=torch.long)
    logits = torch.zeros((1, 11, 16, 16))
    plain = FrontierLoss(LossWeights(frontier_boost=0.0))
    boosted = FrontierLoss(LossWeights(frontier_boost=4.0))
    assert float(boosted(df_pred, logits, df_target, labels)["df"]) > float(
        plain(df_pred, logits, df_target, labels)["df"]
    )


def test_split_indices_keeps_a_validation_frame():
    train_idx, val_idx = split_indices(10, 0.2, seed=3)
    assert len(val_idx) == 2
    assert not set(train_idx) & set(val_idx)
    assert sorted(train_idx + val_idx) == list(range(10))
    assert split_indices(1, 0.2) == ([0], [])


def test_dataset_pairs_frames_with_labels(tmp_path):
    frames, labels = _write_frame(str(tmp_path), "00000")
    _write_frame(str(tmp_path), "00001", seed=1)
    dataset = FrontierFrameDataset(frames, labels, input_size=64)
    assert len(dataset) == 2
    sample = dataset[0]
    assert tuple(sample["input"].shape) == (3, 64, 64)
    assert tuple(sample["df"].shape) == (1, 64, 64)
    assert tuple(sample["label"].shape) == (64, 64)
    histogram = dataset.label_histogram(11)
    assert int(histogram.sum()) == 2 * 64 * 64


def test_dataset_rejects_missing_labels(tmp_path):
    frames, _ = _write_frame(str(tmp_path), "00000")
    empty = tmp_path / "empty_labels"
    empty.mkdir()
    with pytest.raises(FileNotFoundError):
        FrontierFrameDataset(frames, str(empty), input_size=64)


def test_train_step_updates_adapter_but_not_the_frozen_backbone(tmp_path, fake_backbone):
    from frontiernet.model import FrontierNetDeFM

    for index in range(6):
        frames, labels = _write_frame(str(tmp_path), f"{index:05d}", seed=index)
    model = FrontierNetDeFM(classes=11, input_size=64, backbone=fake_backbone)
    backbone_before = [p.detach().clone() for p in model.backbone_parameters()]
    adapter_before = [p.detach().clone() for p in model.adapter_parameters()]

    settings = TrainSettings(
        frames=frames,
        labels=labels,
        out=str(tmp_path / "run"),
        official_weights="unused.pth",
        epochs=1,
        batch_size=2,
        input_size=64,
        device="cpu",
        amp=False,
        workers=0,
        max_steps=2,
        log_every=1,
        val_fraction=0.34,
    )
    summary = train(settings, model=model)

    assert summary["steps"] == 2
    assert summary["frames_train"] == 4
    assert summary["frames_val"] == 2
    assert summary["head_transplant"] == "injected"
    for before, after in zip(backbone_before, model.backbone_parameters()):
        assert torch.equal(before, after.detach())
    assert any(
        not torch.equal(before, after.detach())
        for before, after in zip(adapter_before, model.adapter_parameters())
    )
    assert os.path.isfile(os.path.join(settings.out, "last.pth"))
    assert os.path.isfile(os.path.join(settings.out, "summary.json"))
    with open(os.path.join(settings.out, "summary.json"), encoding="utf-8") as handle:
        payload = json.load(handle)
    assert payload["steps"] == 2
    assert payload["parameters"]["backbone_trainable"] == 0
    assert len(payload["class_histogram"]) == 11
    with open(os.path.join(settings.out, "train_log.jsonl"), encoding="utf-8") as handle:
        events = [json.loads(line) for line in handle if line.strip()]
    assert any(event["event"] == "step" for event in events)
    assert any(event["event"] == "epoch" for event in events)


def test_aligned_preprocess_uses_the_defm_pixel_grid():
    rng = np.random.default_rng(0)
    depth = rng.uniform(0.5, 4.0, size=(120, 160)).astype(np.float32)
    rgb = rng.integers(0, 255, size=(120, 160, 3), dtype=np.uint8)
    aligned = frontiernet_preprocess_aligned(rgb, depth, size=224)
    assert aligned.shape == (4, 224, 224)
    assert aligned.dtype == np.float32
    assert 0.0 <= float(aligned[:3].min()) and float(aligned[:3].max()) <= 1.0
    assert 0.0 <= float(aligned[3].min()) and float(aligned[3].max()) <= 1.0
    # the depth channel must be exactly the DeFM resize, not the letterbox crop
    expected = resize(depth.astype(np.float64), 224)
    expected = (expected - expected.min()) / (expected.max() - expected.min() + 1e-6)
    assert np.allclose(aligned[3], expected.astype(np.float32), atol=1e-6)


def test_aligned_and_official_geometry_disagree_by_design():
    rng = np.random.default_rng(1)
    depth = rng.uniform(0.5, 4.0, size=(480, 640)).astype(np.float32)
    rgb = rng.integers(0, 255, size=(480, 640, 3), dtype=np.uint8)
    aligned = frontiernet_preprocess_aligned(rgb, depth, size=224)
    official = frontiernet_preprocess(rgb / 255.0, depth, scale=0.5, size=224)
    assert aligned.shape == official.shape == (4, 224, 224)
    # a 640x480 frame stretched to 224 differs from the 448x448 centre crop
    assert not np.allclose(aligned[3], official[3], atol=1e-3)


def test_defm_and_aligned_share_the_spatial_transform():
    """A step edge must land on the same column in both depth channels."""
    depth = np.full((480, 640), 1.0, dtype=np.float32)
    depth[:, 320:] = 5.0
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    aligned = frontiernet_preprocess_aligned(rgb, depth, size=224)
    student = defm_preprocess(depth, 224)

    def edge_column(channel):
        gradient = np.abs(np.diff(channel[112, :]))
        return int(np.argmax(gradient))

    assert edge_column(aligned[3]) == edge_column(student[2])
    # 320/640 of the way across a 224-wide image
    assert abs(edge_column(aligned[3]) - 112) <= 1


def test_resolve_weights_finds_the_conf_directory():
    from frontiernet.checkpoint import resolve_weights

    resolved = resolve_weights("conf/models/rgbd_11cls.pth")
    assert resolved.endswith(os.path.join("conf", "models", "rgbd_11cls.pth"))
    assert os.path.isfile(resolved)
