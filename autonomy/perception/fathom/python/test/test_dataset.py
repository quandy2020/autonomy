import json

import numpy as np
import pytest
import torch
from PIL import Image

from train.dataset import RgbdManifestDataset


def _write_manifest_sample(tmp_path):
    rgb = np.array(
        [
            [[0, 64, 255], [128, 32, 16], [255, 0, 128]],
            [[16, 32, 48], [64, 96, 128], [144, 160, 176]],
        ],
        dtype=np.uint8,
    )
    raw_depth = np.array([[1000, 0, 3000], [4000, 5000, 6000]], dtype=np.uint16)
    target_depth = np.array([[1100, 0, 3100], [0, 5100, 6100]], dtype=np.uint16)
    Image.fromarray(rgb).save(tmp_path / "rgb.png")
    Image.fromarray(raw_depth).save(tmp_path / "raw.png")
    Image.fromarray(target_depth).save(tmp_path / "target.png")
    (tmp_path / "manifest.jsonl").write_text(
        json.dumps(
            {
                "rgb": "rgb.png",
                "raw_depth": "raw.png",
                "target_depth": "target.png",
                "intrinsics": [[1, 0, 0.5], [0, 1, 0.5], [0, 0, 1]],
            }
        )
        + "\n"
    )
    return tmp_path / "manifest.jsonl"


def test_manifest_dataset_normalizes_rgb_scales_depth_and_marks_invalid_targets(tmp_path):
    dataset = RgbdManifestDataset(
        _write_manifest_sample(tmp_path), dropout_probability=0.0, depth_scale=0.001
    )

    sample = dataset[0]

    assert sample["image"].shape == (3, 2, 3)
    assert sample["image"].dtype == torch.float32
    assert sample["image"].is_contiguous()
    assert sample["image"].min().item() == 0.0
    assert sample["image"].max().item() == 1.0
    assert torch.allclose(sample["raw_depth"], torch.tensor([[1.0, 0.0, 3.0], [4.0, 5.0, 6.0]]))
    assert torch.allclose(sample["target_depth"], torch.tensor([[1.1, 0.0, 3.1], [0.0, 5.1, 6.1]]))
    assert torch.equal(
        sample["valid_mask"], torch.tensor([[True, False, True], [False, True, True]])
    )
    assert torch.equal(sample["intrinsics"], torch.tensor([[1.0, 0.0, 0.5], [0.0, 1.0, 0.5], [0.0, 0.0, 1.0]]))


def test_manifest_dataset_dropout_zeros_valid_raw_depth_without_changing_target(tmp_path):
    dataset = RgbdManifestDataset(
        _write_manifest_sample(tmp_path), dropout_probability=1.0, depth_scale=0.001
    )

    sample = dataset[0]

    assert torch.equal(sample["raw_depth"], torch.zeros((2, 3)))
    assert torch.allclose(sample["target_depth"], torch.tensor([[1.1, 0.0, 3.1], [0.0, 5.1, 6.1]]))


def test_manifest_dataset_dropout_is_reproducible_with_injected_generator(tmp_path):
    manifest = _write_manifest_sample(tmp_path)
    first = RgbdManifestDataset(
        manifest,
        dropout_probability=0.5,
        depth_scale=0.001,
        generator=torch.Generator().manual_seed(42),
    )[0]
    second = RgbdManifestDataset(
        manifest,
        dropout_probability=0.5,
        depth_scale=0.001,
        generator=torch.Generator().manual_seed(42),
    )[0]

    assert torch.equal(first["raw_depth"], second["raw_depth"])


@pytest.mark.parametrize(
    "dropout_probability",
    [-0.1, 1.1, float("nan"), float("inf"), float("-inf")],
)
def test_manifest_dataset_rejects_dropout_probability_outside_unit_interval(
    tmp_path, dropout_probability
):
    with pytest.raises(ValueError, match="dropout_probability"):
        RgbdManifestDataset(
            _write_manifest_sample(tmp_path),
            dropout_probability=dropout_probability,
            depth_scale=0.001,
        )
