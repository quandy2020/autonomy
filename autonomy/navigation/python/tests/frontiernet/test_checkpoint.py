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

"""Checkpoint transplant tests: official encoder -> DeFM encoder swap."""

import pytest

torch = pytest.importorskip("torch")

from frontiernet.checkpoint import (  # noqa: E402
    load_frontiernet_heads,
    load_heads,
    read_state_dict,
    save_checkpoint,
)
from frontiernet.model import FrontierNetDeFM  # noqa: E402
from frontiernet.unet import TwoHeadUnet  # noqa: E402


@pytest.fixture
def official_checkpoint(tmp_path):
    model = TwoHeadUnet(classes=11, in_channels=4)
    path = tmp_path / "rgbd_11cls.pth"
    torch.save(model.state_dict(), path)
    return model, path


def test_read_state_dict_unwraps_nested_payload(tmp_path):
    model = TwoHeadUnet(classes=11, in_channels=4)
    path = tmp_path / "nested.pth"
    torch.save({"state_dict": model.state_dict(), "meta": {"epoch": 1}}, path)
    state = read_state_dict(str(path))
    assert "decoder.blocks.0.conv1.0.weight" in state
    assert all(isinstance(value, torch.Tensor) for value in state.values())


def test_heads_transplant_and_encoder_is_untouched(official_checkpoint, fake_backbone):
    official, path = official_checkpoint
    model = FrontierNetDeFM(classes=11, backbone=fake_backbone)
    before = {
        key: value.clone()
        for key, value in model.state_dict().items()
        if key.startswith("encoder.")
    }
    report = load_frontiernet_heads(model, str(path))
    assert report.ok, report.summary()
    expected = [key for key in official.state_dict() if not key.startswith("encoder.")]
    assert sorted(report.loaded) == sorted(expected)

    after = model.state_dict()
    for key, value in before.items():
        assert torch.equal(after[key], value), f"encoder tensor {key} was modified"
    assert torch.equal(
        model.decoder.blocks[0].conv1[0].weight,
        official.decoder.blocks[0].conv1[0].weight,
    )
    assert torch.equal(
        model.segmentation_head[6].weight, official.segmentation_head[6].weight
    )


def test_missing_head_tensor_is_reported(official_checkpoint, fake_backbone):
    _, path = official_checkpoint
    state = read_state_dict(str(path))
    state.pop("segmentation_head.6.weight")
    model = FrontierNetDeFM(classes=11, backbone=fake_backbone)
    with pytest.raises(RuntimeError):
        load_heads(model, state, strict=True)
    report = load_heads(model, state, strict=False)
    assert "segmentation_head.6.weight" in report.missing
    assert not report.ok


def test_encoder_tensors_are_skipped_not_rejected(official_checkpoint, fake_backbone):
    _, path = official_checkpoint
    state = read_state_dict(str(path))
    model = FrontierNetDeFM(classes=11, backbone=fake_backbone)
    report = load_heads(model, state, strict=True)
    assert any(key.startswith("encoder.") for key in report.skipped)
    assert report.ok


def test_save_checkpoint_roundtrip(tmp_path, fake_backbone):
    model = FrontierNetDeFM(classes=11, backbone=fake_backbone)
    path = tmp_path / "defm_heads.pth"
    save_checkpoint(model, str(path), meta={"stage": "skeleton"})
    payload = torch.load(path, map_location="cpu")
    assert payload["meta"]["stage"] == "skeleton"
    assert "encoder.adapter.projections.0.weight" in payload["state_dict"]
