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

"""DeFM encoder adapter tests using a stand-in backbone (no DeFM download)."""

import pytest

torch = pytest.importorskip("torch")

from frontiernet.encoder_defm import DefmEncoder, ViTToUnetAdapter  # noqa: E402
from frontiernet.model import FrontierNetDeFM  # noqa: E402
from frontiernet.unet import UnetDecoder  # noqa: E402


def test_encoder_returns_six_features_like_resnet(fake_backbone):
    encoder = DefmEncoder(backbone=fake_backbone)
    features = encoder(torch.zeros(1, 3, 224, 224))
    assert [tuple(feature.shape) for feature in features] == [
        (1, 3, 224, 224),
        (1, 64, 112, 112),
        (1, 64, 56, 56),
        (1, 128, 28, 28),
        (1, 256, 14, 14),
        (1, 512, 7, 7),
    ]
    assert encoder.out_channels == [3, 64, 64, 128, 256, 512]
    assert encoder.output_stride == 32


def test_adapter_rejects_wrong_level_count():
    adapter = ViTToUnetAdapter()
    with pytest.raises(ValueError):
        adapter([torch.zeros(1, 384, 16, 16)])


def test_adapter_target_sizes_follow_input_resolution():
    adapter = ViTToUnetAdapter()
    assert adapter.target_sizes() == [(112, 112), (56, 56), (28, 28), (14, 14), (7, 7)]
    assert adapter.target_sizes((224, 256)) == [
        (112, 128),
        (56, 64),
        (28, 32),
        (14, 16),
        (7, 8),
    ]


def test_official_decoder_accepts_defm_features(fake_backbone):
    encoder = DefmEncoder(backbone=fake_backbone)
    decoder = UnetDecoder(encoder_channels=encoder.out_channels)
    with torch.no_grad():
        decoded = decoder(*encoder(torch.zeros(1, 3, 224, 224)))
    assert tuple(decoded.shape) == (1, 16, 224, 224)


def test_frontiernet_defm_output_shapes(fake_backbone):
    model = FrontierNetDeFM(classes=11, backbone=fake_backbone)
    with torch.no_grad():
        df, masks = model(torch.zeros(1, 3, 224, 224))
    assert tuple(df.shape) == (1, 1, 224, 224)
    assert tuple(masks.shape) == (1, 11, 224, 224)
    assert model.n_channels == 3


def test_frontiernet_defm_pads_non_divisible_input(fake_backbone):
    model = FrontierNetDeFM(classes=11, backbone=fake_backbone)
    with torch.no_grad():
        df, masks = model(torch.zeros(1, 3, 200, 232))
    assert tuple(df.shape) == (1, 1, 200, 232)
    assert tuple(masks.shape) == (1, 11, 200, 232)


def test_backbone_freeze_keeps_adapter_trainable(fake_backbone):
    model = FrontierNetDeFM(classes=11, backbone=fake_backbone, freeze_backbone=True)
    assert model.encoder.backbone_frozen()
    assert all(parameter.requires_grad for parameter in model.adapter_parameters())
    assert all(parameter.requires_grad for parameter in model.head_parameters())
