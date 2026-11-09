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

"""Official TwoHeadUnet shape and state-dict compatibility tests."""

import pytest

torch = pytest.importorskip("torch")

from frontiernet.unet import ResNet34Encoder, TwoHeadUnet  # noqa: E402


def test_encoder_stage_shapes():
    encoder = ResNet34Encoder(in_channels=4)
    features = encoder(torch.zeros(1, 4, 224, 224))
    shapes = [tuple(feature.shape) for feature in features]
    assert shapes == [
        (1, 4, 224, 224),
        (1, 64, 112, 112),
        (1, 64, 56, 56),
        (1, 128, 28, 28),
        (1, 256, 14, 14),
        (1, 512, 7, 7),
    ]
    assert encoder.out_channels == [4, 64, 64, 128, 256, 512]


def test_two_head_unet_output_shapes():
    model = TwoHeadUnet(classes=11, in_channels=4)
    with torch.no_grad():
        df, masks = model(torch.zeros(1, 4, 224, 224))
    assert tuple(df.shape) == (1, 1, 224, 224)
    assert tuple(masks.shape) == (1, 11, 224, 224)


def test_two_head_unet_pads_non_divisible_input():
    model = TwoHeadUnet(classes=11, in_channels=4)
    with torch.no_grad():
        df, masks = model(torch.zeros(1, 4, 200, 232))
    assert tuple(df.shape) == (1, 1, 200, 232)
    assert tuple(masks.shape) == (1, 11, 200, 232)


def test_state_dict_keys_match_official_layout():
    model = TwoHeadUnet(classes=11, in_channels=4)
    keys = set(model.state_dict())
    assert "encoder.conv1.weight" in keys
    assert "encoder.layer4.2.conv2.weight" in keys
    assert "decoder.blocks.0.conv1.0.weight" in keys
    assert "decoder.blocks.4.conv2.1.running_mean" in keys
    assert "segmentation_head.6.weight" in keys
    assert "df_regression_head.6.weight" in keys
    # smp deletes the classifier, so no fc/avgpool tensors may leak in
    assert not any(key.startswith("encoder.fc") for key in keys)
    assert not any(key.startswith("encoder.avgpool") for key in keys)


def test_first_conv_channel_adaptation():
    encoder = ResNet34Encoder(in_channels=4)
    assert encoder.conv1.in_channels == 4
    assert encoder.conv1.weight.shape[1] == 4
    # the extra channel repeats the first pretrained channel
    assert torch.allclose(encoder.conv1.weight[:, 3], encoder.conv1.weight[:, 0])


def test_rgb_only_variant_builds():
    model = TwoHeadUnet(classes=11, in_channels=3)
    with torch.no_grad():
        _, masks = model(torch.zeros(1, 3, 224, 224))
    assert tuple(masks.shape) == (1, 11, 224, 224)
