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

"""Regression guard for the official ``rgbd_11cls.pth`` state-dict contract.

``state_dict_keys.txt`` was captured from the upstream FrontierNet model
(``segmentation_models_pytorch`` 0.3.3 encoder/decoder + the official heads) and
verified against this port: 308 tensors, identical shapes, strict load OK and
bit-identical forward output. Any drift here breaks checkpoint loading.
"""

from pathlib import Path

import pytest

torch = pytest.importorskip("torch")

from frontiernet.unet import TwoHeadUnet  # noqa: E402

KEYS_FILE = Path(__file__).with_name("state_dict_keys.txt")
OFFICIAL_PARAMETERS = 24_533_068


def _expected_keys() -> list:
    return [line.strip() for line in KEYS_FILE.read_text().splitlines() if line.strip()]


def test_key_count_matches_official_checkpoint():
    expected = _expected_keys()
    assert len(expected) == 308
    assert len(set(expected)) == 308


def test_state_dict_matches_official_contract():
    model = TwoHeadUnet(classes=11, in_channels=4)
    actual = sorted(model.state_dict())
    expected = sorted(_expected_keys())
    missing = [key for key in expected if key not in set(actual)]
    extra = [key for key in actual if key not in set(expected)]
    assert not missing, f"missing tensors: {missing[:5]}"
    assert not extra, f"unexpected tensors: {extra[:5]}"
    assert actual == expected


def test_parameter_count_matches_official_model():
    model = TwoHeadUnet(classes=11, in_channels=4)
    assert sum(parameter.numel() for parameter in model.parameters()) == OFFICIAL_PARAMETERS
