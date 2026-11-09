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

"""Freeze policy and optimizer-group tests for the retraining recipe."""

import pytest

torch = pytest.importorskip("torch")

from frontiernet.model import FrontierNetDeFM  # noqa: E402


def test_default_recipe_freezes_only_the_backbone(fake_backbone):
    model = FrontierNetDeFM(classes=11, backbone=fake_backbone)
    assert model.encoder.backbone_frozen()
    assert not any(p.requires_grad for p in model.backbone_parameters())
    assert all(p.requires_grad for p in model.adapter_parameters())
    assert all(p.requires_grad for p in model.head_parameters())


def test_parameter_groups_skip_frozen_backbone(fake_backbone):
    model = FrontierNetDeFM(classes=11, backbone=fake_backbone)
    groups = model.parameter_groups(head_lr=1e-3, adapter_lr=1e-4)
    assert [group["name"] for group in groups] == ["adapter", "heads"]
    learning_rates = {group["name"]: group["lr"] for group in groups}
    assert learning_rates["heads"] == pytest.approx(1e-3)
    assert learning_rates["adapter"] == pytest.approx(1e-4)


def test_unfreezing_backbone_adds_a_group(fake_backbone):
    model = FrontierNetDeFM(classes=11, backbone=fake_backbone)
    model.set_backbone_frozen(False)
    assert not model.encoder.backbone_frozen()
    groups = model.parameter_groups(head_lr=1e-3, backbone_lr=1e-5)
    assert [group["name"] for group in groups] == ["backbone", "adapter", "heads"]
    assert {group["name"]: group["lr"] for group in groups}["backbone"] == pytest.approx(1e-5)


def test_heads_can_be_frozen_for_encoder_only_probing(fake_backbone):
    model = FrontierNetDeFM(classes=11, backbone=fake_backbone)
    model.set_heads_frozen(True)
    assert not any(p.requires_grad for p in model.head_parameters())
    groups = model.parameter_groups(head_lr=1e-3)
    assert [group["name"] for group in groups] == ["adapter"]
    assert len(groups[0]["params"]) == len(model.adapter_parameters())


def test_describe_accounts_for_every_parameter(fake_backbone):
    model = FrontierNetDeFM(classes=11, backbone=fake_backbone)
    info = model.describe()
    assert info["backbone_trainable"] == 0
    assert info["trainable_total"] == info["adapter_total"] + info["heads_total"]
    assert info["total"] == (
        info["backbone_total"] + info["adapter_total"] + info["heads_total"]
    )
    assert info["heads_total"] > 0
