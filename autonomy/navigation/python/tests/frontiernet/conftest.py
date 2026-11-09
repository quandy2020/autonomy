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

"""Shared fixtures; torch is optional so numpy tests still run."""

import pytest


@pytest.fixture
def fake_backbone():
    """Stand-in for DeFM's ViT so tests never download weights."""
    torch = pytest.importorskip("torch")
    import torch.nn as nn
    import torch.nn.functional as F

    class FakeBackbone(nn.Module):
        embed_dim = 384

        def __init__(self, levels: int = 5, grid: int = 16) -> None:
            super().__init__()
            self.levels = levels
            self.grid = grid
            self.stem = nn.Conv2d(3, self.embed_dim, kernel_size=1)

        def forward(self, x):
            base = F.adaptive_avg_pool2d(self.stem(x), self.grid)
            return [base * (index + 1) for index in range(self.levels)]

    del torch
    return FakeBackbone()
