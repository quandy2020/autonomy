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

"""Optimizer factory kept separate from training orchestration."""

from __future__ import annotations


def build_optimizer(model, learning_rate: float = 3e-4, weight_decay: float = 1e-4):
    """AdamW factory; imports torch lazily so deployment stays lightweight."""
    import torch

    return torch.optim.AdamW(model.parameters(), lr=learning_rate, weight_decay=weight_decay)
