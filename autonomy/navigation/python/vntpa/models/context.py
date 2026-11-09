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

"""Convert keyframe memory into fixed-size context tensors."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from vntpa.data.dataset import ContextMemory


@dataclass
class ContextTensors:
    tokens: np.ndarray
    poses: np.ndarray
    valid: np.ndarray


def context(
    memory: ContextMemory, context_size: int, token_count: int, token_dim: int
) -> ContextTensors:
    """Return padded tokens ``[K,P,D]``, poses ``[K,3]`` and a boolean mask."""
    tokens, poses, mask = memory.context(context_size, token_count, token_dim)
    return ContextTensors(tokens, poses, mask.astype(bool))
