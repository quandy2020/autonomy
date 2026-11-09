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

import numpy as np

from vntpa.data.dataset import ContextMemory
from vntpa.models.context import context


def test_context_padding():
    memory = ContextMemory(max_keyframes=4, min_distance=0.0, min_interval_s=0.0)
    memory.add(np.ones((2, 3)), np.zeros(3), 0.0)
    batch = context(memory, 4, 2, 3)
    assert batch.tokens.shape == (4, 2, 3)
    assert batch.valid.tolist() == [True, False, False, False]
