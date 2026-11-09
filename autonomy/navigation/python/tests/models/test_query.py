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

from vntpa.models.query import query


def test_query_shapes():
    pose, goal = query(np.zeros(3), np.array([1.0, 2.0]))
    assert pose.shape == (1, 3) and goal.shape == (1, 2)
    assert np.allclose(pose, [[0.0, 0.0, 0.0]])
    assert np.allclose(goal, [[-2.0, -1.0]])
