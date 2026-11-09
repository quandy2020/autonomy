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

"""Build the planner query tensors."""

from __future__ import annotations

import numpy as np


def query(pose: np.ndarray, goal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(query_pose[1,3], goal[1,2])`` as float32."""
    query_pose = np.asarray(pose, dtype=np.float32).reshape(1, 3)
    query_goal = np.asarray(goal, dtype=np.float32).reshape(1, 2)
    return query_pose, query_goal
