"""Build the planner query tensors."""

from __future__ import annotations

import numpy as np


def build_query(pose: np.ndarray, goal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(query_pose[1,3], goal[1,2])`` as float32."""
    query_pose = np.asarray(pose, dtype=np.float32).reshape(1, 3)
    query_goal = np.asarray(goal, dtype=np.float32).reshape(1, 2)
    return query_pose, query_goal
