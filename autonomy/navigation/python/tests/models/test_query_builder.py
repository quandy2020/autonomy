import numpy as np

from vntpa.models.query_builder import build_query


def test_query_shapes():
    pose, goal = build_query(np.zeros(3), np.array([1.0, 2.0]))
    assert pose.shape == (1, 3) and goal.shape == (1, 2)
