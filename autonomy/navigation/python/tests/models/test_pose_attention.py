import numpy as np

from vntpa.models.pose_attention import relative_pose


def test_relative_pose_rotation():
    delta = relative_pose(np.array([1.0, 0.0, np.pi / 2]), np.zeros(3))
    assert np.allclose(delta, [0.0, -1.0, np.pi / 2], atol=1e-5)
