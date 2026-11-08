import numpy as np

from vntpa.training.losses import cross_entropy


def test_cross_entropy_zero_when_confident():
    loss = cross_entropy(np.array([[10.0, 0.0]]), np.array([0]))
    assert loss < 1e-4
