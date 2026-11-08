import numpy as np

from vntpa.training.evaluation.metrics import spl, success


def test_success_and_spl():
    assert success(np.array([0.0, 0.1]), np.array([0.0, 0.0]))
    assert spl(True, 1.0, 2.0) == 0.5
    assert spl(False, 1.0, 2.0) == 0.0
