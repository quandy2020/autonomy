import numpy as np

from vntpa.models.encoder import DefmOutput


def test_defm_output():
    output = DefmOutput(tokens=np.zeros((2, 4)), pooled=np.zeros(4))
    assert output.tokens.shape == (2, 4)
