import numpy as np

from vntpa.models.action_decoder import Action, decode_logits


def test_argmax_actions():
    logits = np.array([[0, 1, 0, 0], [1, 0, 0, 0]], dtype=np.float32)
    assert decode_logits(logits, 2) == [Action.FORWARD, Action.STOP]
