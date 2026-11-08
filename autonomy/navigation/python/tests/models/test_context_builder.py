import numpy as np

from vntpa.data.dataset import ContextMemory
from vntpa.models.context_builder import build_context


def test_context_padding():
    memory = ContextMemory(max_keyframes=4, min_distance=0.0, min_interval_s=0.0)
    memory.add(np.ones((2, 3)), np.zeros(3), 0.0)
    context = build_context(memory, 4, 2, 3)
    assert context.tokens.shape == (4, 2, 3)
    assert context.valid.tolist() == [True, False, False, False]
