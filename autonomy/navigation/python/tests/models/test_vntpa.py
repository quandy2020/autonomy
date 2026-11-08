import numpy as np

from vntpa.models.vntpa import PlanResult


def test_plan_result_container():
    result = PlanResult(np.zeros((1, 8, 4), dtype=np.float32), [], [])
    assert result.logits.shape == (1, 8, 4)
