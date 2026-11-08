from vntpa.runtime.inference.predictor import VNTResult


def test_result_summary():
    result = VNTResult(None, [], [], 3)
    assert "memory=3" in result.summary()
