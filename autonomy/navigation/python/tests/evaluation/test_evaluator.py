from vntpa.training.evaluation.evaluator import EvalResult


def test_eval_result_rates():
    result = EvalResult(episodes=2, successes=1, spl_values=[0.5, 0.0])
    assert result.success_rate == 0.5
    assert result.mean_spl == 0.25
