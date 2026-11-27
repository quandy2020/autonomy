import argparse
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np
from tune_cartographer import evaluate, values


class EvaluationTest(unittest.TestCase):
    def test_se2_alignment_and_relative_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            t = np.arange(0., 4., .1)
            yaw = t * .2
            truth = np.column_stack((t, np.sin(t), np.cos(t), t * 0, t * 0, t * 0,
                                     np.sin(yaw / 2), np.cos(yaw / 2)))
            estimate = truth.copy()
            c, s = math.cos(.7), math.sin(.7)
            estimate[:, 1:3] = truth[:, 1:3] @ np.array([[c, s], [-s, c]]) + [4, -3]
            estimate[:, 6] = np.sin((yaw + .7) / 2)
            estimate[:, 7] = np.cos((yaw + .7) / 2)
            np.savetxt(directory / "gt.tum", truth)
            np.savetxt(directory / "estimate.tum", estimate)
            metrics = evaluate(directory / "estimate.tum", directory / "gt.tum")
            self.assertLess(metrics["ate_xy_rmse_m"], 1e-10)
            self.assertLess(metrics["rpe_xy_1s_rmse_m"], 1e-10)
            self.assertLess(metrics["rpe_yaw_1s_rmse_rad"], 1e-10)
            estimate[:, 1] *= 1.1
            np.savetxt(directory / "estimate.tum", estimate)
            self.assertGreater(evaluate(directory / "estimate.tum", directory / "gt.tum")["ate_xy_rmse_m"], .01)
            estimate[:, 0] += 100
            np.savetxt(directory / "estimate.tum", estimate)
            with self.assertRaises(ValueError):
                evaluate(directory / "estimate.tum", directory / "gt.tum")

    def test_values_reject_invalid_numbers(self):
        self.assertEqual(values(".02,.025,.02"), [.02, .025])
        for text in ("nan", "inf", "-1", "0"):
            with self.assertRaises(argparse.ArgumentTypeError):
                values(text)


if __name__ == "__main__":
    unittest.main()
