#!/usr/bin/env python3
"""Tests for the fixed-scale TUM RGB-D trajectory evaluator."""

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from evaluate_tum_rgbd import TimedPose, associate_poses, main


def write_trajectory(path, timestamps, positions):
    rows = [
        f"{timestamp:.6f} {x:.9f} {y:.9f} {z:.9f} 0 0 0 1\n"
        for timestamp, (x, y, z) in zip(timestamps, positions)
    ]
    path.write_text("".join(rows), encoding="utf-8")


class AssociationTest(unittest.TestCase):
    def test_is_nearest_and_one_to_one(self):
        reference = [
            TimedPose(1.0, np.zeros(3)),
            TimedPose(1.01, np.zeros(3)),
        ]
        estimate = [
            TimedPose(1.004, np.zeros(3)),
            TimedPose(1.006, np.zeros(3)),
        ]
        self.assertEqual(
            associate_poses(reference, estimate, 0.02), [(0, 0), (1, 1)]
        )


class EvaluationTest(unittest.TestCase):
    def test_main_writes_fixed_scale_metrics_and_expected_coverage(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            ground_truth = root / "groundtruth.txt"
            estimate = root / "estimate.txt"
            expected = root / "associations.txt"
            output = root / "evaluation"
            timestamps = [1.0, 2.0, 3.0, 4.0]
            positions = np.asarray(
                [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0],
                 [1.0, 1.0, 0.0], [1.0, 1.0, 1.0]]
            )
            write_trajectory(ground_truth, timestamps, positions)
            write_trajectory(
                estimate,
                [timestamp + 0.005 for timestamp in timestamps],
                positions + np.asarray([3.0, -2.0, 0.5]),
            )
            expected.write_text(
                "".join(f"{timestamp} rgb/x depth/x\n" for timestamp in range(5)),
                encoding="utf-8",
            )

            arguments = [
                "evaluate_tum_rgbd.py",
                str(ground_truth),
                str(estimate),
                str(output),
                "--expected-timestamps",
                str(expected),
            ]
            with mock.patch.object(sys, "argv", arguments), mock.patch(
                "evaluate_tum_rgbd.create_plot"
            ) as create_plot:
                self.assertEqual(main(), 0)
                create_plot.assert_called_once()

            metrics = json.loads(
                (output / "metrics.json").read_text(encoding="utf-8")
            )
            self.assertEqual(metrics["alignment"], "SE3")
            self.assertTrue(metrics["scale_fixed"])
            self.assertEqual(metrics["associated_poses"], 4)
            self.assertAlmostEqual(metrics["coverage_percent"], 80.0)
            self.assertLess(metrics["ate_rmse_m"], 1.0e-8)
            self.assertTrue((output / "aligned_trajectory.csv").is_file())


if __name__ == "__main__":
    unittest.main()
