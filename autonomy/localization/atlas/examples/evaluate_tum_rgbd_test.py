#!/usr/bin/env python3
"""Tests for the fixed-scale TUM RGB-D trajectory evaluator."""

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from evaluate_tum_rgbd import TimedPose, associate_poses, evaluate, main


def write_trajectory(path, timestamps, positions):
    rows = [
        f"{timestamp:.6f} {x:.9f} {y:.9f} {z:.9f} 0 0 0 1\n"
        for timestamp, (x, y, z) in zip(timestamps, positions)
    ]
    path.write_text("".join(rows), encoding="utf-8")


def write_expected_timestamps(path, timestamps):
    path.write_text(
        "".join(f"{timestamp:.6f} rgb/x depth/x\n" for timestamp in timestamps),
        encoding="utf-8",
    )


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
            write_expected_timestamps(expected, range(5))

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
            self.assertAlmostEqual(
                metrics["tracking_output_coverage_percent"], 80.0
            )
            self.assertAlmostEqual(metrics["gt_support_fraction_percent"], 80.0)
            self.assertAlmostEqual(metrics["association_coverage_percent"], 100.0)
            self.assertLess(metrics["ate_rmse_m"], 1.0e-8)
            self.assertTrue((output / "aligned_trajectory.csv").is_file())

    def test_ground_truth_hole_does_not_penalize_tracking_coverage(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            ground_truth = root / "groundtruth.txt"
            estimate = root / "estimate.txt"
            expected = root / "associations.txt"
            all_timestamps = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
            supported_timestamps = [0.0, 1.0, 4.0, 5.0]
            all_positions = np.asarray(
                [[timestamp, 0.0, 0.0] for timestamp in all_timestamps]
            )
            supported_positions = np.asarray(
                [[timestamp, 0.0, 0.0] for timestamp in supported_timestamps]
            )
            write_trajectory(ground_truth, supported_timestamps, supported_positions)
            write_trajectory(estimate, all_timestamps, all_positions)
            write_expected_timestamps(expected, all_timestamps)

            metrics, *_ = evaluate(ground_truth, estimate, 0.02, expected)

            self.assertEqual(metrics["associated_poses"], 4)
            self.assertEqual(metrics["ground_truth_supported_expected_poses"], 4)
            self.assertAlmostEqual(
                metrics["tracking_output_coverage_percent"], 100.0
            )
            self.assertAlmostEqual(
                metrics["gt_support_fraction_percent"], 100.0 * 4.0 / 6.0
            )
            self.assertAlmostEqual(metrics["association_coverage_percent"], 100.0)
            self.assertAlmostEqual(metrics["coverage_percent"], 100.0)

    def test_missing_estimate_is_not_hidden_by_ground_truth_support(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            ground_truth = root / "groundtruth.txt"
            estimate = root / "estimate.txt"
            expected = root / "associations.txt"
            expected_timestamps = [0.0, 1.0, 2.0, 3.0, 4.0]
            estimate_timestamps = [0.0, 1.0, 3.0, 4.0, 100.0]
            write_trajectory(
                ground_truth,
                expected_timestamps,
                np.asarray([[value, 0.0, 0.0] for value in expected_timestamps]),
            )
            # An unrelated extra output must not conceal the missing frame at 2.0.
            write_trajectory(
                estimate,
                estimate_timestamps,
                np.asarray([[value, 0.0, 0.0] for value in estimate_timestamps]),
            )
            write_expected_timestamps(expected, expected_timestamps)

            metrics, *_ = evaluate(ground_truth, estimate, 0.02, expected)

            self.assertAlmostEqual(
                metrics["tracking_output_coverage_percent"], 80.0
            )
            self.assertAlmostEqual(metrics["gt_support_fraction_percent"], 100.0)
            self.assertAlmostEqual(metrics["association_coverage_percent"], 80.0)
            self.assertAlmostEqual(metrics["coverage_percent"], 80.0)

    def test_duplicate_estimate_timestamp_does_not_inflate_coverage(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            ground_truth = root / "groundtruth.txt"
            estimate = root / "estimate.txt"
            expected = root / "associations.txt"
            expected_timestamps = [0.0, 1.0, 2.0, 3.0]
            write_trajectory(
                ground_truth,
                expected_timestamps,
                np.asarray([[value, 0.0, 0.0] for value in expected_timestamps]),
            )
            write_trajectory(
                estimate,
                [0.0, 1.0, 1.0, 2.0, 3.0],
                np.asarray(
                    [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [10.0, 0.0, 0.0],
                     [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
                ),
            )
            write_expected_timestamps(expected, expected_timestamps)

            metrics, *_ = evaluate(ground_truth, estimate, 0.02, expected)

            self.assertEqual(metrics["raw_estimated_poses"], 5)
            self.assertEqual(metrics["estimated_poses"], 4)
            self.assertEqual(metrics["duplicate_estimated_timestamps"], 1)
            self.assertAlmostEqual(
                metrics["tracking_output_coverage_percent"], 100.0
            )
            self.assertAlmostEqual(metrics["association_coverage_percent"], 100.0)
            self.assertAlmostEqual(metrics["coverage_percent"], 100.0)


if __name__ == "__main__":
    unittest.main()
