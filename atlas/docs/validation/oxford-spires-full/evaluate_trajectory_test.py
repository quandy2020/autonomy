#!/usr/bin/env python3
"""Regression tests for Oxford trajectory alignment metrics."""

import sys
import unittest
import warnings
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import evaluate_trajectory


class EvaluateTrajectoryTest(unittest.TestCase):
    def test_alignment_handles_long_trajectories_without_numeric_warnings(self):
        generator = np.random.default_rng(17)
        estimated_positions = generator.normal(size=(3372, 3)) * 500.0
        ground_truth_positions = generator.normal(size=(3372, 3)) * 100.0
        with warnings.catch_warnings(record=True) as captured_warnings:
            warnings.simplefilter("always", RuntimeWarning)
            rotation, translation = evaluate_trajectory.align_positions(
                estimated_positions, ground_truth_positions
            )
            aligned_positions = (
                np.einsum("ij,nj->ni", rotation, estimated_positions)
                + translation
            )

        self.assertTrue(np.isfinite(aligned_positions).all())
        self.assertEqual(
            [warning for warning in captured_warnings
             if issubclass(warning.category, RuntimeWarning)],
            [],
        )


if __name__ == "__main__":
    unittest.main()
