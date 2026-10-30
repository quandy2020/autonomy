#!/usr/bin/env python3
"""Unit tests for the EuRoC trajectory alignment helpers."""

import unittest

import numpy as np

from evaluate_euroc import similarity_alignment
from evaluate_kitti import rigid_alignment


class SimilarityAlignmentTest(unittest.TestCase):
    def test_recovers_known_scale_rotation_and_translation(self):
        source = np.asarray(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
                [2.0, -1.0, 0.5],
            ],
            dtype=float,
        )
        rotation = np.asarray(
            [
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        expected_scale = 2.5
        translation = np.asarray([4.0, -3.0, 1.5], dtype=float)
        target = expected_scale * (rotation @ source.T).T + translation

        scale, fitted_rotation, fitted_translation = similarity_alignment(
            source, target
        )
        aligned = scale * (fitted_rotation @ source.T).T + fitted_translation
        np.testing.assert_allclose(scale, expected_scale, atol=1.0e-12)
        np.testing.assert_allclose(fitted_rotation, rotation, atol=1.0e-12)
        np.testing.assert_allclose(fitted_translation, translation, atol=1.0e-12)
        np.testing.assert_allclose(aligned, target, atol=1.0e-12)

        se3_rotation, se3_translation = rigid_alignment(source, target)
        se3_aligned = (se3_rotation @ source.T).T + se3_translation
        se3_rmse = float(
            np.sqrt(np.mean(np.sum((se3_aligned - target) ** 2, axis=1)))
        )
        self.assertGreater(se3_rmse, 0.5)

    def test_rejects_zero_variance_source(self):
        source = np.ones((4, 3), dtype=float)
        target = np.arange(12, dtype=float).reshape(4, 3)
        with self.assertRaisesRegex(ValueError, "degenerate"):
            similarity_alignment(source, target)

    def test_rejects_non_finite_or_mismatched_inputs(self):
        with self.assertRaises(ValueError):
            similarity_alignment(np.zeros((3, 3)), np.zeros((4, 3)))
        invalid = np.zeros((3, 3), dtype=float)
        invalid[1, 2] = np.nan
        with self.assertRaises(ValueError):
            similarity_alignment(invalid, np.zeros((3, 3)))


if __name__ == "__main__":
    unittest.main()
