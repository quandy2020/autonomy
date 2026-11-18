#!/usr/bin/env python3
"""Offline evaluator regression fixtures using the real generated automsgs.

These small synthetic maps test score semantics; they are never simulator,
SLAM, exploration, fusion, or physical-execution acceptance evidence.
"""

import importlib.util
import math
from pathlib import Path
import unittest

import numpy as np
from automsgs.msgs.geometry_msgs.pose_stamped_pb2 import PoseStamped
from automsgs.msgs.map_msgs.occupancy_grid_pb2 import OccupancyGrid


SCRIPT = Path(__file__).resolve().parents[1] / 'compare_exploration_maps.py'
SPEC = importlib.util.spec_from_file_location('compare_exploration_maps', SCRIPT)
COMPARISON = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(COMPARISON)


def grid(values, origin=(0, 0), frame='map'):
    """Build a clearly synthetic regression fixture with the native schema."""
    array = np.asarray(values, dtype=np.int32)
    message = OccupancyGrid()
    message.header.frame_id = frame
    message.info.resolution = 1
    message.info.width = array.shape[1]
    message.info.height = array.shape[0]
    message.info.origin.position.x = origin[0]
    message.info.origin.position.y = origin[1]
    message.info.origin.orientation.w = 1
    message.data.extend(array.reshape(-1).tolist())
    return message


def pose(x=2.5, y=2.5, frame='map'):
    """Return a synthetic start pose in the declared map frame."""
    message = PoseStamped()
    message.header.frame_id = frame
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.orientation.w = 1
    return message


class MapComparisonTest(unittest.TestCase):
    """Protect fixed denominators, unknown semantics, and explicit alignment."""

    def compare(self, truth, algorithm, start=None, radius=0):
        """Evaluate a fixture without implying a real exploration run."""
        return COMPARISON.compare_grids(truth, algorithm, start or pose(), radius)

    def test_unknown_map_cannot_claim_coverage(self):
        report = self.compare(grid(np.zeros((5, 5))), grid(np.full((5, 5), -1)))
        self.assertEqual(report['reachable_truth_cells'], 25)
        self.assertEqual(report['known_coverage_fraction'], 0)
        self.assertEqual(report['residual_unknown_regions']['largest'][0]['cells'], 25)
        self.assertFalse(report['exploration_accepted'])

    def test_known_occupied_is_not_correct_free_coverage(self):
        values = np.zeros((5, 5), dtype=int)
        values[1, 1] = 100
        report = self.compare(grid(np.zeros((5, 5))), grid(values))
        self.assertEqual(report['known_coverage_fraction'], 1)
        self.assertAlmostEqual(report['correct_free_coverage_fraction'], 24 / 25)
        self.assertAlmostEqual(report['reachable_false_occupied_fraction'], 1 / 25)

    def test_uncertain_is_separate_from_unknown_and_confirmed_free(self):
        report = self.compare(grid(np.zeros((5, 5))), grid(np.full((5, 5), 50)))
        self.assertEqual(report['known_coverage_fraction'], 1)
        self.assertEqual(report['correct_free_coverage_fraction'], 0)
        self.assertEqual(report['reachable_uncertain_cells'], 25)
        self.assertEqual(report['residual_unknown_regions']['count'], 0)
        self.assertEqual(report['residual_not_confirmed_free_regions']['largest'][0]['cells'], 25)

    def test_smaller_algorithm_extent_does_not_reduce_denominator(self):
        report = self.compare(grid(np.zeros((5, 5))), grid(np.zeros((3, 3)), (1, 1)))
        self.assertEqual(report['reachable_truth_cells'], 25)
        self.assertEqual(report['reachable_unknown_cells'], 16)
        self.assertAlmostEqual(report['correct_free_coverage_fraction'], 9 / 25)
        self.assertEqual(report['alignment']['offset_cells'], [1, 1])

    def test_disconnected_room_is_excluded_from_start_domain(self):
        values = np.zeros((5, 5), dtype=int)
        values[:, 2] = 100
        report = self.compare(grid(values), grid(values), pose(0.5, 2.5))
        self.assertEqual(report['reachable_truth_cells'], 10)
        self.assertEqual(report['correct_free_coverage_fraction'], 1)

    def test_diagonal_does_not_cross_blocked_corners(self):
        mask = np.asarray([[True, False], [False, True]])
        reached = COMPARISON.connected_component(mask, (0, 0))
        self.assertEqual(np.count_nonzero(reached), 1)

    def test_footprint_blocks_wall_adjacent_start(self):
        values = np.zeros((5, 5), dtype=int)
        values[:, 0] = 100
        with self.assertRaisesRegex(ValueError, 'footprint-clear'):
            self.compare(grid(values), grid(values), pose(1.5, 2.5), radius=0.6)
        report = self.compare(grid(values), grid(values), radius=0.4)
        self.assertGreater(report['reachable_truth_cells'], 0)

    def test_false_free_on_true_obstacles_is_reported(self):
        truth = np.zeros((5, 5), dtype=int)
        truth[1, 1] = 100
        report = self.compare(grid(truth), grid(np.zeros((5, 5))))
        self.assertEqual(report['global_confusion_cells']['occupied']['free'], 1)

    def test_rejects_implicit_frame_resolution_or_origin_fitting(self):
        truth = grid(np.zeros((5, 5)))
        other = grid(np.zeros((5, 5)), frame='odom')
        with self.assertRaisesRegex(ValueError, 'frames differ'):
            self.compare(truth, other)
        other.header.frame_id = 'map'
        other.info.resolution = 0.5
        with self.assertRaisesRegex(ValueError, 'resolutions differ'):
            self.compare(truth, other)
        other.info.resolution = 1
        other.info.origin.position.x = 0.25
        with self.assertRaisesRegex(ValueError, 'cell lattice'):
            self.compare(truth, other)

    def test_rejects_malformed_native_messages(self):
        truth = grid(np.zeros((5, 5)))
        other = grid(np.zeros((5, 5)))
        del other.data[-1]
        with self.assertRaisesRegex(ValueError, 'data length'):
            self.compare(truth, other)
        other = grid(np.zeros((5, 5)))
        other.data[0] = 2147483647
        with self.assertRaisesRegex(ValueError, 'occupancy values'):
            self.compare(truth, other)
        other.data[0] = 0
        other.info.origin.orientation.w = 0
        with self.assertRaisesRegex(ValueError, 'normalized grid'):
            self.compare(truth, other)

    def test_rejects_wrong_start_frame_and_nonfinite_geometry(self):
        truth = grid(np.zeros((5, 5)))
        with self.assertRaisesRegex(ValueError, 'truth frame'):
            self.compare(truth, truth, pose(frame='odom'))
        with self.assertRaisesRegex(ValueError, 'start position'):
            self.compare(truth, truth, pose(math.nan, 2.5))
        with self.assertRaisesRegex(ValueError, 'robot radius'):
            self.compare(truth, truth, radius=math.inf)


if __name__ == '__main__':
    unittest.main()
