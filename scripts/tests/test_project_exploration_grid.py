"""Offline projection regressions using the actual generated native messages."""

from pathlib import Path
import math
import json
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_exploration_grid import project_grid
from automsgs.msgs.map_msgs.occupancy_grid_pb2 import OccupancyGrid
from automsgs.msgs.geometry_msgs.transform_stamped_pb2 import TransformStamped


def grid(frame, width, height, data, resolution=1):
    result = OccupancyGrid()
    result.header.frame_id = frame
    result.info.width, result.info.height = width, height
    result.info.resolution = resolution
    result.info.origin.orientation.w = 1
    result.data.extend(data)
    return result


def transform(source='source', target='target'):
    result = TransformStamped(child_frame_id=source)
    result.header.frame_id = target
    result.transform.rotation.w = 1
    return result


class ProjectionTest(unittest.TestCase):
    def test_target_coverage_projection_uses_minimum_and_unknown_veto(self):
        source = grid('source', 4, 2, [100, 100, 100, -1, 100, 20, 100, 100])
        target = grid('target', 2, 1, [100, 100], resolution=2)
        output, details = project_grid(source, target, transform(), aggregation='minimum')
        self.assertEqual(list(output.data), [20, -1])
        self.assertEqual(details['aggregation'], 'minimum')
        target.data[:] = [0, 0]
        repeated, _ = project_grid(source, target, transform(), aggregation='minimum')
        self.assertEqual(output, repeated)
        with self.assertRaises(ValueError):
            project_grid(source, target, transform(), aggregation='average')
    def test_identity_retains_unknown_and_ignores_target_values(self):
        source = grid('source', 2, 2, [0, -1, 40, 100])
        target = grid('target', 3, 2, [100] * 6)
        output, details = project_grid(source, target, transform())
        self.assertEqual(list(output.data), [0, -1, -1, 40, 100, -1])
        self.assertFalse(details['target_values_used_for_classification'])
        target.data[:] = [0] * 6
        repeated, _ = project_grid(source, target, transform())
        self.assertEqual(output, repeated)

    def test_fractional_cell_offset_does_not_invent_free_space(self):
        source = grid('source', 2, 1, [0, -1])
        target = grid('target', 2, 1, [0, 0])
        mapping = transform()
        mapping.transform.translation.x = -.25
        output, _ = project_grid(source, target, mapping)
        self.assertEqual(list(output.data), [-1, -1])

    def test_downsample_uses_all_cells_and_obstacle_precedence(self):
        source = grid('source', 4, 2, [0, 0, 0, -1, 0, 100, 0, 0])
        target = grid('target', 2, 1, [0, 0], resolution=2)
        output, _ = project_grid(source, target, transform())
        self.assertEqual(list(output.data), [100, -1])

    def test_optical_xz_origin_is_composed_before_projection(self):
        source = grid('source', 2, 2, [0, 100, 40, -1])
        source.info.origin.orientation.x = math.sqrt(.5)
        source.info.origin.orientation.w = math.sqrt(.5)
        mapping = transform()
        mapping.transform.rotation.x = -math.sqrt(.5)
        mapping.transform.rotation.w = math.sqrt(.5)
        output, _ = project_grid(source, grid('target', 2, 2, [0]*4), mapping)
        self.assertEqual(list(output.data), list(source.data))

    def test_parallel_height_requires_explicit_permission(self):
        mapping = transform()
        mapping.transform.translation.z = .21
        source, target = grid('source', 1, 1, [0]), grid('target', 1, 1, [0])
        with self.assertRaises(ValueError):
            project_grid(source, target, mapping)
        output, details = project_grid(source, target, mapping, True)
        self.assertEqual(list(output.data), [0])
        self.assertAlmostEqual(details['parallel_plane_height_offset_m'], .21)

    def test_mismatched_frames_bad_quaternion_and_tilt_are_rejected(self):
        source, target = grid('source', 1, 1, [0]), grid('target', 1, 1, [0])
        mapping = transform(source='wrong')
        with self.assertRaises(ValueError):
            project_grid(source, target, mapping)
        mapping = transform()
        mapping.transform.rotation.w = .9
        with self.assertRaises(ValueError):
            project_grid(source, target, mapping)
        mapping.transform.rotation.w = math.cos(.1)
        mapping.transform.rotation.x = math.sin(.1)
        with self.assertRaises(ValueError):
            project_grid(source, target, mapping, True)

    def test_extreme_resolution_ratio_is_bounded(self):
        source, target = grid('source', 1, 1, [0]), grid('target', 1, 1, [0], 1000)
        with self.assertRaises(ValueError):
            project_grid(source, target, transform())

    def test_cli_retains_failure_report_when_grid_cannot_be_written(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name, message in [('source', grid('source', 1, 1, [0])),
                                  ('target', grid('target', 1, 1, [100])),
                                  ('transform', transform())]:
                (root / (name + '.pb')).write_bytes(message.SerializeToString())
            # A regular file makes this output parent unusable even as root.
            (root / 'occupied').write_text('preserve')
            command = [sys.executable, str(Path(__file__).resolve().parents[1] / 'project_exploration_grid.py'),
                       '--source-map', str(root / 'source.pb'), '--target-grid', str(root / 'target.pb'),
                       '--transform', str(root / 'transform.pb'), '--output', str(root / 'occupied' / 'grid.pb'),
                       '--report', str(root / 'report.json')]
            result = subprocess.run(command, capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 1, result.stderr)
            report = json.loads((root / 'report.json').read_text())
            self.assertFalse(report['projected'])
            self.assertIn('error', report)
            self.assertEqual((root / 'occupied').read_text(), 'preserve')


if __name__ == '__main__':
    unittest.main()
