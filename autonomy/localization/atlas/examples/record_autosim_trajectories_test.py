#!/usr/bin/env python3
"""Standard-library tests for the AutoSim validation recorder."""

from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import record_autosim_trajectories as recorder


def namespace(**kwargs):
    return SimpleNamespace(**kwargs)


def pose(
    position: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
):
    return namespace(
        position=namespace(x=position[0], y=position[1], z=position[2]),
        orientation=namespace(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3],
        ),
    )


def stamped_pose(
    sec: int,
    nanosec: int,
    position: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
):
    return namespace(
        header=namespace(stamp=namespace(sec=sec, nanosec=nanosec)),
        pose=pose(position, quaternion),
    )


def odometry_message(
    sec: int,
    nanosec: int,
    position: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
    *,
    parent_frame: str = "map",
    child_frame: str = "base_link",
):
    # AutoMsgs Odometry nests Pose as message.pose.pose.pose.
    return namespace(
        header=namespace(
            stamp=namespace(sec=sec, nanosec=nanosec),
            frame_id=parent_frame,
        ),
        child_frame_id=child_frame,
        pose=namespace(pose=namespace(pose=pose(position, quaternion))),
    )


def path_message(poses, frame_id="map"):
    return namespace(header=namespace(frame_id=frame_id), poses=poses)


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.readers = {}
        self.shutdown_called = False

    def create_reader(self, topic, callback, message_type):
        handle = namespace(topic=topic)
        self.readers[topic] = (callback, message_type, handle)
        return handle

    def publish(self, topic, message):
        self.readers[topic][0](message)

    def shutdown(self):
        self.shutdown_called = True


class FakeLink:
    def __init__(self):
        self.initialized_name = None
        self.node = None
        self.shutdown_called = False

    def init(self, name):
        self.initialized_name = name

    def Node(self, name):
        self.node = FakeNode(name)
        return self.node

    def is_shutdown(self):
        return self.shutdown_called

    def shutdown(self):
        self.shutdown_called = True


class RecordAutosimTrajectoriesTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory(
            prefix="atlas_record_autosim_"
        )
        self.output_directory = Path(self.temporary_directory.name)
        self.extrinsic = recorder.RigidTransform(
            recorder.DEFAULT_BASE_TO_CAMERA_XYZ,
            recorder.DEFAULT_BASE_TO_CAMERA_QUATERNION,
        )

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def test_ground_truth_base_pose_is_composed_to_optical_camera(self) -> None:
        message = odometry_message(
            12,
            345,
            (1.0, 2.0, 0.01),
            (0.0, 0.0, 0.0, 1.0),
        )

        converted = recorder.odometry_to_tum_pose(message, self.extrinsic)

        self.assertEqual(converted.timestamp_ns, 12_000_000_345)
        self.assertAlmostEqual(converted.x, 1.03)
        self.assertAlmostEqual(converted.y, 2.0)
        self.assertAlmostEqual(converted.z, 0.21)
        self.assertAlmostEqual(converted.qx, -0.5)
        self.assertAlmostEqual(converted.qy, 0.5)
        self.assertAlmostEqual(converted.qz, -0.5)
        self.assertAlmostEqual(converted.qw, 0.5)

    def test_extrinsic_translation_rotates_with_turning_base(self) -> None:
        half_sqrt = math.sqrt(0.5)
        message = odometry_message(
            1,
            0,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, half_sqrt, half_sqrt),
        )

        converted = recorder.odometry_to_tum_pose(message, self.extrinsic)

        self.assertAlmostEqual(converted.x, 0.0, places=12)
        self.assertAlmostEqual(converted.y, 0.03, places=12)
        self.assertAlmostEqual(converted.z, 0.20, places=12)

    def test_odometry_trajectory_preserves_epoch_regression(self) -> None:
        trajectory = recorder.OdometryTrajectory("/atlas/odometry")
        trajectory.append(
            odometry_message(2, 0, (0, 0, 0), (0, 0, 0, 1))
        )
        trajectory.append(
            odometry_message(1, 0, (5, 0, 0), (0, 0, 0, 1))
        )

        summary = trajectory.summary(self.output_directory / "atlas.tum")

        self.assertEqual(summary["pose_count"], 2)
        self.assertEqual(summary["timestamp_regression_count"], 1)
        self.assertTrue(summary["valid"])

    def test_path_conversion_preserves_order_and_exact_timestamps(self) -> None:
        message = path_message(
            [
                stamped_pose(
                    12, 345, (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 2.0)
                ),
                stamped_pose(
                    11, 999, (4.0, 5.0, 6.0), (0.0, 0.0, 1.0, 1.0)
                ),
            ]
        )

        poses = recorder.path_to_tum_poses(message)

        self.assertEqual(
            [item.timestamp_ns for item in poses],
            [12_000_000_345, 11_000_000_999],
        )
        self.assertAlmostEqual(poses[0].qw, 1.0)
        self.assertAlmostEqual(poses[1].qz, 1.0 / math.sqrt(2.0))
        text = recorder.tum_text(poses, "/atlas/trajectory")
        self.assertIn("12.000000345 1.000000000000", text)
        self.assertLess(text.index("12.000000345"), text.index("11.000000999"))

    def test_conversion_rejects_invalid_stamp_and_quaternion(self) -> None:
        with self.assertRaisesRegex(ValueError, "nanosec"):
            recorder.odometry_to_tum_pose(
                odometry_message(
                    0, 1_000_000_000, (0, 0, 0), (0, 0, 0, 1)
                )
            )
        with self.assertRaisesRegex(ValueError, "quaternion norm is zero"):
            recorder.odometry_to_tum_pose(
                odometry_message(0, 0, (0, 0, 0), (0, 0, 0, 0))
            )

    def test_map_summaries_require_points_and_known_grid_cells(self) -> None:
        dense = namespace(width=4, height=2, point_step=16, data=bytes(128))
        dense_summary = recorder.summarize_dense_map(dense)
        self.assertEqual(dense_summary["point_count"], 8)
        self.assertTrue(dense_summary["valid"])
        empty_data = namespace(width=4, height=2, point_step=16, data=b"")
        self.assertFalse(recorder.summarize_dense_map(empty_data)["valid"])

        grid = namespace(
            info=namespace(width=3, height=2), data=[-1, -1, 0, 0, 100, -1]
        )
        grid_summary = recorder.summarize_occupancy_grid(grid)
        self.assertEqual(grid_summary["known_cell_count"], 3)
        self.assertEqual(grid_summary["occupied_cell_count"], 1)
        self.assertTrue(grid_summary["valid"])

        unknown_grid = namespace(
            info=namespace(width=2, height=2), data=[-1, -1, -1, -1]
        )
        self.assertFalse(
            recorder.summarize_occupancy_grid(unknown_grid)["valid"]
        )
        malformed_grid = namespace(info=namespace(width=2, height=2), data=[0])
        self.assertFalse(
            recorder.summarize_occupancy_grid(malformed_grid)["valid"]
        )

    def test_atomic_write_replaces_complete_file(self) -> None:
        output = self.output_directory / "nested" / "validation_summary.json"
        recorder.atomic_write_text(output, "old\n")
        recorder.atomic_write_text(output, json.dumps({"success": True}) + "\n")

        self.assertTrue(json.loads(output.read_text(encoding="utf-8"))["success"])
        self.assertEqual(list(output.parent.glob(f".{output.name}.*.tmp")), [])

    def make_recorder(self):
        link = FakeLink()
        message_types = recorder.MessageTypes(
            "Odometry", "Path", "PointCloud2", "Grid"
        )
        validation = recorder.AutosimRecorder(
            link,
            message_types,
            node_name="test_recorder",
            atlas_odometry_topic="/atlas/odometry",
            ground_truth_odometry_topic="/gt/odometry",
            atlas_path_topic="/atlas/trajectory",
            ground_truth_path_topic="/atlas/ground_truth_trajectory",
            dense_map_topic="/atlas/dense_map_points",
            occupancy_grid_topic="/atlas/occupancy_grid",
            base_to_camera=self.extrinsic,
        )
        return link, validation

    def test_summary_requires_raw_odometry_and_optional_maps(self) -> None:
        _, validation = self.make_recorder()

        summary = recorder.build_summary(
            validation,
            output_dir=self.output_directory,
            require_maps=True,
            requested_duration_sec=1.0,
            elapsed_sec=1.0,
            exit_reason="duration",
        )
        self.assertFalse(summary["success"])
        self.assertEqual(len(summary["errors"]), 4)

        validation.atlas_odometry.append(
            odometry_message(
                1,
                2,
                (1, 2, 3),
                (0, 0, 0, 1),
                child_frame="camera_optical_frame",
            )
        )
        validation.ground_truth_odometry.append(
            odometry_message(1, 2, (1, 2, 3), (0, 0, 0, 1)),
            self.extrinsic,
        )
        validation.latest_dense_map = namespace(
            width=1, height=1, point_step=16, data=bytes(16)
        )
        validation.latest_occupancy_grid = namespace(
            info=namespace(width=2, height=1), data=[0, 100]
        )
        summary = recorder.build_summary(
            validation,
            output_dir=self.output_directory,
            require_maps=True,
            requested_duration_sec=1.0,
            elapsed_sec=1.0,
            exit_reason="duration",
        )
        self.assertTrue(summary["success"])
        self.assertEqual(summary["errors"], [])

    def test_autolink_recorder_binds_and_drains_all_topics(self) -> None:
        link, validation = self.make_recorder()
        atlas = odometry_message(
            1,
            2,
            (1, 2, 3),
            (0, 0, 0, 1),
            child_frame="camera_optical_frame",
        )
        ground_truth = odometry_message(1, 2, (1, 2, 3), (0, 0, 0, 1))
        path = path_message(
            [stamped_pose(1, 2, (1, 2, 3), (0, 0, 0, 1))]
        )
        link.node.publish("/atlas/odometry", atlas)
        link.node.publish("/atlas/odometry", atlas)
        link.node.publish("/gt/odometry", ground_truth)
        link.node.publish("/atlas/trajectory", path)
        link.node.publish("/atlas/ground_truth_trajectory", path)
        link.node.publish(
            "/atlas/dense_map_points",
            namespace(width=1, height=1, point_step=16, data=bytes(16))
        )
        link.node.publish(
            "/atlas/occupancy_grid",
            namespace(info=namespace(width=1, height=1), data=[100])
        )

        validation.poll_once()

        self.assertEqual(
            set(link.node.readers),
            {
                "/atlas/odometry",
                "/gt/odometry",
                "/atlas/trajectory",
                "/atlas/ground_truth_trajectory",
                "/atlas/dense_map_points",
                "/atlas/occupancy_grid",
            },
        )
        self.assertEqual(validation.atlas_odometry.message_count, 2)
        self.assertEqual(len(validation.atlas_odometry.poses), 2)
        self.assertEqual(validation.ground_truth_odometry.message_count, 1)
        self.assertEqual(validation.atlas_path.pose_count, 1)
        self.assertEqual(validation.dense_map_message_count, 1)
        self.assertEqual(validation.occupancy_grid_message_count, 1)
        validation.freeze()
        link.node.publish("/atlas/odometry", atlas)
        self.assertEqual(validation.atlas_odometry.message_count, 2)
        validation.shutdown()
        self.assertTrue(link.node.shutdown_called)
        self.assertTrue(link.shutdown_called)


if __name__ == "__main__":
    unittest.main()
