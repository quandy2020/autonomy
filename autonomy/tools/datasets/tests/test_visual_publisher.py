# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np
from autonomy.tools.datasets.cli import DatasetsCli
from autonomy.tools.datasets.messages import Messages
from autonomy.tools.datasets.visual_publisher import (
    VisualDatasetPublisher,
    VisualPublishChannels,
)


def _stamp_ns(message: object) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def _image(timestamp_ns: int, *, depth: bool = False) -> types.SimpleNamespace:
    if depth:
        pixels = np.arange(6, dtype=np.uint16).reshape(2, 3)
        encoding = "mono16"
        frame_id = "depth_sensor"
    else:
        pixels = np.arange(6, dtype=np.uint8).reshape(2, 3)
        encoding = "mono8"
        frame_id = "camera_sensor"
    return types.SimpleNamespace(
        timestamp_ns=timestamp_ns,
        pixels=pixels,
        encoding=encoding,
        frame_id=frame_id,
        path=f"/{timestamp_ns}.png",
    )


def _frame(
    index: int,
    timestamp_ns: int,
    *,
    secondary_role: str | None = None,
    secondary_timestamp_ns: int | None = None,
    ground_truth: bool = False,
) -> types.SimpleNamespace:
    secondary = None
    if secondary_role is not None:
        secondary = _image(
            secondary_timestamp_ns or timestamp_ns,
            depth=secondary_role == "depth",
        )
    return types.SimpleNamespace(
        index=index,
        primary=_image(timestamp_ns),
        secondary=secondary,
        secondary_role=secondary_role,
        ground_truth=(
            types.SimpleNamespace(
                timestamp_ns=timestamp_ns,
                position=(1.0, 2.0, 3.0),
                orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            )
            if ground_truth
            else None
        ),
    )


class _Reader:
    def __init__(
        self,
        sensor_mode: str,
        frames: list[types.SimpleNamespace],
        imu: list[types.SimpleNamespace] | None = None,
    ) -> None:
        self.sensor_mode = sensor_mode
        self.frames = frames
        self.imu = imu or []
        self.frame_timestamps_ns = [frame.primary.timestamp_ns for frame in frames]
        self.load_images = True

    def __len__(self) -> int:
        return len(self.frames)

    def iter_frames(
        self, *, start: int = 0, max_frames: int | None = None
    ) -> object:
        end = len(self.frames)
        if max_frames is not None:
            end = min(end, start + max_frames)
        yield from self.frames[start:end]


class _Writer:
    def __init__(self, topic: str, events: list[tuple]) -> None:
        self.topic = topic
        self.events = events

    def write(self, message: object) -> None:
        self.events.append(("write", self.topic, message))


class _Node:
    def __init__(self, events: list[tuple]) -> None:
        self.events = events
        self.shutdown_called = False

    def create_writer(
        self, topic: str, _message_type: type, *, qos_depth: int
    ) -> _Writer:
        self.events.append(("create", topic, qos_depth))
        return _Writer(topic, self.events)

    def shutdown(self) -> None:
        self.shutdown_called = True
        self.events.append(("node_shutdown",))


class _Link:
    def __init__(self, events: list[tuple]) -> None:
        self.events = events
        self.node: _Node | None = None

    def init(self, node_name: str) -> None:
        self.events.append(("init", node_name))

    def Node(self, node_name: str) -> _Node:  # pylint: disable=invalid-name
        self.events.append(("node", node_name))
        self.node = _Node(self.events)
        return self.node

    def shutdown(self) -> None:
        self.events.append(("link_shutdown",))


class VisualDatasetPublisherTest(unittest.TestCase):
    def test_exact_nanosecond_stamp(self) -> None:
        timestamp_ns = 1_403_636_579_763_555_584
        self.assertEqual(
            Messages.stamp_from_nanoseconds(timestamp_ns),
            (1_403_636_579, 763_555_584),
        )
        self.assertEqual(Messages.stamp_from_nanoseconds(-1), (-1, 999_999_999))

    def test_stereo_inertial_order_start_and_max_frames(self) -> None:
        events: list[tuple] = []
        frames = [
            _frame(0, 1_000_000_000, secondary_role="right"),
            _frame(1, 1_100_000_000, secondary_role="right"),
            _frame(2, 1_200_000_000, secondary_role="right"),
        ]
        imu = [
            types.SimpleNamespace(
                timestamp_ns=timestamp_ns,
                angular_velocity=(1.0, 2.0, 3.0),
                linear_acceleration=(4.0, 5.0, 6.0),
            )
            for timestamp_ns in (
                599_999_999,
                600_000_000,
                900_000_000,
                1_100_000_000,
                1_200_000_000,
            )
        ]
        progress: list[tuple[int, int, float]] = []
        publisher = VisualDatasetPublisher(
            _Reader("stereo_inertial", frames, imu),
            link=_Link(events),
            rate=0.0,
            startup_delay=0.0,
            shutdown_delay=0.0,
            on_progress=lambda current, total, timestamp: progress.append(
                (current, total, timestamp)
            ),
        )

        self.assertEqual(publisher.run(start=1, max_frames=1), 1)

        writes = [event for event in events if event[0] == "write"]
        self.assertEqual(
            [event[1] for event in writes],
            [
                "/imu",
                "/imu",
                "/imu",
                "/camera/rgb/image_raw",
                "/camera/right/image_raw",
            ],
        )
        self.assertEqual(
            [_stamp_ns(event[2]) for event in writes[:3]],
            [600_000_000, 900_000_000, 1_100_000_000],
        )
        self.assertEqual(_stamp_ns(writes[3][2]), 1_100_000_000)
        self.assertEqual(_stamp_ns(writes[4][2]), 1_100_000_000)
        self.assertEqual(progress, [(1, 1, 1.1)])
        self.assertIn(("node_shutdown",), events)
        self.assertIn(("link_shutdown",), events)

    def test_ground_truth_is_published_before_its_image(self) -> None:
        events: list[tuple] = []
        publisher = VisualDatasetPublisher(
            _Reader("monocular", [_frame(0, 5_000_000_123, ground_truth=True)]),
            link=_Link(events),
            rate=0.0,
            startup_delay=0.0,
            shutdown_delay=0.0,
            publish_ground_truth=True,
        )

        self.assertEqual(publisher.run(), 1)

        writes = [event for event in events if event[0] == "write"]
        self.assertEqual(
            [event[1] for event in writes],
            ["/gt/odometry", "/camera/rgb/image_raw"],
        )
        odometry = writes[0][2]
        self.assertEqual(_stamp_ns(odometry), 5_000_000_123)
        self.assertEqual(odometry.header.frame_id, "map")
        self.assertEqual(odometry.child_frame_id, "camera_link")
        self.assertEqual(odometry.pose.pose.pose.position.x, 1.0)

    def test_rate_is_absolute_and_sleep_precedes_each_frame(self) -> None:
        events: list[tuple] = []
        frames = [
            _frame(0, 1_000_000_000),
            _frame(1, 1_100_000_000),
            _frame(2, 1_200_000_000),
        ]
        clock_values = iter((10.0, 10.02, 10.17))

        def sleep(delay: float) -> None:
            events.append(("sleep", delay))

        publisher = VisualDatasetPublisher(
            _Reader("monocular", frames),
            link=_Link(events),
            rate=1.0,
            startup_delay=0.0,
            shutdown_delay=0.0,
            clock=lambda: next(clock_values),
            sleeper=sleep,
        )

        self.assertEqual(publisher.run(), 3)

        timeline = [event for event in events if event[0] in ("sleep", "write")]
        self.assertEqual([event[0] for event in timeline], [
            "write",
            "sleep",
            "write",
            "sleep",
            "write",
        ])
        self.assertAlmostEqual(timeline[1][1], 0.08)
        self.assertAlmostEqual(timeline[3][1], 0.03)

    def test_startup_delay_happens_before_first_image(self) -> None:
        events: list[tuple] = []

        def sleep(delay: float) -> None:
            events.append(("sleep", delay))

        publisher = VisualDatasetPublisher(
            _Reader("monocular", [_frame(0, 5_000_000_000)]),
            link=_Link(events),
            rate=0.0,
            startup_delay=1.25,
            shutdown_delay=0.0,
            sleeper=sleep,
        )
        publisher.run()
        timeline = [event for event in events if event[0] in ("sleep", "write")]
        self.assertEqual(timeline[0], ("sleep", 1.25))
        self.assertEqual(timeline[1][1], "/camera/rgb/image_raw")

    def test_rgbd_preserves_depth_encoding_and_timestamp(self) -> None:
        events: list[tuple] = []
        frame = _frame(
            0,
            2_000_000_000,
            secondary_role="depth",
            secondary_timestamp_ns=2_010_000_000,
        )
        publisher = VisualDatasetPublisher(
            _Reader("rgbd", [frame]),
            channels=VisualPublishChannels(
                primary="/rgb",
                depth="/depth",
            ),
            link=_Link(events),
            rate=0.0,
            startup_delay=0.0,
            shutdown_delay=0.0,
        )

        publisher.run()

        writes = [event for event in events if event[0] == "write"]
        self.assertEqual([event[1] for event in writes], ["/rgb", "/depth"])
        depth = writes[1][2]
        self.assertEqual(depth.encoding, "mono16")
        self.assertEqual(depth.step, 6)
        self.assertEqual(len(depth.data), 12)
        self.assertEqual(_stamp_ns(depth), 2_010_000_000)

    def test_missing_stereo_secondary_fails_and_shuts_down(self) -> None:
        events: list[tuple] = []
        publisher = VisualDatasetPublisher(
            _Reader("stereo", [_frame(0, 1_000_000_000)]),
            link=_Link(events),
            rate=0.0,
            startup_delay=0.0,
            shutdown_delay=0.0,
        )
        with self.assertRaisesRegex(ValueError, "secondary_role=right"):
            publisher.run()
        self.assertIn(("node_shutdown",), events)
        self.assertIn(("link_shutdown",), events)

    def test_shutdown_delay_happens_after_last_image(self) -> None:
        events: list[tuple] = []

        def sleep(delay: float) -> None:
            events.append(("sleep", delay))

        publisher = VisualDatasetPublisher(
            _Reader("monocular", [_frame(0, 5_000_000_000)]),
            link=_Link(events),
            rate=0.0,
            startup_delay=0.0,
            shutdown_delay=0.75,
            sleeper=sleep,
        )
        publisher.run()
        timeline = [event for event in events if event[0] in ("sleep", "write")]
        self.assertEqual(timeline[0][0], "write")
        self.assertEqual(timeline[1], ("sleep", 0.75))


class DatasetsCliReaderRoutingTest(unittest.TestCase):
    @staticmethod
    def _args(dataset: str, sensor: str = "auto") -> types.SimpleNamespace:
        return types.SimpleNamespace(
            dataset=dataset,
            path=Path("/tmp/sequence"),
            atlas_sensor=sensor,
            camera="cam0",
            max_image_dt=0.05,
            max_pair_dt=0.0125,
            prefer_undistorted=False,
            association_file=Path("/tmp/associations.txt"),
        )

    def test_euroc_auto_routes_to_monocular(self) -> None:
        with mock.patch(
            "autonomy.tools.datasets.readers.euroc.EurocReader"
        ) as reader:
            DatasetsCli()._open_reader(
                self._args("euroc"), load_images=False
            )
        reader.assert_called_once_with(
            Path("/tmp/sequence"),
            sensor_mode="monocular",
            load_images=False,
        )

    def test_tum_vi_uses_fixed_stereo_inertial_constructor(self) -> None:
        with mock.patch(
            "autonomy.tools.datasets.readers.euroc.TumViReader"
        ) as reader:
            DatasetsCli()._open_reader(
                self._args("tum-vi"), load_images=True
            )
        reader.assert_called_once_with(
            Path("/tmp/sequence"),
            load_images=True,
        )

    def test_tum_rgbd_converts_pair_tolerance_to_nanoseconds(self) -> None:
        with mock.patch(
            "autonomy.tools.datasets.readers.tum_rgbd.TumRgbdReader"
        ) as reader:
            DatasetsCli()._open_reader(
                self._args("tum-rgbd"), load_images=True
            )
        reader.assert_called_once_with(
            Path("/tmp/sequence"),
            association_file=Path("/tmp/associations.txt"),
            load_images=True,
            max_pair_dt_ns=12_500_000,
        )

    def test_kitti_routes_to_stereo_reader(self) -> None:
        with mock.patch(
            "autonomy.tools.datasets.readers.kitti.KittiReader"
        ) as reader:
            DatasetsCli()._open_reader(
                self._args("kitti"), load_images=False
            )
        reader.assert_called_once_with(
            Path("/tmp/sequence"),
            load_images=False,
            ground_truth_path=None,
        )

    def test_dataset_rejects_incompatible_sensor_mode(self) -> None:
        with self.assertRaisesRegex(SystemExit, "KITTI odometry requires stereo"):
            DatasetsCli()._open_reader(
                self._args("kitti", "rgbd"), load_images=False
            )


if __name__ == "__main__":
    unittest.main()
