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

"""Publish visual SLAM datasets on Atlas-aligned Autolink topics."""

from __future__ import annotations

import bisect
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.sensor_msgs.imu_pb2 import Imu
from autonomy.tools.datasets.messages import Messages

_INERTIAL_MODES = {"monocular_inertial", "stereo_inertial"}
_STEREO_MODES = {"stereo", "stereo_inertial"}
_SUPPORTED_MODES = {
    "monocular",
    "stereo",
    "monocular_inertial",
    "stereo_inertial",
    "rgbd",
}


@dataclass
class VisualPublishChannels:
    """Atlas input channels used by visual dataset replay."""

    primary: str = "/camera/rgb/image_raw"
    right: str = "/camera/right/image_raw"
    depth: str = "/camera/depth/image_raw"
    imu: str = "/imu"
    ground_truth: str = "/gt/odometry"


@dataclass
class VisualPublishFrames:
    """Frame IDs attached to replayed sensor messages."""

    primary: str = "camera"
    right: str = "camera_right"
    depth: str = "camera_depth"
    imu: str = "imu"
    ground_truth: str = "map"
    ground_truth_child: str = "camera_link"


class VisualDatasetPublisher:
    """Replay a visual or visual-inertial dataset for ``AtlasNode``."""

    def __init__(
        self,
        reader: Any,
        *,
        channels: VisualPublishChannels | None = None,
        frames: VisualPublishFrames | None = None,
        rate: float = 1.0,
        startup_delay: float = 1.0,
        shutdown_delay: float = 0.5,
        imu_lead_seconds: float = 0.5,
        publish_ground_truth: bool = False,
        link: Any | None = None,
        node_name: str = "visual_dataset_publisher",
        on_progress: Callable[[int, int, float], None] | None = None,
        clock: Callable[[], float] = time.perf_counter,
        sleeper: Callable[[float], None] = time.sleep,
    ) -> None:
        self.reader = reader
        self.sensor_mode = str(reader.sensor_mode)
        if self.sensor_mode not in _SUPPORTED_MODES:
            raise ValueError(f"unsupported Atlas sensor mode: {self.sensor_mode}")
        self.channels = channels or VisualPublishChannels()
        self.frames = frames or VisualPublishFrames()
        self.rate = float(rate)
        self.startup_delay = float(startup_delay)
        self.shutdown_delay = float(shutdown_delay)
        self.imu_lead_ns = round(float(imu_lead_seconds) * 1e9)
        self.publish_ground_truth = bool(publish_ground_truth)
        if self.rate < 0.0:
            raise ValueError("rate must be non-negative")
        if self.startup_delay < 0.0:
            raise ValueError("startup_delay must be non-negative")
        if self.shutdown_delay < 0.0:
            raise ValueError("shutdown_delay must be non-negative")
        if self.imu_lead_ns < 0:
            raise ValueError("imu_lead_seconds must be non-negative")
        self.node_name = node_name
        self.on_progress = on_progress
        self._clock = clock
        self._sleep = sleeper
        self._link = link
        self._node: Any | None = None
        self._writers: dict[str, Any] = {}

    @property
    def is_inertial(self) -> bool:
        return self.sensor_mode in _INERTIAL_MODES

    @property
    def expected_secondary_role(self) -> str | None:
        if self.sensor_mode in _STEREO_MODES:
            return "right"
        if self.sensor_mode == "rgbd":
            return "depth"
        return None

    def _resolve_link(self) -> Any:
        if self._link is not None:
            return self._link
        try:
            import autolink as resolved
        except ImportError as exc:
            raise ImportError(
                "autolink Python package is required; run inside autonomy runtime"
            ) from exc
        return resolved

    def bind(self) -> None:
        link = self._resolve_link()
        if hasattr(link, "init"):
            link.init(self.node_name)
        self._node = link.Node(self.node_name)
        self._writers = {
            "primary": self._node.create_writer(
                self.channels.primary, Image, qos_depth=20
            )
        }
        if self.expected_secondary_role == "right":
            self._writers["right"] = self._node.create_writer(
                self.channels.right, Image, qos_depth=20
            )
        elif self.expected_secondary_role == "depth":
            self._writers["depth"] = self._node.create_writer(
                self.channels.depth, Image, qos_depth=20
            )
        if self.is_inertial:
            self._writers["imu"] = self._node.create_writer(
                self.channels.imu, Imu, qos_depth=500
            )
        if self.publish_ground_truth:
            self._writers["ground_truth"] = self._node.create_writer(
                self.channels.ground_truth, Odometry, qos_depth=50
            )

    def shutdown(self) -> None:
        if self._node is not None and hasattr(self._node, "shutdown"):
            self._node.shutdown()
        self._node = None
        self._writers.clear()
        link = self._link
        if link is None:
            try:
                import autolink as link
            except ImportError:
                link = None
        if link is not None and hasattr(link, "shutdown"):
            link.shutdown()

    def _publish_imu_upto(self, imu_index: int, timestamp_ns: int) -> int:
        if not self.is_inertial:
            return imu_index
        samples = self.reader.imu
        writer = self._writers["imu"]
        while (
            imu_index < len(samples)
            and int(samples[imu_index].timestamp_ns) <= int(timestamp_ns)
        ):
            sample = samples[imu_index]
            writer.write(
                Messages.encode_inertial(
                    Messages.stamp_from_nanoseconds(sample.timestamp_ns),
                    self.frames.imu,
                    (0.0, 0.0, 0.0, 1.0),
                    sample.angular_velocity,
                    sample.linear_acceleration,
                )
            )
            imu_index += 1
        return imu_index

    def _publish_image(self, writer_name: str, sample: Any, frame_id: str) -> None:
        if sample.pixels is None:
            raise ValueError(f"image pixels were not loaded: {sample.path}")
        self._writers[writer_name].write(
            Messages.encode_image(
                sample.pixels,
                Messages.stamp_from_nanoseconds(sample.timestamp_ns),
                frame_id or sample.frame_id,
                sample.encoding,
            )
        )

    def _publish_ground_truth(self, frame: Any) -> None:
        if not self.publish_ground_truth:
            return
        sample = getattr(frame, "ground_truth", None)
        if sample is None:
            raise ValueError(
                f"ground truth is enabled but frame {frame.index} has no pose"
            )
        self._writers["ground_truth"].write(
            Messages.encode_odometry(
                Messages.stamp_from_nanoseconds(sample.timestamp_ns),
                self.frames.ground_truth,
                self.frames.ground_truth_child,
                sample.position,
                sample.orientation_xyzw,
            )
        )

    def _publish_frame(self, frame: Any) -> None:
        expected_role = self.expected_secondary_role
        if expected_role is None:
            if frame.secondary is not None or frame.secondary_role is not None:
                raise ValueError(
                    f"{self.sensor_mode} frame must not contain a secondary image"
                )
        elif frame.secondary is None or frame.secondary_role != expected_role:
            raise ValueError(
                f"{self.sensor_mode} frame requires secondary_role={expected_role}"
            )

        # Publish the pose first so Atlas can synchronize it with this image.
        self._publish_ground_truth(frame)
        self._publish_image("primary", frame.primary, self.frames.primary)
        if expected_role == "right":
            self._publish_image("right", frame.secondary, self.frames.right)
        elif expected_role == "depth":
            self._publish_image("depth", frame.secondary, self.frames.depth)

    def run(self, *, start: int = 0, max_frames: int | None = None) -> int:
        """Publish one finite slice and return the number of image frames."""
        start = int(start)
        if start < 0:
            raise ValueError("start must be non-negative")
        if max_frames is not None and int(max_frames) < 0:
            raise ValueError("max_frames must be non-negative")
        available = max(0, len(self.reader) - start)
        total = available
        if max_frames is not None:
            total = min(total, int(max_frames))
        if total == 0:
            return 0

        if self._node is None:
            self.bind()
        first_timestamp_ns = int(self.reader.frame_timestamps_ns[start])
        imu_index = 0
        if self.is_inertial:
            imu_times = [int(sample.timestamp_ns) for sample in self.reader.imu]
            imu_index = bisect.bisect_left(
                imu_times, first_timestamp_ns - self.imu_lead_ns
            )

        published = 0
        sensor_origin_ns: int | None = None
        wall_origin: float | None = None
        try:
            if self.startup_delay > 0.0:
                self._sleep(self.startup_delay)
            for frame in self.reader.iter_frames(
                start=start,
                max_frames=total,
            ):
                timestamp_ns = int(frame.primary.timestamp_ns)
                if self.rate > 0.0:
                    if sensor_origin_ns is None:
                        sensor_origin_ns = timestamp_ns
                        wall_origin = self._clock()
                    else:
                        assert wall_origin is not None
                        target = wall_origin + (
                            (timestamp_ns - sensor_origin_ns) * 1e-9 / self.rate
                        )
                        delay = target - self._clock()
                        if delay > 0.0:
                            self._sleep(delay)

                imu_index = self._publish_imu_upto(imu_index, timestamp_ns)
                self._publish_frame(frame)
                published += 1
                if self.on_progress is not None:
                    self.on_progress(published, total, timestamp_ns * 1e-9)
            if published > 0 and self.shutdown_delay > 0.0:
                self._sleep(self.shutdown_delay)
        finally:
            self.shutdown()
        return published
