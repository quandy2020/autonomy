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

"""Publish dataset sensor streams on autolink for Argus subscribers."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Callable

from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.sensor_msgs.imu_pb2 import Imu
from automsgs.msgs.sensor_msgs.point_cloud2_pb2 import PointCloud2

from autonomy.tools.datasets.messages import Messages
from autonomy.tools.datasets.readers.spires import SpiresReader


@dataclass
class PublishChannels:
    imu: str = "/imu"
    points: str = "/points"
    image: str = "/camera/rgb/image_raw"


@dataclass
class PublishFrames:
    imu: str = "imu_link"
    lidar: str = "lidar_link"
    camera: str = "camera_link"


class DatasetPublisher:
    """Replay a Spires sequence onto Argus-aligned autolink topics."""

    def __init__(
        self,
        reader: SpiresReader,
        *,
        channels: PublishChannels | None = None,
        frames: PublishFrames | None = None,
        rate: float = 1.0,
        publish_image: bool = True,
        link: Any | None = None,
        node_name: str = "dataset_publisher",
        on_progress: Callable[[int, int, float], None] | None = None,
    ) -> None:
        self.reader = reader
        self.channels = channels or PublishChannels()
        self.frames = frames or PublishFrames()
        self.rate = max(0.0, float(rate))
        self.publish_image = bool(publish_image) and reader.load_images
        self.node_name = node_name
        self.on_progress = on_progress
        self._link = link
        self._node: Any | None = None
        self._writers: dict[str, Any] = {}

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
            "imu": self._node.create_writer(
                self.channels.imu, Imu, qos_depth=200
            ),
            "points": self._node.create_writer(
                self.channels.points, PointCloud2, qos_depth=10
            ),
        }
        if self.publish_image:
            self._writers["image"] = self._node.create_writer(
                self.channels.image, Image, qos_depth=10
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
            try:
                link.shutdown()
            except Exception:
                pass

    def _publish_imu_upto(self, imu_i: int, t_lidar: float) -> int:
        writer = self._writers["imu"]
        imu = self.reader.imu
        while imu_i < len(imu) and imu[imu_i].t <= t_lidar:
            sample = imu[imu_i]
            stamp = Messages.stamp_from_seconds(sample.t)
            writer.write(
                Messages.encode_inertial(
                    stamp,
                    self.frames.imu,
                    (0.0, 0.0, 0.0, 1.0),
                    sample.gyr,
                    sample.acc,
                )
            )
            imu_i += 1
        return imu_i

    def run(
        self,
        *,
        start: int = 0,
        max_frames: int | None = None,
        loop: bool = False,
    ) -> int:
        """Publish frames; return number of lidar frames published."""
        if self._node is None:
            self.bind()
        published = 0
        try:
            while True:
                imu_i = 0
                prev_t: float | None = None
                # Skip IMU far before the first published lidar (keep ~0.5 s lead).
                if 0 <= start < len(self.reader):
                    t0 = self.reader.lidar_times[start]
                    while (
                        imu_i < len(self.reader.imu)
                        and self.reader.imu[imu_i].t < t0 - 0.5
                    ):
                        imu_i += 1

                for frame in self.reader.iter_frames(
                    start=start,
                    max_frames=max_frames,
                    load_image=self.publish_image,
                ):
                    imu_i = self._publish_imu_upto(imu_i, frame.timestamp)
                    stamp = Messages.stamp_from_seconds(frame.timestamp)
                    self._writers["points"].write(
                        Messages.encode_point_cloud2(
                            frame.xyz,
                            stamp,
                            self.frames.lidar,
                            intensity=frame.intensity,
                        )
                    )
                    if (
                        self.publish_image
                        and "image" in self._writers
                        and frame.image is not None
                    ):
                        img_stamp = Messages.stamp_from_seconds(
                            frame.image_timestamp or frame.timestamp
                        )
                        self._writers["image"].write(
                            Messages.encode_image(
                                frame.image,
                                img_stamp,
                                self.frames.camera,
                                "rgb8",
                            )
                        )
                    published += 1
                    if self.on_progress is not None:
                        self.on_progress(published, len(self.reader), frame.timestamp)

                    if prev_t is not None and self.rate > 0.0:
                        dt = max(0.0, float(frame.timestamp - prev_t)) / self.rate
                        if dt > 0.0:
                            time.sleep(dt)
                    prev_t = frame.timestamp

                if not loop:
                    break
        finally:
            self.shutdown()
        return published
