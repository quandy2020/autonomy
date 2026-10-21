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
from pathlib import Path
from typing import Any, Callable

from automsgs.msgs.nav_msgs.odometry_pb2 import Odometry
from automsgs.msgs.nav_msgs.path_pb2 import Path as PathMsg
from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.sensor_msgs.imu_pb2 import Imu
from automsgs.msgs.sensor_msgs.point_cloud2_pb2 import PointCloud2

from autonomy.tools.datasets.messages import Messages
from autonomy.tools.datasets.readers.spires import SpiresReader
from autonomy.tools.datasets.tum import (
    TumIndex,
    TumPose,
    load_tum,
    relative_to_origin,
    resolve_spires_gt,
)


@dataclass
class PublishChannels:
    imu: str = "/imu"
    points: str = "/points"
    image: str = "/camera/rgb/image_raw"
    gt_odom: str = "/gt/odometry"
    gt_path: str = "/gt/path"


@dataclass
class PublishFrames:
    imu: str = "imu_link"
    lidar: str = "lidar_link"
    camera: str = "camera_link"
    gt: str = "map"
    gt_child: str = "gt_link"


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
        publish_gt: bool = True,
        gt_file: Path | None = None,
        gt_max_dt: float = 0.5,
        gt_path_stride: int = 1,
        gt_path_every: int = 1,
        # Always publish absolute TUM GT by default. Never rebase / warp GT
        # (gt_zero_start would transform truth — treat as cheating for overlay).
        gt_zero_start: bool = False,
        link: Any | None = None,
        node_name: str = "dataset_publisher",
        on_progress: Callable[[int, int, float], None] | None = None,
    ) -> None:
        self.reader = reader
        self.channels = channels or PublishChannels()
        self.frames = frames or PublishFrames()
        self.rate = max(0.0, float(rate))
        self.publish_image = bool(publish_image) and reader.load_images
        self.publish_gt = bool(publish_gt)
        self.gt_file = gt_file
        self.gt_max_dt = float(gt_max_dt)
        self.gt_path_stride = max(1, int(gt_path_stride))
        # 1 = publish growing /gt/path every lidar frame (synced with Argus traj).
        self.gt_path_every = max(1, int(gt_path_every))
        # Zero first GT pose → identity so /gt/path overlays Argus (also ~origin).
        self.gt_zero_start = bool(gt_zero_start)
        self.node_name = node_name
        self.on_progress = on_progress
        self._link = link
        self._node: Any | None = None
        self._writers: dict[str, Any] = {}
        self._gt: TumIndex | None = None
        self._gt_path: Path | None = None
        self._gt_origin: TumPose | None = None
        self._gt_path_last_t: float | None = None
        self._gt_path_live: Any | None = None
        self._gt_path_end: int = 0

        if self.publish_gt:
            resolved = resolve_spires_gt(reader.seq_root, gt_file)
            if resolved is None:
                self.publish_gt = False
            else:
                self._gt_path = resolved
                self._gt = TumIndex(load_tum(resolved))

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
        if self.publish_gt and self._gt is not None:
            self._writers["gt_odom"] = self._node.create_writer(
                self.channels.gt_odom, Odometry, qos_depth=50
            )
            self._writers["gt_path"] = self._node.create_writer(
                self.channels.gt_path, PathMsg, qos_depth=2
            )
            # Grow /gt/path with playback time (same idea as /argus/trajectory).
            self._gt_path_live = PathMsg()
            self._gt_path_end = 0

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

    def _publish_imu_upto(self, imu_i: int, t_end: float) -> int:
        writer = self._writers["imu"]
        imu = self.reader.imu
        while imu_i < len(imu) and imu[imu_i].t <= float(t_end):
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

    def _gt_xyz_quat(
        self, pose: TumPose
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        """GT pose for publish; optionally expressed relative to first used pose."""
        if not self.gt_zero_start:
            return pose.xyz, pose.quat_xyzw
        if self._gt_origin is None:
            self._gt_origin = pose
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        return relative_to_origin(pose, self._gt_origin)

    def _append_gt_path_for_lidar(
        self, t_lidar: float, stamp: tuple[int, int]
    ) -> bool:
        """Append one GT pose at this lidar time (same cadence as /argus/trajectory)."""
        if self._gt is None or self._gt_path_live is None:
            return False
        pose = self._gt.nearest(t_lidar, max_dt=self.gt_max_dt)
        if pose is None:
            Messages.set_header(
                self._gt_path_live.header, stamp, self.frames.gt
            )
            return False
        # Skip duplicate nearest pose (lidar faster than GT rate).
        if (
            self._gt_path_last_t is not None
            and abs(float(pose.t) - float(self._gt_path_last_t)) < 1e-9
        ):
            Messages.set_header(
                self._gt_path_live.header, stamp, self.frames.gt
            )
            return False
        if self.gt_path_stride > 1 and self._gt_path_end % self.gt_path_stride != 0:
            self._gt_path_end += 1
            self._gt_path_last_t = float(pose.t)
            Messages.set_header(
                self._gt_path_live.header, stamp, self.frames.gt
            )
            return False

        xyz, quat = self._gt_xyz_quat(pose)
        pose_stamp = Messages.stamp_from_seconds(t_lidar)
        ps = self._gt_path_live.poses.add()
        Messages.set_header(ps.header, pose_stamp, self.frames.gt)
        ps.pose.position.x = float(xyz[0])
        ps.pose.position.y = float(xyz[1])
        ps.pose.position.z = float(xyz[2])
        ps.pose.orientation.x = float(quat[0])
        ps.pose.orientation.y = float(quat[1])
        ps.pose.orientation.z = float(quat[2])
        ps.pose.orientation.w = float(quat[3])
        self._gt_path_end += 1
        self._gt_path_last_t = float(pose.t)
        Messages.set_header(self._gt_path_live.header, stamp, self.frames.gt)
        return True

    def _publish_gt(self, t_lidar: float, *, frame_index: int) -> None:
        if self._gt is None or "gt_odom" not in self._writers:
            return
        pose = self._gt.nearest(t_lidar, max_dt=self.gt_max_dt)
        if pose is None:
            return
        stamp = Messages.stamp_from_seconds(t_lidar)
        xyz, quat = self._gt_xyz_quat(pose)
        self._writers["gt_odom"].write(
            Messages.encode_odometry(
                stamp,
                self.frames.gt,
                self.frames.gt_child,
                xyz,
                quat,
            )
        )
        if "gt_path" not in self._writers or self._gt_path_live is None:
            return
        grown = self._append_gt_path_for_lidar(t_lidar, stamp)
        # Publish when grown, on first frame, or every N frames (late Autoviz).
        if grown or frame_index == 0 or (frame_index + 1) % self.gt_path_every == 0:
            if self._gt_path_live.poses:
                self._writers["gt_path"].write(self._gt_path_live)

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
                # Absolute timeline: wall clock tracks sensor time / rate, so
                # encode+publish cost is absorbed instead of stacking on sleep.
                t0_sensor: float | None = None
                t0_wall: float | None = None
                if self._gt_path_live is not None:
                    self._gt_path_live = PathMsg()
                    self._gt_path_end = 0
                    self._gt_path_last_t = None
                    self._gt_origin = None
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
                    # Match offline_spires_livo: IMU through lidar header
                    # before the cloud, then lead past scan end (~0.1s) so
                    # SyncOnlyLio is not stuck waiting for IMU.
                    imu_i = self._publish_imu_upto(
                        imu_i, float(frame.timestamp) + 1e-4
                    )
                    stamp = Messages.stamp_from_seconds(frame.timestamp)
                    # GT before /points so Argus can SE(3)-align on the same
                    # t_lidar (otherwise TryAlignToGt often locks onto the
                    # previous frame and the green traj is rigidly offset).
                    self._publish_gt(frame.timestamp, frame_index=published)
                    self._writers["points"].write(
                        Messages.encode_point_cloud2(
                            frame.xyz,
                            stamp,
                            self.frames.lidar,
                            intensity=frame.intensity,
                            time_ms=frame.time_ms,
                        )
                    )
                    imu_i = self._publish_imu_upto(
                        imu_i, float(frame.timestamp) + 0.12
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

                    if self.rate > 0.0:
                        if t0_sensor is None:
                            t0_sensor = float(frame.timestamp)
                            t0_wall = time.perf_counter()
                        else:
                            assert t0_wall is not None
                            target = t0_wall + (
                                float(frame.timestamp) - t0_sensor
                            ) / self.rate
                            delay = target - time.perf_counter()
                            if delay > 0.0:
                                time.sleep(delay)

                if not loop:
                    break
        finally:
            self.shutdown()
        return published
