#!/usr/bin/env python3
"""Convert an MCAP file with ROS 2 messages to the voxel_slam text dataset.

See rosbag_to_voxel_slam.py for the output layout. Requires ``mcap-ros2-support``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from rosbag_to_voxel_slam import points_from_cloud, write_dataset


def _log_time_ns(record) -> int:
    for name in ("log_time_ns", "publish_time_ns", "log_time"):
        value = getattr(record, name, None)
        if value is not None:
            return int(value)
    return 0


def convert_mcap(path: Path, output: Path, lidar_topics: set[str], imu_topics: set[str]) -> tuple[int, int]:
    try:
        from mcap_ros2.reader import read_ros2_messages
    except ImportError as exc:
        raise SystemExit("mcap_to_voxel_slam needs the 'mcap-ros2-support' package") from exc

    from rosbag_to_voxel_slam import _stamp_seconds

    imus = []
    scans = []
    seen = set()
    topics = sorted(lidar_topics | imu_topics)
    for record in read_ros2_messages(str(path), topics=topics):
        seen.add(record.channel.topic)
        msg = record.ros_msg
        stamp = _stamp_seconds(getattr(getattr(msg, "header", None), "stamp", None), _log_time_ns(record))
        topic = record.channel.topic
        if topic in imu_topics:
            gyro = msg.angular_velocity
            accel = msg.linear_acceleration
            imus.append((stamp, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z))
        elif topic in lidar_topics:
            scans.append((stamp, points_from_cloud(msg)))
    if not scans and not imus:
        raise SystemExit(
            "mcap produced no IMU samples or scans. "
            + (f"saw: {', '.join(sorted(seen))}" if seen else "no matching channels")
        )
    write_dataset(output, imus, scans)
    return len(imus), len(scans)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mcap", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--lidar-topic", action="append", default=["/points_raw"])
    parser.add_argument("--imu-topic", action="append", default=["/imu/data"])
    args = parser.parse_args(argv)
    imu_count, scan_count = convert_mcap(args.mcap, args.output, set(args.lidar_topic), set(args.imu_topic))
    print(f"wrote {scan_count} scans and {imu_count} imu samples to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
