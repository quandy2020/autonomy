#!/usr/bin/env python3
"""Convert a ROS 1/2 bag to the voxel_slam text dataset.

Output layout (read by FileReplaySource):

    <out>/imu.txt                 timestamp wx wy wz ax ay az
    <out>/scans.txt               timestamp scans/NNNNNN.txt
    <out>/scans/NNNNNN.txt        x y z intensity relative_time

Timestamps are seconds. relative_time is seconds after the scan start.
Requires the pure-Python ``rosbags`` package.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path


_FLOAT_TYPES = {7: ("<f", 4), 8: ("<d", 8)}
_TIME_FIELDS = ("time", "timestamp", "t", "relative_time", "curvature")


def _stamp_seconds(stamp, fallback_ns: int) -> float:
    if stamp is None:
        return fallback_ns * 1e-9
    if isinstance(stamp, (int, float)):
        value = float(stamp)
        return value * 1e-9 if value > 1e12 else value
    sec = getattr(stamp, "sec", getattr(stamp, "secs", None))
    if sec is None:
        return fallback_ns * 1e-9
    nsec = getattr(stamp, "nanosec", getattr(stamp, "nsecs", getattr(stamp, "nsec", 0)))
    return float(sec) + float(nsec) * 1e-9


def _field_map(msg):
    return {field.name: field for field in msg.fields}


def _read_numeric(data: bytes, base: int, field) -> float:
    fmt = _FLOAT_TYPES.get(field.datatype)
    if fmt is None:
        raise ValueError(f"unsupported PointField datatype {field.datatype} for {field.name}")
    pattern, _size = fmt
    return struct.unpack_from(pattern, data, base + field.offset)[0]


def points_from_cloud(msg) -> list[tuple[float, float, float, float, float]]:
    fields = _field_map(msg)
    for axis in ("x", "y", "z"):
        if axis not in fields:
            raise ValueError("PointCloud2 is missing x/y/z")
    data = bytes(msg.data)
    step = int(msg.point_step)
    count = int(msg.width) * int(msg.height)
    intensity_field = fields.get("intensity")
    time_field = next((fields[name] for name in _TIME_FIELDS if name in fields), None)
    points = []
    times = []
    for index in range(count):
        base = index * step
        x = _read_numeric(data, base, fields["x"])
        y = _read_numeric(data, base, fields["y"])
        z = _read_numeric(data, base, fields["z"])
        intensity = _read_numeric(data, base, intensity_field) if intensity_field else 0.0
        relative = _read_numeric(data, base, time_field) if time_field else 0.0
        points.append((x, y, z, intensity, relative))
        times.append(relative)
    if times and min(times) > 1.0:
        origin = min(times)
        points = [(x, y, z, intensity, t - origin) for x, y, z, intensity, t in points]
    return points


def write_dataset(output: Path, imus, scans) -> None:
    scans_dir = output / "scans"
    scans_dir.mkdir(parents=True, exist_ok=True)
    imus = sorted(imus, key=lambda sample: sample[0])
    scans = sorted(scans, key=lambda sample: sample[0])
    imu_lines = [
        f"{t:.9f} {wx:.9f} {wy:.9f} {wz:.9f} {ax:.9f} {ay:.9f} {az:.9f}"
        for t, wx, wy, wz, ax, ay, az in imus
    ]
    (output / "imu.txt").write_text("\n".join(imu_lines) + ("\n" if imu_lines else ""))
    index_lines = []
    for index, (timestamp, points) in enumerate(scans):
        relative = f"scans/{index:06d}.txt"
        body = [
            f"{x:.6f} {y:.6f} {z:.6f} {intensity:.6f} {relative_time:.9f}"
            for x, y, z, intensity, relative_time in points
        ]
        (output / relative).write_text("\n".join(body) + ("\n" if body else ""))
        index_lines.append(f"{timestamp:.9f} {relative}")
    (output / "scans.txt").write_text("\n".join(index_lines) + ("\n" if index_lines else ""))


def convert_bag(bag: Path, output: Path, lidar_topics: set[str], imu_topics: set[str]) -> tuple[int, int]:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:
        raise SystemExit("rosbag_to_voxel_slam needs the 'rosbags' package") from exc

    imus = []
    scans = []
    seen = set()
    with AnyReader([bag]) as reader:
        selected = [conn for conn in reader.connections if conn.topic in lidar_topics or conn.topic in imu_topics]
        if not selected:
            available = sorted({conn.topic for conn in reader.connections})
            raise SystemExit("none of the requested topics are in the bag. have: " + ", ".join(available))
        for conn in reader.connections:
            seen.add(conn.topic)
        for conn, timestamp, raw in reader.messages(connections=selected):
            msg = reader.deserialize(raw, conn.msgtype)
            stamp = _stamp_seconds(getattr(getattr(msg, "header", None), "stamp", None), timestamp)
            if conn.topic in imu_topics:
                gyro = msg.angular_velocity
                accel = msg.linear_acceleration
                imus.append((stamp, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z))
            else:
                scans.append((stamp, points_from_cloud(msg)))
    if not scans and not imus:
        raise SystemExit("bag produced no IMU samples or scans. topics: " + ", ".join(sorted(seen)))
    write_dataset(output, imus, scans)
    return len(imus), len(scans)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 1 .bag or ROS 2 bag directory")
    parser.add_argument("output", type=Path, help="dataset directory to create")
    parser.add_argument("--lidar-topic", action="append", default=["/points_raw"])
    parser.add_argument("--imu-topic", action="append", default=["/imu/data"])
    args = parser.parse_args(argv)
    imu_count, scan_count = convert_bag(args.bag, args.output, set(args.lidar_topic), set(args.imu_topic))
    print(f"wrote {scan_count} scans and {imu_count} imu samples to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
