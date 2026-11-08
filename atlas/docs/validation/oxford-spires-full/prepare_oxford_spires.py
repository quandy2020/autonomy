#!/usr/bin/env python3
"""Convert raw Oxford Spires LiDAR and IMU files to the replay text format."""

import argparse
import csv
from pathlib import Path

import numpy as np


def read_binary_pcd(path):
    """Read scalar x/y/z/intensity/timestamp fields from a binary PCD."""
    header = {}
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"{path}: missing DATA header")
            text = line.decode("ascii").strip()
            if text and not text.startswith("#"):
                pieces = text.split()
                header[pieces[0]] = pieces[1:]
            if text.startswith("DATA "):
                if text != "DATA binary":
                    raise ValueError(f"{path}: expected binary PCD")
                payload_offset = stream.tell()
                break

    fields = header["FIELDS"]
    sizes = list(map(int, header["SIZE"]))
    types = header["TYPE"]
    counts = list(map(int, header.get("COUNT", ["1"] * len(fields))))
    offsets = []
    point_step = 0
    for size, count in zip(sizes, counts):
        offsets.append(point_step)
        point_step += size * count

    scalar_types = {
        "F": {4: "<f4", 8: "<f8"},
        "U": {1: "u1", 2: "<u2", 4: "<u4", 8: "<u8"},
        "I": {1: "i1", 2: "<i2", 4: "<i4", 8: "<i8"},
    }
    required_fields = {"x", "y", "z", "intensity", "timestamp"}
    names = []
    formats = []
    field_offsets = []
    for name, size, field_type, count, offset in zip(
        fields, sizes, types, counts, offsets
    ):
        if name not in required_fields:
            continue
        if count != 1:
            raise ValueError(f"{path}: field {name} must be scalar")
        names.append(name)
        formats.append(scalar_types[field_type][size])
        field_offsets.append(offset)
    if set(names) != required_fields:
        raise ValueError(f"{path}: missing fields {required_fields - set(names)}")

    point_count = int(header["POINTS"][0])
    point_type = np.dtype(
        {"names": names, "formats": formats, "offsets": field_offsets,
         "itemsize": point_step}
    )
    with path.open("rb") as stream:
        stream.seek(payload_offset)
        payload = stream.read(point_count * point_step)
    if len(payload) != point_count * point_step:
        raise ValueError(f"{path}: truncated binary point data")
    return np.frombuffer(payload, dtype=point_type, count=point_count)


def read_scan(path, output_path):
    """Write one raw LiDAR sweep with relative point times in seconds."""
    scan_start_time = float(path.stem)
    cloud = read_binary_pcd(path)
    point_times = cloud["timestamp"].astype(np.float64)
    positions = np.column_stack((cloud["x"], cloud["y"], cloud["z"]))
    intensities = cloud["intensity"].astype(np.float64)
    valid = np.isfinite(point_times) & np.isfinite(positions).all(axis=1)
    valid &= np.isfinite(intensities)
    valid &= point_times >= scan_start_time - 1e-4
    valid &= point_times <= scan_start_time + 0.11
    relative_times = np.maximum(point_times[valid] - scan_start_time, 0.0)
    rows = np.column_stack((positions[valid], intensities[valid], relative_times))
    np.savetxt(output_path, rows, fmt="%.7g %.7g %.7g %.7g %.9g")


def convert_imu(source_path, output_path):
    """Preserve raw IMU measurements and their native sensor axes."""
    with source_path.open(newline="") as source, output_path.open("w") as target:
        reader = csv.DictReader(source)
        target.write("# timestamp wx wy wz ax ay az; native IMU sensor axes\n")
        for sample in reader:
            timestamp = float(sample["secs"]) + float(sample["nsecs"]) * 1e-9
            angular_velocity = [float(sample[f"ang_vel_{axis}"]) for axis in "xyz"]
            acceleration = [float(sample[f"acc_{axis}"]) for axis in "xyz"]
            values = angular_velocity + acceleration
            target.write(
                f"{timestamp:.9f} "
                + " ".join(f"{value:.12g}" for value in values)
                + "\n"
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence-directory", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--maximum-scans", type=int)
    arguments = parser.parse_args()

    lidar_directory = arguments.sequence_directory / "raw" / "lidar-clouds"
    imu_path = arguments.sequence_directory / "raw" / "imu.csv"
    if not lidar_directory.is_dir() or not imu_path.is_file():
        parser.error("sequence directory must contain raw/lidar-clouds and raw/imu.csv")
    if arguments.maximum_scans is not None and arguments.maximum_scans < 1:
        parser.error("--maximum-scans must be positive")
    if arguments.output_directory.exists():
        parser.error(f"output directory already exists: {arguments.output_directory}")

    scans = sorted(lidar_directory.glob("*.pcd"))
    if arguments.maximum_scans is not None:
        scans = scans[: arguments.maximum_scans]
    if not scans:
        parser.error(f"no PCD scans found in {lidar_directory}")

    output_directory = arguments.output_directory
    cloud_directory = output_directory / "clouds"
    cloud_directory.mkdir(parents=True)
    with (output_directory / "scans.txt").open("w") as scan_index:
        scan_index.write(
            "# timestamp cloud_path; native LiDAR axes, point time seconds\n"
        )
        for index, scan_path in enumerate(scans):
            relative_path = f"clouds/{index:06d}.txt"
            read_scan(scan_path, output_directory / relative_path)
            scan_index.write(f"{float(scan_path.stem):.9f} {relative_path}\n")

    convert_imu(imu_path, output_directory / "imu.txt")
    print(f"converted {len(scans)} raw scans and IMU data to {output_directory}")


if __name__ == "__main__":
    main()
