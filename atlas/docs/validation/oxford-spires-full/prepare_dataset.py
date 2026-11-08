#!/usr/bin/env python3
"""Stage text data already expressed in native Oxford sensor frames."""

import argparse
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-directory", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--maximum-scans", type=int)
    arguments = parser.parse_args()
    source = arguments.input_directory
    output = arguments.output_directory
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite {output}")
    imu_path = source / "imu.txt"
    imu_lines = imu_path.read_text().splitlines()
    header = "\n".join(line for line in imu_lines if line.lstrip().startswith("#"))
    if "native IMU sensor axes" not in header:
        raise ValueError(
            f"{imu_path} must declare 'native IMU sensor axes'; "
            "base-frame IMU measurements are not accepted"
        )
    output.mkdir(parents=True)
    cloud_directory = output / "clouds"
    cloud_directory.mkdir()
    scan_entries = []
    for line in (source / "scans.txt").read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        timestamp, relative_path = line.split(maxsplit=1)
        scan_entries.append((timestamp, relative_path))
    if arguments.maximum_scans is not None:
        if arguments.maximum_scans < 1:
            raise ValueError("--maximum-scans must be positive")
        scan_entries = scan_entries[:arguments.maximum_scans]

    with (output / "scans.txt").open("w") as scan_index:
        scan_index.write(
            "# timestamp cloud_path; native LiDAR coordinates and relative time in milliseconds\n"
        )
        for scan_index_number, (timestamp, relative_path) in enumerate(scan_entries):
            input_path = source / relative_path
            points = np.loadtxt(input_path, comments="#", ndmin=2)
            if points.shape[1] != 5:
                raise ValueError(f"Expected five point fields in {input_path}")
            output_relative_path = f"clouds/{scan_index_number:06d}.txt"
            np.savetxt(cloud_directory / f"{scan_index_number:06d}.txt", points,
                       fmt="%.12g")
            scan_index.write(f"{timestamp} {output_relative_path}\n")

    with imu_path.open() as input_file, (output / "imu.txt").open("w") as output_file:
        output_file.write(
            "# timestamp wx wy wz ax ay az; native IMU axes\n"
        )
        for line in input_file:
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            values = np.fromstring(line, sep=" ")
            if values.size != 7:
                raise ValueError(f"Expected seven IMU values: {line!r}")
            output_file.write(
                f"{values[0]:.9f} "
                + " ".join(
                    f"{value:.12g}"
                    for value in values[1:7]
                )
                + "\n"
            )
    print(f"staged {len(scan_entries)} scans at {output}")


if __name__ == "__main__":
    main()
