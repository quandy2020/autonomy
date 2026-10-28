#!/usr/bin/env python3
"""Render a top-down occupancy image from a voxel_slam dataset directory.

Reads scans listed in scans.txt (x y z intensity relative_time) and writes a
binary PGM. Pixel value is the clipped point count in that column.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def load_points(dataset: Path) -> list[tuple[float, float, float]]:
    index = dataset / "scans.txt"
    if not index.is_file():
        raise SystemExit(f"missing {index}")
    points = []
    for raw in index.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        _timestamp, relative = line.split()[:2]
        scan_path = dataset / relative
        for point_line in scan_path.read_text().splitlines():
            point_line = point_line.strip()
            if not point_line or point_line.startswith("#"):
                continue
            x, y, z, *_rest = point_line.split()
            points.append((float(x), float(y), float(z)))
    if not points:
        raise SystemExit(f"{dataset} has no points")
    return points


def render(points: list[tuple[float, float, float]], resolution: float) -> tuple[int, int, bytearray, float, float]:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    width = max(1, int((max_x - min_x) / resolution) + 1)
    height = max(1, int((max_y - min_y) / resolution) + 1)
    counts = [0] * (width * height)
    for x, y, _z in points:
        col = min(width - 1, int((x - min_x) / resolution))
        row = min(height - 1, int((y - min_y) / resolution))
        counts[row * width + col] += 1
    peak = max(counts)
    pixels = bytearray(width * height)
    for index, count in enumerate(counts):
        pixels[index] = 0 if count == 0 else min(255, 40 + int(215 * count / peak))
    return width, height, pixels, min_x, min_y


def write_pgm(path: Path, width: int, height: int, pixels: bytearray) -> None:
    # Image rows grow downward; map +y grows upward.
    flipped = bytearray()
    for row in range(height - 1, -1, -1):
        flipped.extend(pixels[row * width : (row + 1) * width])
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    path.write_bytes(header + bytes(flipped))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resolution", type=float, default=0.1, help="meters per pixel")
    args = parser.parse_args(argv)
    if args.resolution <= 0.0:
        raise SystemExit("--resolution must be > 0")
    points = load_points(args.dataset)
    width, height, pixels, origin_x, origin_y = render(points, args.resolution)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_pgm(args.output, width, height, pixels)
    print(
        f"wrote {args.output} ({width}x{height}, {len(points)} points, "
        f"origin {origin_x:.3f} {origin_y:.3f}, resolution {args.resolution} m)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
