#!/usr/bin/env python3
"""Evaluate an Atlas KITTI trajectory with rigid SE(3) alignment."""

import argparse
import csv
import json
import struct
import zlib
from pathlib import Path

import numpy as np


def load_poses(path: Path) -> np.ndarray:
    rows = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            values = np.fromstring(line, sep=" ")
            if values.size == 0:
                continue
            if values.size != 12 or not np.all(np.isfinite(values)):
                raise ValueError(f"{path}:{line_number}: expected 12 finite values")
            pose = np.eye(4)
            pose[:3, :] = values.reshape(3, 4)
            rows.append(pose)
    if not rows:
        raise ValueError(f"{path}: no poses")
    return np.stack(rows)


def rigid_alignment(source: np.ndarray, target: np.ndarray):
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    covariance = (source - source_mean).T @ (target - target_mean)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_mean - rotation @ source_mean
    return rotation, translation


def draw_line(image, x0, y0, x1, y1, color):
    height, width, _ = image.shape
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        if 0 <= x0 < width and 0 <= y0 < height:
            image[y0, x0] = color
        if x0 == x1 and y0 == y1:
            break
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


def map_points(points, left, top, width, height):
    minimum = points.min(axis=0)
    maximum = points.max(axis=0)
    span = np.maximum(maximum - minimum, 1.0e-9)
    scale = min((width - 20) / span[0], (height - 20) / span[1])
    offset = np.array(
        [
            left + (width - span[0] * scale) * 0.5,
            top + (height - span[1] * scale) * 0.5,
        ]
    )
    mapped = (points - minimum) * scale + offset
    mapped[:, 1] = top + height - (mapped[:, 1] - top)
    return np.rint(mapped).astype(int)


def draw_polyline(image, points, color):
    for first, second in zip(points[:-1], points[1:]):
        draw_line(image, *first, *second, color)


def write_png(path, image):
    height, width, _ = image.shape
    raw = b"".join(b"\x00" + image[row].tobytes() for row in range(height))

    def chunk(kind, data):
        return (
            struct.pack(">I", len(data))
            + kind
            + data
            + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
        )

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9))
    png += chunk(b"IEND", b"")
    path.write_bytes(png)


def create_plot(path, gt_xyz, aligned_xyz, errors, rmse):
    image = np.full((600, 1200, 3), 255, dtype=np.uint8)
    for x in (40, 600, 1160):
        draw_line(image, x, 30, x, 560, (210, 210, 210))
    draw_line(image, 40, 560, 1160, 560, (210, 210, 210))

    trajectories = np.concatenate(
        [gt_xyz[:, (0, 2)], aligned_xyz[:, (0, 2)]], axis=0
    )
    mapped = map_points(trajectories, 50, 40, 520, 500)
    draw_polyline(image, mapped[: len(gt_xyz)], (40, 170, 40))
    draw_polyline(image, mapped[len(gt_xyz) :], (220, 50, 50))

    maximum = max(float(errors.max()), rmse, 1.0e-9)
    error_points = np.column_stack(
        [
            np.linspace(630, 1150, len(errors)),
            550 - errors / maximum * 480,
        ]
    )
    draw_polyline(image, np.rint(error_points).astype(int), (40, 80, 220))
    rmse_y = int(round(550 - rmse / maximum * 480))
    draw_line(image, 630, rmse_y, 1150, rmse_y, (220, 50, 50))
    write_png(path, image)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("ground_truth", type=Path)
    parser.add_argument("estimate", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()

    ground_truth = load_poses(args.ground_truth)
    estimate = load_poses(args.estimate)
    count = min(len(ground_truth), len(estimate))
    if count < 3:
        raise ValueError("at least three associated poses are required")
    if len(ground_truth) != len(estimate):
        print(
            f"warning: pose counts differ; evaluating first {count} pairs "
            f"(ground truth={len(ground_truth)}, estimate={len(estimate)})"
        )

    ground_truth = ground_truth[:count]
    estimate = estimate[:count]
    gt_xyz = ground_truth[:, :3, 3]
    estimated_xyz = estimate[:, :3, 3]
    rotation, translation = rigid_alignment(estimated_xyz, gt_xyz)
    aligned_xyz = (rotation @ estimated_xyz.T).T + translation
    errors = np.linalg.norm(aligned_xyz - gt_xyz, axis=1)

    stats = {
        "associated_poses": int(count),
        "alignment": "SE3",
        "scale_fixed": True,
        "ate_rmse_m": float(np.sqrt(np.mean(errors**2))),
        "ate_mean_m": float(np.mean(errors)),
        "ate_median_m": float(np.median(errors)),
        "ate_std_m": float(np.std(errors)),
        "ate_min_m": float(np.min(errors)),
        "ate_max_m": float(np.max(errors)),
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    with (args.output_dir / "metrics.json").open("w", encoding="utf-8") as stream:
        json.dump(stats, stream, indent=2, sort_keys=True)
        stream.write("\n")

    with (args.output_dir / "aligned_trajectory.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "frame",
                "gt_x",
                "gt_y",
                "gt_z",
                "estimate_x",
                "estimate_y",
                "estimate_z",
                "ate_m",
            ]
        )
        for index in range(count):
            writer.writerow(
                [index, *gt_xyz[index], *aligned_xyz[index], errors[index]]
            )

    create_plot(
        args.output_dir / "trajectory_and_ate.png",
        gt_xyz,
        aligned_xyz,
        errors,
        stats["ate_rmse_m"],
    )
    print(json.dumps(stats, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
