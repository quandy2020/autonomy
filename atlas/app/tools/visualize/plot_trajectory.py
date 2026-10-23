#!/usr/bin/env python3
"""Plot a TUM trajectory (timestamp tx ty tz qx qy qz qw) to SVG or PNG."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def load_tum(path: Path) -> list[tuple[float, float, float, float]]:
    poses = []
    for line_number, raw in enumerate(path.read_text().splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 4:
            raise SystemExit(f"{path}:{line_number}: expected timestamp tx ty tz")
        timestamp, x, y, z = (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))
        poses.append((timestamp, x, y, z))
    if not poses:
        raise SystemExit(f"{path} has no poses")
    return poses


def _bounds(series: list[list[tuple[float, float, float, float]]]):
    xs = [pose[1] for poses in series for pose in poses]
    ys = [pose[2] for poses in series for pose in poses]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span = max(max_x - min_x, max_y - min_y, 1e-6)
    pad = 0.05 * span
    return min_x - pad, max_x + pad, min_y - pad, max_y + pad


def write_svg(path: Path, series: list[tuple[str, str, list]]) -> None:
    min_x, max_x, min_y, max_y = _bounds([poses for _name, _color, poses in series])
    width, height = 800.0, 800.0

    def project(x: float, y: float) -> tuple[float, float]:
        px = (x - min_x) / (max_x - min_x) * width
        py = height - (y - min_y) / (max_y - min_y) * height
        return px, py

    body = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{int(width)}" height="{int(height)}" viewBox="0 0 {int(width)} {int(height)}">',
        '<rect width="100%" height="100%" fill="white"/>',
    ]
    for name, color, poses in series:
        points = " ".join(f"{project(x, y)[0]:.2f},{project(x, y)[1]:.2f}" for _t, x, y, _z in poses)
        body.append(f'<polyline fill="none" stroke="{color}" stroke-width="1.5" points="{points}"/>')
        body.append(f"<!-- {name} {len(poses)} poses -->")
    body.append("</svg>")
    path.write_text("\n".join(body) + "\n")


def write_png(path: Path, series: list[tuple[str, str, list]]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("PNG output needs matplotlib; use a .svg path instead") from exc
    figure, axis = plt.subplots()
    for name, color, poses in series:
        axis.plot([pose[1] for pose in poses], [pose[2] for pose in poses], color=color, label=name)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.legend()
    figure.savefig(path, dpi=120)
    plt.close(figure)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path, help="estimated TUM trajectory")
    parser.add_argument("--ground-truth", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    series = [("estimate", "#1f4e79", load_tum(args.trajectory))]
    if args.ground_truth is not None:
        series.append(("ground truth", "#c45911", load_tum(args.ground_truth)))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.suffix.lower() == ".png":
        write_png(args.output, series)
    else:
        write_svg(args.output, series)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
