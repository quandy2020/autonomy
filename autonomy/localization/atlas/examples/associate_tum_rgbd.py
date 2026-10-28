#!/usr/bin/env python3
"""Associate TUM RGB-D color and depth streams by timestamp.

The generated rows use the format expected by atlas_rgbd_tum:

    rgb_timestamp rgb_path depth_timestamp depth_path
"""

from __future__ import annotations

import argparse
import bisect
import dataclasses
import math
import pathlib
import sys
from collections.abc import Sequence


@dataclasses.dataclass(frozen=True)
class TimestampedPath:
    timestamp: float
    timestamp_text: str
    path: str
    source_index: int


def read_file_list(path: pathlib.Path) -> list[TimestampedPath]:
    """Reads a TUM timestamp/path list and returns it in timestamp order."""
    entries: list[TimestampedPath] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.replace(",", " ").split()
            if len(fields) < 2:
                raise ValueError(
                    f"{path}:{line_number}: expected TIMESTAMP PATH"
                )
            try:
                timestamp = float(fields[0])
            except ValueError as error:
                raise ValueError(
                    f"{path}:{line_number}: invalid timestamp {fields[0]!r}"
                ) from error
            if not math.isfinite(timestamp):
                raise ValueError(
                    f"{path}:{line_number}: timestamp must be finite"
                )
            entries.append(
                TimestampedPath(timestamp, fields[0], fields[1], len(entries))
            )

    if not entries:
        raise ValueError(f"{path}: no timestamped paths found")
    entries.sort(key=lambda entry: (entry.timestamp, entry.source_index))
    return entries


def associate(
    rgb_entries: Sequence[TimestampedPath],
    depth_entries: Sequence[TimestampedPath],
    max_difference: float,
) -> list[tuple[TimestampedPath, TimestampedPath]]:
    """Returns deterministic, one-to-one nearest-timestamp associations."""
    if not math.isfinite(max_difference) or max_difference <= 0.0:
        raise ValueError("max_difference must be a finite positive number")

    depth_timestamps = [entry.timestamp for entry in depth_entries]
    candidates: list[tuple[float, int, int]] = []
    for rgb_index, rgb_entry in enumerate(rgb_entries):
        begin = bisect.bisect_right(
            depth_timestamps, rgb_entry.timestamp - max_difference
        )
        end = bisect.bisect_left(
            depth_timestamps, rgb_entry.timestamp + max_difference
        )
        for depth_index in range(begin, end):
            difference = abs(
                rgb_entry.timestamp - depth_entries[depth_index].timestamp
            )
            candidates.append((difference, rgb_index, depth_index))

    candidates.sort()
    used_rgb: set[int] = set()
    used_depth: set[int] = set()
    matches: list[tuple[TimestampedPath, TimestampedPath]] = []
    for _, rgb_index, depth_index in candidates:
        if rgb_index in used_rgb or depth_index in used_depth:
            continue
        used_rgb.add(rgb_index)
        used_depth.add(depth_index)
        matches.append((rgb_entries[rgb_index], depth_entries[depth_index]))

    matches.sort(key=lambda match: (match[0].timestamp, match[0].source_index))
    return matches


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Associate TUM RGB-D rgb.txt and depth.txt entries using "
            "one-to-one nearest timestamp matching."
        )
    )
    parser.add_argument("rgb_file", type=pathlib.Path, help="TUM rgb.txt")
    parser.add_argument("depth_file", type=pathlib.Path, help="TUM depth.txt")
    parser.add_argument(
        "-o",
        "--output",
        type=pathlib.Path,
        help="output association file (default: stdout)",
    )
    parser.add_argument(
        "--max-difference",
        type=float,
        default=0.02,
        help="maximum timestamp difference in seconds (default: 0.02)",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        rgb_entries = read_file_list(args.rgb_file)
        depth_entries = read_file_list(args.depth_file)
        matches = associate(rgb_entries, depth_entries, args.max_difference)
        if not matches:
            raise ValueError("no timestamp pairs satisfy --max-difference")
        lines = [
            f"{rgb.timestamp_text} {rgb.path} "
            f"{depth.timestamp_text} {depth.path}\n"
            for rgb, depth in matches
        ]
        if args.output is None:
            sys.stdout.writelines(lines)
        else:
            args.output.write_text("".join(lines), encoding="utf-8")
    except (OSError, ValueError) as error:
        print(f"associate_tum_rgbd.py: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
