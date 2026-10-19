# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Lightweight trajectory visualization (optional matplotlib)."""

from __future__ import annotations

import json
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from benchmark.tasks.episode_runner import EpisodeTrace


def traces_to_svg(traces: Sequence[EpisodeTrace], path: str | Path, size: int = 400) -> Path:
    """Write a simple SVG overlay of paths (no matplotlib required)."""
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    # World assumed ±10m
    scale = size / 20.0

    def xy(x: float, y: float) -> tuple[float, float]:
        return (10 + x) * scale, (10 - y) * scale

    parts: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{size}" height="{size}">',
        f'<rect width="{size}" height="{size}" fill="#0b1220"/>',
    ]
    colors = ["#4cc9f0", "#f72585", "#b5e48c", "#ffd166", "#90e0ef"]
    for idx, trace in enumerate(traces):
        if len(trace.points) < 2:
            continue
        color = colors[idx % len(colors)]
        d = []
        for i, p in enumerate(trace.points):
            px, py = xy(p.pose.x, p.pose.y)
            d.append(("M" if i == 0 else "L") + f"{px:.1f},{py:.1f}")
        parts.append(f'<path d="{" ".join(d)}" fill="none" stroke="{color}" stroke-width="2"/>')
        gx, gy = xy(trace.setup.goal.x, trace.setup.goal.y)
        parts.append(f'<circle cx="{gx:.1f}" cy="{gy:.1f}" r="4" fill="{color}"/>')
    parts.append("</svg>")
    out.write_text("\n".join(parts), encoding="utf-8")
    return out


def load_traces_json(path: str | Path) -> list[dict[str, Any]]:
    return json.loads(Path(path).read_text(encoding="utf-8"))
