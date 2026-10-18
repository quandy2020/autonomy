#!/usr/bin/env python3
"""Convert Kalibr/ROS camera_info YAML → Argus EquidistantCamera YAML.

Usage:
  python3 scripts/convert_kalibr_camera.py --kalibr cam0.yaml --out camera_equidistant.yaml
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path


def _parse_simple_yaml(text: str) -> dict:
    """Minimal parser for Kalibr cam*.yaml (no nested OpenCV !!opencv-matrix)."""
    out: dict = {}
    # flatten "key: value" and "data: [a, b, ...]" / multi-line lists
    text = re.sub(r"#.*", "", text)
    # join continued list lines
    text = re.sub(r",\s*\n\s*", ", ", text)
    for m in re.finditer(
        r"^([A-Za-z0-9_]+):\s*(.+)$", text, flags=re.MULTILINE
    ):
        key, val = m.group(1), m.group(2).strip()
        if val.startswith("[") and val.endswith("]"):
            nums = [float(x.strip()) for x in val[1:-1].split(",") if x.strip()]
            out[key] = nums
        else:
            try:
                out[key] = int(val)
            except ValueError:
                try:
                    out[key] = float(val)
                except ValueError:
                    out[key] = val.strip("\"'")
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--kalibr", required=True, help="calibration/cam0.yaml")
    ap.add_argument("--out", required=True, help="Argus equidistant yaml")
    args = ap.parse_args()

    raw = Path(args.kalibr).read_text()
    d = _parse_simple_yaml(raw)
    K = d.get("data")  # ambiguous if multiple "data" keys — parse carefully
    # Prefer camera_matrix / distortion_coefficients blocks via regex
    cm = re.search(
        r"camera_matrix:.*?data:\s*\[([^\]]+)\]", raw, flags=re.DOTALL
    )
    dc = re.search(
        r"distortion_coefficients:.*?data:\s*\[([^\]]+)\]", raw, flags=re.DOTALL
    )
    if not cm or not dc:
        raise SystemExit("failed to parse camera_matrix / distortion_coefficients")
    kvals = [float(x) for x in re.split(r"[,\s]+", cm.group(1).strip()) if x]
    dvals = [float(x) for x in re.split(r"[,\s]+", dc.group(1).strip()) if x]
    if len(kvals) < 9 or len(dvals) < 4:
        raise SystemExit(f"bad lengths K={len(kvals)} D={len(dvals)}")

    w = int(d.get("image_width", 1440))
    h = int(d.get("image_height", 1080))
    fx, fy, cx, cy = kvals[0], kvals[4], kvals[2], kvals[5]
    k1, k2, k3, k4 = dvals[:4]

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        "%YAML:1.0\n"
        "---\n"
        'cam_model: "EquidistantCamera"\n'
        f"cam_width: {w}\n"
        f"cam_height: {h}\n"
        "scale: 1.0\n"
        f"cam_fx: {fx}\n"
        f"cam_fy: {fy}\n"
        f"cam_cx: {cx}\n"
        f"cam_cy: {cy}\n"
        f"k1: {k1}\n"
        f"k2: {k2}\n"
        f"k3: {k3}\n"
        f"k4: {k4}\n"
    )
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
