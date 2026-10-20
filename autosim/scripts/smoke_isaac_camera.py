#!/usr/bin/env python3
"""Smoke: mount Isaac RGB-D and print frame stats (run via isaaclab.sh -p)."""

from __future__ import annotations

import logging
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
os.chdir(ROOT)
sys.path.insert(0, str(ROOT))

logging.basicConfig(level=logging.INFO)

from autosim.backends.isaac.world import IsaacWorld  # noqa: E402
from autosim.runtime import Config  # noqa: E402


def main() -> int:
    cfg = Config.load(ROOT / "config" / "isaac.yaml")
    print("creating...", flush=True)
    sim = IsaacWorld.create(cfg.data)
    print("mounted", sim._sensors._camera is not None, flush=True)
    sim.reset(0.0, 0.0, 0.0)
    for _ in range(30):
        sim.step()
    color, depth = sim.color_depth()
    finite = int(np.isfinite(depth).sum())
    print(
        "RGB",
        color.shape,
        "mean",
        float(color.mean()),
        "max",
        int(color.max()),
        "min",
        int(color.min()),
        flush=True,
    )
    print(
        "DEPTH",
        depth.shape,
        "finite",
        finite,
        "nan",
        int(np.isnan(depth).sum()),
        flush=True,
    )
    ok = int(color.max()) > 0 and finite > 100
    print("CAMERA_OK" if ok else "CAMERA_WEAK", flush=True)
    sim.close()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
