#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors
# SPDX-License-Identifier: Apache-2.0
"""Efficiency metrics: FPS / CPU / GPU / Mem from runner logs or live sample."""

from __future__ import annotations

import argparse
import json
import os
import re
import resource
import sys
import time
from pathlib import Path
from typing import Any


_STEP_RE = re.compile(
    r"per_step_ms\s*=\s*([0-9.]+)|fps\s*=\s*([0-9.]+)|steps\s*=\s*(\d+).*total_ms\s*=\s*([0-9.]+)",
    re.IGNORECASE,
)


def parse_log(path: Path) -> dict[str, float]:
    text = path.read_text(encoding="utf-8", errors="ignore")
    fps = float("nan")
    per_step_ms = float("nan")
    for m in _STEP_RE.finditer(text):
        if m.group(1):
            per_step_ms = float(m.group(1))
            if per_step_ms > 0:
                fps = 1000.0 / per_step_ms
        elif m.group(2):
            fps = float(m.group(2))
        elif m.group(3) and m.group(4):
            steps = float(m.group(3))
            total_ms = float(m.group(4))
            if steps > 0 and total_ms > 0:
                per_step_ms = total_ms / steps
                fps = 1000.0 * steps / total_ms
    return {"fps": fps, "per_step_ms": per_step_ms}


def sample_process() -> dict[str, float]:
    """Lightweight self sample (RSS). GPU left as NaN without NVML."""
    usage = resource.getrusage(resource.RUSAGE_SELF)
    # ru_maxrss is KB on Linux
    rss_mb = float(usage.ru_maxrss) / 1024.0
    return {
        "cpu_user_s": float(usage.ru_utime),
        "cpu_sys_s": float(usage.ru_stime),
        "rss_mb": rss_mb,
        "gpu_util_pct": float("nan"),
        "gpu_mem_mb": float("nan"),
    }


def evaluate(log: Path | None = None, sample: bool = False) -> dict[str, Any]:
    out: dict[str, Any] = {}
    if log and log.is_file():
        out.update(parse_log(log))
    if sample:
        out.update(sample_process())
    return out


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--log", type=Path, default=None, help="runner stdout log")
    p.add_argument("--sample", action="store_true", help="sample current process")
    p.add_argument("--out", type=Path, default=None)
    args = p.parse_args()
    if not args.log and not args.sample:
        print("need --log and/or --sample", file=sys.stderr)
        return 1
    result = evaluate(args.log, sample=args.sample)
    result["hostname"] = os.uname().nodename
    result["ts"] = time.time()
    text = json.dumps(result, indent=2)
    print(text)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
