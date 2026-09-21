#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors
# SPDX-License-Identifier: Apache-2.0
"""Thermal metrics: temperature rise / CPU frequency throttle (Linux sysfs)."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any


def _read_first(glob_pat: str) -> float | None:
    paths = sorted(Path("/sys").glob(glob_pat))
    for p in paths:
        try:
            raw = p.read_text(encoding="utf-8").strip()
            return float(raw)
        except (OSError, ValueError):
            continue
    return None


def sample_thermal() -> dict[str, Any]:
    # thermal_zone temp is millidegree C
    t_mC = _read_first("class/thermal/thermal_zone*/temp")
    # cpufreq in kHz
    f_khz = _read_first("devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
    f_max = _read_first("devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq")
    temp_c = (t_mC / 1000.0) if t_mC is not None else float("nan")
    freq_mhz = (f_khz / 1000.0) if f_khz is not None else float("nan")
    max_mhz = (f_max / 1000.0) if f_max is not None else float("nan")
    throttle = float("nan")
    if f_khz is not None and f_max is not None and f_max > 0:
        throttle = 1.0 - (f_khz / f_max)
    return {
        "temp_c": temp_c,
        "cpu_freq_mhz": freq_mhz,
        "cpu_freq_max_mhz": max_mhz,
        "throttle_ratio": throttle,
        "ts": time.time(),
    }


def monitor(duration_s: float, interval_s: float) -> dict[str, Any]:
    samples: list[dict[str, Any]] = []
    t_end = time.time() + duration_s
    while time.time() < t_end:
        samples.append(sample_thermal())
        time.sleep(max(0.05, interval_s))
    if not samples:
        return {"samples": 0}
    temps = [s["temp_c"] for s in samples if s["temp_c"] == s["temp_c"]]
    thr = [s["throttle_ratio"] for s in samples if s["throttle_ratio"] == s["throttle_ratio"]]
    return {
        "samples": len(samples),
        "temp_c_start": samples[0]["temp_c"],
        "temp_c_end": samples[-1]["temp_c"],
        "temp_c_rise": (temps[-1] - temps[0]) if len(temps) >= 2 else float("nan"),
        "temp_c_max": max(temps) if temps else float("nan"),
        "throttle_ratio_max": max(thr) if thr else float("nan"),
        "trace": samples,
    }


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration", type=float, default=5.0, help="seconds")
    p.add_argument("--interval", type=float, default=0.5, help="sample period")
    p.add_argument("--once", action="store_true", help="single sample")
    p.add_argument("--out", type=Path, default=None)
    args = p.parse_args()
    result = sample_thermal() if args.once else monitor(args.duration, args.interval)
    text = json.dumps(result, indent=2)
    print(text)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
