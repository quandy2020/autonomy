#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors
# SPDX-License-Identifier: Apache-2.0
"""Robustness metrics: failure rate / tracking loss / loop-closure recall."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any


_FAIL = re.compile(r"Step failed|No odometry|OnLost|tracking.?lost", re.I)
_OK = re.compile(r"Step ok|OnTrackingOk|tracking.?ok", re.I)
_LOOP_DET = re.compile(r"loop.?detected|loop.?candidate", re.I)
_LOOP_ACC = re.compile(r"loop.?accepted|loop.?closed", re.I)


def evaluate_log(path: Path) -> dict[str, Any]:
    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    fail = sum(1 for ln in lines if _FAIL.search(ln))
    ok = sum(1 for ln in lines if _OK.search(ln))
    loop_det = sum(1 for ln in lines if _LOOP_DET.search(ln))
    loop_acc = sum(1 for ln in lines if _LOOP_ACC.search(ln))
    total = fail + ok
    failure_rate = (fail / total) if total else float("nan")
    recall = (loop_acc / loop_det) if loop_det else float("nan")
    return {
        "steps_ok": ok,
        "steps_fail": fail,
        "failure_rate": failure_rate,
        "loop_candidates": loop_det,
        "loop_accepted": loop_acc,
        "loop_recall": recall,
    }


def evaluate_events(events: Path) -> dict[str, Any]:
    """JSONL events: {\"type\": \"lost\"|\"ok\"|\"loop_candidate\"|\"loop_accepted\"}."""
    ok = fail = cand = acc = 0
    for line in events.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            ev = json.loads(line)
        except json.JSONDecodeError:
            continue
        t = str(ev.get("type", "")).lower()
        if t in ("ok", "tracking_ok"):
            ok += 1
        elif t in ("lost", "fail", "tracking_lost"):
            fail += 1
        elif t in ("loop_candidate", "loop_detected"):
            cand += 1
        elif t in ("loop_accepted", "loop_closed"):
            acc += 1
    total = ok + fail
    return {
        "steps_ok": ok,
        "steps_fail": fail,
        "failure_rate": (fail / total) if total else float("nan"),
        "loop_candidates": cand,
        "loop_accepted": acc,
        "loop_recall": (acc / cand) if cand else float("nan"),
    }


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--log", type=Path, default=None)
    p.add_argument("--events", type=Path, default=None, help="JSONL event stream")
    p.add_argument("--out", type=Path, default=None)
    args = p.parse_args()
    if args.events and args.events.is_file():
        result = evaluate_events(args.events)
    elif args.log and args.log.is_file():
        result = evaluate_log(args.log)
    else:
        print("need --log or --events", file=sys.stderr)
        return 1
    text = json.dumps(result, indent=2)
    print(text)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
