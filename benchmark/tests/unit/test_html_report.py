# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from pathlib import Path

from benchmark.evaluation.report import write_report


def test_html_report(tmp_path: Path) -> None:
    summary = {
        "efficiency": {"success_rate": 0.8},
        "safety": {"collision_amount": 1},
        "smoothness": {"mean_jerk": 0.1},
        "status": {"goal_reached": 4},
        "count": 5,
    }
    path = write_report(summary, tmp_path, name="demo")
    assert path.exists()
    html = (tmp_path / "demo.html").read_text(encoding="utf-8")
    assert "Arena Evaluation" in html
    assert "success_rate" in html
