# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from benchmark.dashboard.bicmap_scenes import BICMAP_SCENES, bicmap_payload


def test_bicmap_scenes_cover_missions() -> None:
    missions = {s["mission"] for s in BICMAP_SCENES}
    assert {"navigate", "dock", "explore_mapping", "patrol", "teleop"} <= missions


def test_bicmap_payload() -> None:
    p = bicmap_payload()
    assert p["offline"] is True
    assert "vendor/bic-map" in p["local_js"]
    assert p["scenes"]


def test_vendored_bicmap_js_exists() -> None:
    from pathlib import Path

    from benchmark.common.config import package_root

    js = package_root() / "src/benchmark/dashboard/static/vendor/bic-map/bicMap.ext.min.js"
    # editable install: package_root is benchmark/; also try relative to this file
    if not js.is_file():
        js = Path(__file__).resolve().parents[2] / "src/benchmark/dashboard/static/vendor/bic-map/bicMap.ext.min.js"
    assert js.is_file(), f"missing vendored BicMap: {js}"
    assert js.stat().st_size > 100_000
