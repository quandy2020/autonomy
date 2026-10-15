# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Arena Unity ↔ Habitat photorealistic sidecar (RGB-D capable)."""

from __future__ import annotations

from typing import Any, Dict, Optional

from benchmark.simulators.sidecar import SidecarSimulator


class UnitySimulator(SidecarSimulator):
    name = "unity"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        cfg = dict(config or {})
        cfg.setdefault("bridge_node", "benchmark_unity")
        cfg.setdefault("backend", "unity")
        cfg.setdefault("subscribe_scan", True)
        cfg.setdefault("launch_autosim", False)
        cfg.setdefault("autolink", True)
        super().__init__(cfg)
