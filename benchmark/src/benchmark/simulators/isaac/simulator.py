# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Isaac Sim adapter (autosim sidecar + autolink cmd_vel + twin)."""

from __future__ import annotations

from typing import Any, Dict, Optional

from benchmark.simulators.sidecar import SidecarSimulator


class IsaacSimulator(SidecarSimulator):
    name = "isaac"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        cfg = dict(config or {})
        cfg.setdefault("bridge_node", "benchmark_isaac")
        super().__init__(cfg)
