# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Arena Flatland ↔ kinematic FakeSimulator (2D Arena-style)."""

from __future__ import annotations

from typing import Any, Dict, Optional

from benchmark.simulators.fake.simulator import FakeSimulator


class FlatlandSimulator(FakeSimulator):
    """Flatland-compatible adapter: same kinematics as ``fake``."""

    name = "flatland"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        cfg = dict(config or {})
        cfg.setdefault("backend", "flatland")
        super().__init__(cfg)
