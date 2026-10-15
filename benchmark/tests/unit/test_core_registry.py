# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from benchmark.core.registry import get_registry


def test_builtin_registration() -> None:
    hub = get_registry()
    hub.ensure_builtins()
    assert "fake" in hub.simulators
    assert "navigate" in hub.tasks
    assert "pure_pursuit" in hub.planners
