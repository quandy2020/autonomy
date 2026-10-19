# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Flask real-time monitoring dashboard (requires the ``benchmark`` package)."""

from __future__ import annotations

__all__ = ["create_app"]


def create_app():  # type: ignore[no-untyped-def]
    """Create the Flask app; imports ``benchmark.*`` catalog / monitor backends."""
    from benchmark.dashboard.app import create_app as _create

    return _create()
