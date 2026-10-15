# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""API route registration."""

from __future__ import annotations

from typing import Any


def register_routes(app: Any) -> None:
    from benchmark.api.routes import eval as eval_routes
    from benchmark.api.routes import sim as sim_routes
    from benchmark.api.routes import train as train_routes

    sim_routes.mount(app)
    eval_routes.mount(app)
    train_routes.mount(app)
