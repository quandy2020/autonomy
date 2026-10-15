# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""FastAPI server for remote task / eval / train / world-gen."""

from __future__ import annotations


def create_app():  # type: ignore[no-untyped-def]
    try:
        from fastapi import FastAPI
    except ImportError as exc:  # pragma: no cover
        raise ImportError("Install with: pip install -e '.[api]'") from exc

    from benchmark.api.routes import register_routes

    app = FastAPI(title="autonomy-benchmark", version="0.1.0")
    register_routes(app)
    return app
