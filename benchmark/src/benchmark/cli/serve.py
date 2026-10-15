# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""HTTP API server CLI."""

from __future__ import annotations

import click


@click.command("serve")
@click.option("--host", default="0.0.0.0", show_default=True)
@click.option("--port", type=int, default=8088, show_default=True)
def serve_cmd(host: str, port: int) -> None:
    """Start FastAPI server (requires ``pip install -e '.[api]'``)."""
    try:
        import uvicorn
    except ImportError as exc:
        raise click.ClickException("Install with: pip install -e '.[api]'") from exc
    from benchmark.api.server import create_app

    app = create_app()
    click.echo(f"serving on http://{host}:{port}")
    uvicorn.run(app, host=host, port=port)
