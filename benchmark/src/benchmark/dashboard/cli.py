# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""CLI: ``benchmark dashboard`` — Flask real-time monitor."""

from __future__ import annotations

import click


@click.command("dashboard")
@click.option("--host", default="127.0.0.1", show_default=True)
@click.option("--port", type=int, default=8090, show_default=True)
@click.option("--debug", is_flag=True, help="Flask debug reloader")
def dashboard_cmd(host: str, port: int, debug: bool) -> None:
    """Start Flask monitoring dashboard (config / tasks / collect / live jobs)."""
    try:
        from benchmark.dashboard.app import create_app
    except ImportError as exc:
        raise click.ClickException(
            "Dashboard requires the benchmark package + Flask. "
            "Install with: pip install -e '.[dashboard]'"
        ) from exc

    app = create_app()
    click.echo(f"dashboard → http://{host}:{port}  (benchmark.dashboard)")
    app.run(host=host, port=port, debug=debug, threaded=True)
