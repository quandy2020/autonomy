"""Subprocess helpers."""

from __future__ import annotations

import subprocess
from collections.abc import Sequence


def run_captured(cmd: Sequence[str], *, cwd: str | None = None, check: bool = True) -> str:
    result = subprocess.run(
        list(cmd),
        cwd=cwd,
        capture_output=True,
        text=True,
        check=False,
    )
    output = (result.stdout or "") + (result.stderr or "")
    if check and result.returncode != 0:
        raise subprocess.CalledProcessError(result.returncode, list(cmd), output)
    return output
