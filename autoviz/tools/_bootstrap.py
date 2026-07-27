"""Shared import bootstrap for Python entrypoints under autoviz/tools/."""

from __future__ import annotations

import contextlib
import faulthandler
import os
import sys
from pathlib import Path

_DEBUG_ENV = "AUTOVIZ_CI_DEBUG"
_GHA_ENV = "GITHUB_ACTIONS"
_HOOK_MARKER = "_autoviz_ci_debug_hook"


def _debug_enabled() -> bool:
    return os.environ.get(_DEBUG_ENV, "").lower() in {"1", "true", "yes", "on"}


def _configure_debug() -> None:
    if not _debug_enabled():
        return

    if not faulthandler.is_enabled():
        with contextlib.suppress(RuntimeError, ValueError, OSError):
            faulthandler.enable(file=sys.stderr)

    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is not None:
            with contextlib.suppress(ValueError, OSError):
                reconfigure(line_buffering=True)

    if os.environ.get(_GHA_ENV, "").lower() != "true":
        return

    if not getattr(sys.excepthook, _HOOK_MARKER, False):
        prev = sys.excepthook

        def _excepthook(exc_type, exc, tb) -> None:
            with contextlib.suppress(Exception):
                print(f"::error::{exc_type.__name__}: {exc}", file=sys.stderr, flush=True)
            prev(exc_type, exc, tb)

        setattr(_excepthook, _HOOK_MARKER, True)
        sys.excepthook = _excepthook


_configure_debug()


def ensure_tools_dir(start: str | Path) -> Path:
    """Ensure the top-level ``tools`` directory is importable."""
    path = Path(start).resolve()
    current = path if path.is_dir() else path.parent
    for candidate in [current, *current.parents]:
        if candidate.name == "tools":
            tools_dir = candidate
            break
        sibling = candidate / "tools"
        if sibling.is_dir():
            tools_dir = sibling
            break
    else:
        raise RuntimeError(f"Could not locate tools directory from {start}")

    if str(tools_dir) not in sys.path:
        sys.path.insert(0, str(tools_dir))
    return tools_dir
