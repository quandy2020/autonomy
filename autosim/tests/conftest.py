"""Ensure local automsgs *_pb2 stubs are importable without a full install."""
from __future__ import annotations

import sys
from pathlib import Path

_DEPS = Path(__file__).resolve().parents[1] / ".deps" / "python"
if _DEPS.is_dir():
    p = str(_DEPS)
    if p not in sys.path:
        sys.path.insert(0, p)
