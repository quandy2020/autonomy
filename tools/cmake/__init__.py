"""CMake-related developer checks and packaging helpers."""

from __future__ import annotations

__all__ = ["REPOSITORY_ROOT"]

from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
