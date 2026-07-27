"""Minimal terminal logging for Autoviz tools."""

from __future__ import annotations

import sys


def _supports_color() -> bool:
    return hasattr(sys.stdout, "isatty") and sys.stdout.isatty()


def _paint(text: str, code: str) -> str:
    if not _supports_color():
        return text
    return f"\033[{code}m{text}\033[0m"


def log_info(message: str) -> None:
    print(message)


def log_ok(message: str) -> None:
    print(_paint(f"✓ {message}", "32"))


def log_warn(message: str) -> None:
    print(_paint(f"! {message}", "33"), file=sys.stderr)


def log_error(message: str) -> None:
    print(_paint(f"✗ {message}", "31"), file=sys.stderr)


def log_step(message: str) -> None:
    print(_paint(f"→ {message}", "36"))
