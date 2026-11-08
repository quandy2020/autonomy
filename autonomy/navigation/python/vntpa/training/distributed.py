"""Minimal distributed-environment helpers."""

from __future__ import annotations

import os


def world_size() -> int:
    return int(os.environ.get("WORLD_SIZE", "1"))


def local_rank() -> int:
    return int(os.environ.get("LOCAL_RANK", "0"))


def is_distributed() -> bool:
    return world_size() > 1
