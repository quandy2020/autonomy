# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Logging helpers."""

from __future__ import annotations

import logging
import sys


def setup_logging(level: str = "INFO", *, name: str | None = None) -> logging.Logger:
    root = logging.getLogger(name or "benchmark")
    if not root.handlers:
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(
            logging.Formatter("%(asctime)s %(levelname)s [%(name)s] %(message)s", "%H:%M:%S")
        )
        root.addHandler(handler)
    root.setLevel(getattr(logging, level.upper(), logging.INFO))
    return root
