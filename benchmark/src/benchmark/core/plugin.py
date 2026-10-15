# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Entry-point based plugin discovery."""

from __future__ import annotations

import logging
from importlib.metadata import entry_points
from typing import Any, Iterable

from benchmark.core.registry import get_registry

_LOG = logging.getLogger(__name__)
_GROUPS = (
    ("benchmark.simulators", "simulators"),
    ("benchmark.tasks", "tasks"),
    ("benchmark.planners", "planners"),
    ("benchmark.metrics", "metrics"),
)


def _iter_eps(group: str) -> Iterable[Any]:
    try:
        eps = entry_points(group=group)
    except TypeError:  # pragma: no cover - py3.10 compat alternate API
        eps = entry_points().get(group, [])  # type: ignore[assignment]
    return eps


def load_entry_points() -> None:
    """Load setuptools entry points into the global registry (idempotent)."""
    hub = get_registry()
    for group, attr in _GROUPS:
        registry = getattr(hub, attr)
        for ep in _iter_eps(group):
            name = ep.name
            if name in registry:
                continue
            try:
                obj = ep.load()
            except Exception as exc:  # pragma: no cover
                _LOG.warning("failed to load entry point %s:%s (%s)", group, name, exc)
                continue
            registry.register(name, obj if callable(obj) else (lambda o=obj: o))
