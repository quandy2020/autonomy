# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Plugin registries for simulators, tasks, planners, metrics."""

from __future__ import annotations

from typing import Any, Callable, Dict, Generic, Iterable, Optional, TypeVar

from benchmark.core.exceptions import RegistryError

T = TypeVar("T")


class Registry(Generic[T]):
    """Simple name → factory registry."""

    def __init__(self, kind: str) -> None:
        self.kind = kind
        self._items: Dict[str, Callable[..., T]] = {}

    def register(self, name: str, factory: Callable[..., T]) -> Callable[..., T]:
        key = name.strip().lower()
        self._items[key] = factory
        return factory

    def decorator(self, name: str) -> Callable[[Callable[..., T]], Callable[..., T]]:
        def wrap(factory: Callable[..., T]) -> Callable[..., T]:
            return self.register(name, factory)

        return wrap

    def get(self, name: str) -> Callable[..., T]:
        key = name.strip().lower()
        if key not in self._items:
            known = ", ".join(sorted(self._items) or ("<empty>",))
            raise RegistryError(f"unknown {self.kind} {name!r}; known: {known}")
        return self._items[key]

    def create(self, name: str, *args: Any, **kwargs: Any) -> T:
        return self.get(name)(*args, **kwargs)

    def names(self) -> Iterable[str]:
        return sorted(self._items)

    def __contains__(self, name: str) -> bool:
        return name.strip().lower() in self._items


class RegistryHub:
    def __init__(self) -> None:
        self.simulators: Registry[Any] = Registry("simulator")
        self.tasks: Registry[Any] = Registry("task")
        self.planners: Registry[Any] = Registry("planner")
        self.metrics: Registry[Any] = Registry("metric")
        self._loaded = False

    def ensure_builtins(self) -> None:
        if self._loaded:
            return
        # Local imports avoid circular deps at module import time.
        from benchmark import planning as _planning  # noqa: F401
        from benchmark import simulators as _simulators  # noqa: F401
        from benchmark import tasks as _tasks  # noqa: F401
        from benchmark.core import plugin as _plugin

        _plugin.load_entry_points()
        self._loaded = True


_HUB: Optional[RegistryHub] = None


def get_registry() -> RegistryHub:
    global _HUB
    if _HUB is None:
        _HUB = RegistryHub()
    return _HUB
