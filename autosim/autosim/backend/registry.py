# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Create a WorldBackend from validated config settings."""

from __future__ import annotations

from typing import Any, Callable, Dict, Mapping

BACKENDS: Dict[str, Callable[[Mapping[str, Any]], Any]] = {}
_BUILTINS_LOADED = False


def register(name: str, factory: Callable[[Mapping[str, Any]], Any]) -> None:
    """Register ``factory(settings_data) -> WorldBackend`` under ``name``."""
    BACKENDS[str(name).strip().lower()] = factory


def create_backend(settings: Mapping[str, Any]) -> Any:
    """Instantiate the backend named by ``habitat.backend``.

    Args:
        settings: Full config mapping (``{"habitat": {...}}``) or the
            ``habitat`` block alone.

    Returns:
        A WorldBackend implementation.

    Raises:
        ValueError: Unknown backend name.
        ImportError: Optional engine dependency missing.
    """
    _ensure_builtins()
    habitat = settings.get("habitat") if isinstance(settings.get("habitat"), Mapping) else settings
    if not isinstance(habitat, Mapping):
        raise ValueError("settings must contain a habitat mapping")
    name = str(habitat.get("backend", "habitat")).strip().lower()
    factory = BACKENDS.get(name)
    if factory is None:
        known = ", ".join(sorted(BACKENDS) or ("habitat", "fake", "isaac"))
        raise ValueError(f"habitat.backend must be one of: {known} (got {name!r})")
    return factory(settings if "habitat" in settings else {"habitat": habitat})


def _ensure_builtins() -> None:
    """Import backend packages so they self-register."""
    global _BUILTINS_LOADED
    if _BUILTINS_LOADED:
        return
    from autosim.backends import fake as _fake  # noqa: F401
    from autosim.backends import habitat as _habitat  # noqa: F401
    from autosim.backends import isaac as _isaac  # noqa: F401

    _BUILTINS_LOADED = True
