# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""YAML / CLI configuration loading with nested overrides."""

from __future__ import annotations

import os
import re
from collections.abc import Mapping, MutableMapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from benchmark.common.exceptions import ConfigError

_ENV_PATTERN = re.compile(r"\$\{([^}:]+)(?::-([^}]*))?\}")


def _expand_env(value: str) -> str:
    def repl(match: re.Match[str]) -> str:
        key, default = match.group(1), match.group(2)
        return os.environ.get(key, default if default is not None else "")

    return _ENV_PATTERN.sub(repl, value)


def _expand_tree(node: Any) -> Any:
    if isinstance(node, str):
        return _expand_env(node)
    if isinstance(node, list):
        return [_expand_tree(x) for x in node]
    if isinstance(node, dict):
        return {k: _expand_tree(v) for k, v in node.items()}
    return node


def deep_merge(
    base: MutableMapping[str, Any], override: Mapping[str, Any]
) -> MutableMapping[str, Any]:
    for key, value in override.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, Mapping):
            deep_merge(base[key], value)  # type: ignore[arg-type]
        else:
            base[key] = value  # type: ignore[index]
    return base


def set_dotted(data: MutableMapping[str, Any], dotted: str, value: Any) -> None:
    parts = dotted.split(".")
    cur: MutableMapping[str, Any] = data
    for part in parts[:-1]:
        nxt = cur.get(part)
        if not isinstance(nxt, dict):
            nxt = {}
            cur[part] = nxt
        cur = nxt  # type: ignore[assignment]
    cur[parts[-1]] = value


def package_root() -> Path:
    """``benchmark/`` package root (contains ``configs/``, ``pyproject.toml``)."""
    return Path(__file__).resolve().parents[3]


def resolve_path(path: str | Path, *, base: Path | None = None) -> Path:
    p = Path(path)
    if p.is_absolute():
        return p
    root = base or package_root()
    return (root / p).resolve()


def load_yaml(path: str | Path) -> dict[str, Any]:
    path = resolve_path(path)
    if not path.is_file():
        raise ConfigError(f"config not found: {path}")
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ConfigError(f"config root must be a mapping: {path}")
    return _expand_tree(data)


@dataclass
class BenchmarkConfig:
    """Resolved experiment configuration."""

    raw: dict[str, Any] = field(default_factory=dict)
    path: Path | None = None

    @property
    def experiment(self) -> dict[str, Any]:
        return dict(self.raw.get("experiment") or {})

    @property
    def task_name(self) -> str:
        task = self.raw.get("task") or {}
        return str(task.get("name") or "navigate")

    @property
    def simulator_name(self) -> str:
        sim = self.raw.get("simulator") or {}
        return str(sim.get("name") or "fake")

    @property
    def planner_name(self) -> str:
        planner = self.raw.get("planner") or {}
        return str(planner.get("name") or "pure_pursuit")

    def section(self, key: str) -> dict[str, Any]:
        value = self.raw.get(key) or {}
        if not isinstance(value, dict):
            raise ConfigError(f"section {key!r} must be a mapping")
        return dict(value)

    def load_ref(self, section: str, key: str = "config") -> dict[str, Any]:
        """Load a nested YAML referenced by ``section.config``."""
        block = self.section(section)
        ref = block.get(key)
        if not ref:
            return block
        base = self.path.parent if self.path else package_root()
        return load_yaml(resolve_path(str(ref), base=base))


def load_config(
    path: str | Path,
    *,
    overrides: Sequence[str] | None = None,
    merges: Sequence[Mapping[str, Any]] | None = None,
) -> BenchmarkConfig:
    """Load experiment YAML and apply ``key=value`` overrides.

    Args:
        path: Path to ``configs/default.yaml`` (or similar).
        overrides: CLI-style dotted pairs, e.g. ``task.name=slam``.
        merges: Extra dicts merged on top (later wins).
    """
    resolved = resolve_path(path)
    raw = load_yaml(resolved)

    # Pull in referenced section configs without clobbering experiment overrides.
    for section in ("simulator", "robot", "world", "planner", "task", "evaluation"):
        block = raw.get(section)
        if isinstance(block, dict) and block.get("config"):
            try:
                nested = load_yaml(resolve_path(str(block["config"]), base=resolved.parent))
                merged = deep_merge(dict(nested), block)
                raw[section] = merged
            except ConfigError:
                pass

    if merges:
        for m in merges:
            deep_merge(raw, m)

    if overrides:
        for item in overrides:
            if "=" not in item:
                raise ConfigError(f"override must be key=value, got {item!r}")
            key, val = item.split("=", 1)
            parsed: Any = yaml.safe_load(val)
            set_dotted(raw, key.strip(), parsed)

    return BenchmarkConfig(raw=raw, path=resolved)
