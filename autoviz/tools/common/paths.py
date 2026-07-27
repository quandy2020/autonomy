"""Path helpers for Autoviz developer tools."""

from __future__ import annotations

from pathlib import Path


def find_autoviz_root(start: Path | None = None) -> Path:
    default_root = Path(__file__).resolve().parents[2]
    if (default_root / "CMakeLists.txt").is_file() and (default_root / "autoviz").is_dir():
        if start is None:
            return default_root

    current = (start or Path.cwd()).resolve()
    if current.is_file():
        current = current.parent
    for candidate in [current, *current.parents]:
        if (candidate / "CMakeLists.txt").is_file() and (candidate / "autoviz").is_dir():
            return candidate
    raise RuntimeError(f"Could not locate autoviz package root from {start}")


def find_autonomy_root(start: Path | None = None) -> Path:
    autoviz_root = find_autoviz_root(start)
    autonomy_root = autoviz_root.parent
    if not (autonomy_root / "CMakeLists.txt").is_file():
        raise RuntimeError(f"Autonomy CMake root not found above {autoviz_root}")
    return autonomy_root


def autoviz_build_dir(autoviz_root: Path | None = None) -> Path:
    return (autoviz_root or find_autoviz_root()) / "build"


def translations_dir(autoviz_root: Path | None = None) -> Path:
    return (autoviz_root or find_autoviz_root()) / "translations"


def default_qgc_translations_dir() -> Path:
    autoviz_root = find_autoviz_root()
    candidates = [
        autoviz_root.parents[3] / "ros" / "qgroundcontrol" / "translations",
        autoviz_root.parents[4] / "ros" / "qgroundcontrol" / "translations",
        Path.home() / "workspace" / "github" / "ros" / "qgroundcontrol" / "translations",
    ]
    for path in candidates:
        if path.is_dir():
            return path
    return candidates[0]
