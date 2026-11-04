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

"""Discover TrackVLA-layout SMPL-X humanoid URDF + motion pickles."""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

logger = logging.getLogger(__name__)

ENV_HUMANOID_ROOT = "AUTOSIM_HUMANOID_ROOT"


@dataclass(frozen=True)
class HumanoidAsset:
    """One avatar: URDF mesh + SMPL-X walk/stop motion pickle."""

    name: str
    urdf: Path
    motion: Path


def resolve_project_path(path: str) -> Path:
    """Resolve a path relative to the autosim project root."""
    text = str(path or "").strip()
    if not text:
        return Path()
    candidate = Path(text).expanduser()
    if candidate.is_absolute():
        return candidate
    try:
        from autosim.runtime.paths import project_root

        return (project_root() / candidate).resolve()
    except Exception:
        return candidate.resolve()


def discover_humanoid_root(explicit: str = "", *, require_explicit: bool = False) -> Path:
    """Resolve ``humanoid_data`` directory (TrackVLA layout).

    Search order: config ``root``, ``AUTOSIM_HUMANOID_ROOT``,
    ``autosim/data/humanoids/humanoid_data``.

    When ``explicit`` is set but missing, return empty (do not silently fall
    back) so misconfigured paths fail closed.
    """
    if explicit.strip():
        resolved = resolve_project_path(explicit)
        if resolved.is_dir():
            return resolved
        logger.warning("humanoids.root not a directory: %s", resolved)
        return Path()
    if require_explicit:
        return Path()
    env = str(os.environ.get(ENV_HUMANOID_ROOT) or "").strip()
    if env:
        path = Path(env).expanduser()
        if (path / "humanoid_data").is_dir():
            return (path / "humanoid_data").resolve()
        if path.is_dir():
            return path.resolve()
        logger.warning("%s is not a directory: %s", ENV_HUMANOID_ROOT, path)
    try:
        from autosim.runtime.paths import project_root

        bundled = project_root() / "data" / "humanoids" / "humanoid_data"
        if bundled.is_dir():
            return bundled.resolve()
    except Exception:
        pass
    return Path()


def list_humanoid_assets(
    root: str | Path,
    names: Sequence[str] | None = None,
) -> list[HumanoidAsset]:
    """Collect ``<name>/<name>.urdf`` + ``*_motion_data_smplx.pkl`` under root."""
    base = Path(root).expanduser() if root else Path()
    if not base.is_dir():
        return []
    wanted = [str(n).strip() for n in (names or []) if str(n).strip()]
    assets: list[HumanoidAsset] = []
    if wanted:
        dirs = []
        for name in wanted:
            child = base / name
            if child.is_dir():
                dirs.append(child)
            else:
                logger.warning("humanoid name not found under %s: %s", base, name)
    else:
        dirs = sorted(p for p in base.iterdir() if p.is_dir())
    for child in dirs:
        name = child.name
        urdf = child / f"{name}.urdf"
        motion = child / f"{name}_motion_data_smplx.pkl"
        if not urdf.is_file():
            # Some packs use a single motion pickle name pattern.
            logger.debug("skip humanoid %s: missing %s", name, urdf.name)
            continue
        if not motion.is_file():
            # Fallback: any *_motion_data_smplx.pkl in the folder.
            candidates = sorted(child.glob("*_motion_data_smplx.pkl"))
            if not candidates:
                logger.debug("skip humanoid %s: missing motion pickle", name)
                continue
            motion = candidates[0]
        assets.append(HumanoidAsset(name=name, urdf=urdf, motion=motion))
    return assets
