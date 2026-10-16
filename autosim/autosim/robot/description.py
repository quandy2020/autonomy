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

"""Prepare URDF payload for ``/robot_description`` (host-visible paths)."""

from __future__ import annotations

import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional, Tuple

from autosim.robot.urdf import UrdfModel

# Autoviz ``loadFromString`` uses cwd as baseDirectory — relative mesh=
# filenames would miss. Rewrite to absolute paths under the URDF directory.
_MESH_ATTR = re.compile(
    r'(filename\s*=\s*")([^"]+)(")',
    re.IGNORECASE,
)

_DEFAULT_CONTAINER_ROOT = "/workspace/autonomy"


def absolutize_mesh_filenames(urdf_text: str, urdf_dir: Path) -> str:
    """Rewrite relative ``filename=`` mesh paths to absolute paths."""
    root = Path(urdf_dir).resolve()

    def replace(match: re.Match[str]) -> str:
        prefix, raw, suffix = match.group(1), match.group(2), match.group(3)
        path = Path(raw.strip())
        if not path.is_absolute():
            path = (root / path).resolve()
        return f"{prefix}{path.as_posix()}{suffix}"

    return _MESH_ATTR.sub(replace, urdf_text)


def _infer_bind_source(mountpoint: str) -> Optional[str]:
    """Return the mountinfo ``root`` (host path) for ``mountpoint``."""
    target = mountpoint.rstrip("/") or "/"
    try:
        with open("/proc/self/mountinfo", encoding="utf-8") as handle:
            for line in handle:
                parts = line.split()
                if len(parts) < 5:
                    continue
                root_in_fs = parts[3]
                mnt = parts[4].rstrip("/") or "/"
                if mnt != target:
                    continue
                if root_in_fs and root_in_fs != "/":
                    return root_in_fs
    except OSError:
        return None
    return None


def workspace_roots() -> Tuple[str, str]:
    """Return ``(container_root, host_root)`` when host remapping is requested.

    Set ``AUTOSIM_HOST_ROOT`` (and optionally ``AUTOSIM_CONTAINER_ROOT``) only when
    Autoviz runs on the **host** while Isaac runs in Docker. When Autoviz shares
    the container (typical SpaceHero setup), leave both unset so the published
    path stays under ``/workspace/autonomy``.
    """
    host = os.environ.get("AUTOSIM_HOST_ROOT", "").strip()
    if not host:
        return "", ""
    container = os.environ.get("AUTOSIM_CONTAINER_ROOT", "").strip()
    if container:
        return container.rstrip("/"), host.rstrip("/")
    inferred = _infer_bind_source(_DEFAULT_CONTAINER_ROOT)
    return _DEFAULT_CONTAINER_ROOT, (inferred or host).rstrip("/")


def host_visible_path(path: Path) -> Path:
    """Optionally map a container path to the host bind for host-side Autoviz.

    Default: return ``path`` unchanged (Autoviz in the same container).
    With ``AUTOSIM_HOST_ROOT`` set: rewrite ``/workspace/autonomy/...`` to the
    host bind so a host Autoviz can open the URDF and meshes.
    """
    resolved = path.resolve()
    text = resolved.as_posix()
    container_root, host_root = workspace_roots()
    if not container_root or not host_root:
        return resolved
    if text == container_root or text.startswith(container_root + "/"):
        return Path(host_root + text[len(container_root) :])
    return resolved


def load_robot_description(urdf: str) -> Optional[str]:
    """Build ``/robot_description`` payload for Autoviz RobotModel.

    Publishes the **absolute URDF path** (not inline XML). Autoviz
    ``loadFromString`` treats a non-``<robot`` string as a file path and sets
    ``baseDirectory`` to the URDF folder so relative ``meshes/...`` resolve.

    When the sim runs in Docker, the path is remapped to the host bind mount
    so a host-side Autoviz can open the file.

    Args:
        urdf: Project-relative or absolute URDF path (empty → ``None``).

    Returns:
        Absolute filesystem path string, or ``None`` if blank / missing.
    """
    path = UrdfModel.resolve(urdf)
    if path is None or not path.is_file():
        return None
    text = path.read_text(encoding="utf-8")
    try:
        root = ET.fromstring(text)
    except ET.ParseError:
        return None
    if root.tag != "robot":
        return None
    return host_visible_path(path).as_posix()
