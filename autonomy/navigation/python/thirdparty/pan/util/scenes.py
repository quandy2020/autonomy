"""Locate Habitat scene meshes for HM3D (``*.basis.glb``) and MP3D/Gibson (``*.glb``)."""

from __future__ import annotations

import os


def scene_glb_path(scene_root: str, scene_id: str) -> str:
    """Scene mesh under ``<scene_root>/<id>/``.

    HM3D uses ``<short_id>.basis.glb`` (id after the first dash). Matterport3D and
    Gibson use ``<id>.glb``. Semantic meshes are skipped.
    """
    folder = os.path.join(scene_root, scene_id)
    short = scene_id.split("-", 1)[-1]
    candidates = (
        os.path.join(folder, f"{short}.basis.glb"),
        os.path.join(folder, f"{scene_id}.basis.glb"),
        os.path.join(folder, f"{scene_id}.glb"),
        os.path.join(folder, f"{short}.glb"),
    )
    for path in candidates:
        if os.path.exists(path):
            return path
    if os.path.isdir(folder):
        for name in sorted(os.listdir(folder)):
            lower = name.lower()
            if lower.endswith(".glb") and "semantic" not in lower:
                return os.path.join(folder, name)
    return candidates[0]
