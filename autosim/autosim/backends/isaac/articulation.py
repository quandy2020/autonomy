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

"""Generic Isaac articulation helpers (root lookup, dynamic links, URDF rename)."""

from __future__ import annotations

import logging
from typing import Any

logger = logging.getLogger(__name__)


def kit_log(message: str) -> None:
    """Write to Isaac Kit log (Python INFO is often filtered there)."""
    try:
        import carb

        carb.log_warn(f"[autosim] {message}")
    except Exception:
        pass
    logger.warning("%s", message)


def find_articulation_root(stage: Any, under: str) -> str | None:
    """Return first prim path with ``ArticulationRootAPI`` under ``under``.

    Never returns a bare mount Xform without the API, and never appends
    ``base_footprint`` onto a path that already ends with it.
    """
    from pxr import Usd, UsdPhysics

    root = stage.GetPrimAtPath(under)
    if not root or not root.IsValid():
        return None
    if root.HasAPI(UsdPhysics.ArticulationRootAPI):
        return str(root.GetPath())
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    under_path = str(under).rstrip("/")
    # Fallback: child base_footprint only when ``under`` is the mount Xform.
    if under_path.endswith("/base_footprint"):
        return None
    for name in ("base_footprint", "base_link"):
        candidate = f"{under_path}/{name}"
        prim = stage.GetPrimAtPath(candidate)
        if prim and prim.IsValid() and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return candidate
    return None



def prim_exists(stage: Any, path: str) -> bool:
    prim = stage.GetPrimAtPath(path)
    return bool(prim and prim.IsValid())



def robot_tree_root(articulation_root: str) -> str:
    """Return robot mount Xform from an articulation link path.

    ``/beni/base_footprint`` → ``/beni``;
    ``/Root/Robot/base_footprint`` → ``/Root/Robot``.
    """
    path = str(articulation_root).rstrip("/")
    if path.endswith("/base_footprint"):
        parent = path[: -len("/base_footprint")]
        return parent if parent else path
    parts = path.strip("/").split("/")
    if not parts or parts == [""]:
        return path or articulation_root
    if parts[0] in ("Root", "World") and len(parts) >= 2:
        return "/" + "/".join(parts[:2])
    return "/" + parts[0]



def ensure_articulation_dynamic(stage: Any, articulation_root: str) -> int:
    """Force every rigid link under the articulation to be dynamic (not kinematic).

    InteriorAgent scene prep freezes furniture as kinematic; if the robot skip
    prefix is wrong, Beni links become static and PhysX refuses joints
    (``CreateJoint - cannot create a joint between static bodies``) → 0 DOFs.
    Kinematic root also means no gravity fall (不会倒) and free teleports (穿模).
    """
    from pxr import Usd, UsdPhysics

    root = stage.GetPrimAtPath(articulation_root)
    if not root or not root.IsValid():
        return 0
    fixed = 0
    walk_root = root
    parent = root.GetParent()
    if parent and parent.IsValid() and str(parent.GetPath()) not in ("/", ""):
        walk_root = parent
    # Ensure ArticulationRootAPI on the reported root (floating base).
    try:
        if not root.HasAPI(UsdPhysics.ArticulationRootAPI):
            UsdPhysics.ArticulationRootAPI.Apply(root)
    except Exception:
        pass
    for prim in Usd.PrimRange(walk_root):
        path = str(prim.GetPath())
        # Skip joint prims / visuals — only true links need RigidBody.
        name = prim.GetName()
        if name in ("joints", "visuals", "collisions") or "/joints/" in path:
            continue
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            # Links with mass or collision must be dynamic for the articulation.
            has_mass = prim.HasAPI(UsdPhysics.MassAPI)
            has_col = prim.HasAPI(UsdPhysics.CollisionAPI)
            if not (has_mass or has_col):
                continue
            try:
                UsdPhysics.RigidBodyAPI.Apply(prim)
            except Exception:
                continue
        api = UsdPhysics.RigidBodyAPI(prim)
        try:
            enabled = api.GetRigidBodyEnabledAttr()
            if not enabled:
                api.CreateRigidBodyEnabledAttr(True)
            else:
                enabled.Set(True)
            kin = api.GetKinematicEnabledAttr()
            if not kin:
                api.CreateKinematicEnabledAttr(False)
            else:
                kin.Set(False)
            fixed += 1
        except Exception:
            continue
        # PhysX schema can independently disable gravity (不会倒).
        try:
            from pxr import PhysxSchema

            if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                px = PhysxSchema.PhysxRigidBodyAPI(prim)
                dg = px.GetDisableGravityAttr()
                if dg:
                    dg.Set(False)
                else:
                    px.CreateDisableGravityAttr(False)
        except Exception:
            pass
    if fixed:
        kit_log(
            f"forced {fixed} articulation rigid bodies dynamic under {walk_root.GetPath()}"
        )
    return fixed



def urdf_renamed_for_import(urdf_path: Path, robot_name: str) -> Path:
    """Write a sibling URDF with a unique ``<robot name>`` (meshes stay relative)."""
    import re

    text = urdf_path.read_text(encoding="utf-8")
    rewritten, n = re.subn(
        r'(<robot\s+[^>]*name=")([^"]*)(")',
        rf"\g<1>{robot_name}\g<3>",
        text,
        count=1,
    )
    if n != 1:
        rewritten = re.sub(
            r"<robot\b",
            f'<robot name="{robot_name}"',
            text,
            count=1,
        )
    out = urdf_path.parent / f".{robot_name}.urdf"
    out.write_text(rewritten, encoding="utf-8")
    return out


