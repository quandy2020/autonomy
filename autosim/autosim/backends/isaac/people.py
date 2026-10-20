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

"""Sync CPU Traffic agents into the Isaac USD stage (RGB / PhysX visuals).

Motion and ``/scan`` stay on CPU ``Traffic``. This module only places:

* ``capsule`` — kinematic capsule (obstacles / legacy pedestrians)
* ``character`` — Isaac People human USDs (+ optional collision proxy)
"""

from __future__ import annotations

import logging
import math
import os
import time
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from autosim.backends.isaac.animation import (
    PeopleAnimator,
    discover_animation_root,
    list_animation_usds,
)

logger = logging.getLogger(__name__)

# Isaac People bipeds face -Y; autosim map yaw=0 faces +X.
CHARACTER_YAW_OFFSET = -0.5 * math.pi
SKIP_CHARACTER_NAME_PARTS = (
    "biped",
    "setup",
    "retarget",
    "animation",
    "test",
    "tests",
    "golden",
    "conveyor",
    "empty_stage",
)


class PeopleSync:
    """Mirror Traffic poses into USD (capsules and/or human characters)."""

    def __init__(
        self,
        stage: Any,
        *,
        root_path: str = "/Root/AutosimTraffic",
        enabled: bool = True,
        obstacle_visual: str = "none",
        pedestrian_visual: str = "character",
        character_assets: Sequence[str] | None = None,
        character_z: float = 0.0,
        character_yaw_offset: float = CHARACTER_YAW_OFFSET,
        collision_proxy: bool = True,
        sync_hz: float = 30.0,
        animator: PeopleAnimator | None = None,
    ) -> None:
        self.stage = stage
        self.root_path = str(root_path).rstrip("/") or "/Root/AutosimTraffic"
        self.enabled = bool(enabled)
        self.obstacle_visual = str(obstacle_visual or "none").strip().lower()
        self.pedestrian_visual = str(pedestrian_visual or "none").strip().lower()
        self.character_assets = [str(p) for p in (character_assets or []) if str(p).strip()]
        self.character_z = float(character_z)
        self.character_yaw_offset = float(character_yaw_offset)
        self.collision_proxy = bool(collision_proxy)
        self.sync_period = (
            1.0 / float(sync_hz) if float(sync_hz) > 0.0 else 0.0
        )
        self.last_sync_mono = 0.0
        self.ready = False
        self.character_warned = False
        self.animator = animator or PeopleAnimator(enabled=False)

    def sync(self, traffic: Any) -> None:
        """Mirror Traffic poses into the open USD stage (rate-limited)."""
        if not self.enabled or traffic is None or self.stage is None:
            return
        now = time.monotonic()
        dt = 0.0 if self.last_sync_mono <= 0.0 else max(0.0, now - self.last_sync_mono)
        if self.sync_period > 0.0 and dt < self.sync_period and self.last_sync_mono > 0.0:
            return
        self.last_sync_mono = now
        try:
            from pxr import Gf, UsdGeom, UsdPhysics
        except ImportError:
            return
        stage = self.stage
        if not self.ready:
            if not stage.GetPrimAtPath(self.root_path).IsValid():
                UsdGeom.Xform.Define(stage, self.root_path)
            self.ready = True

        obstacle = getattr(traffic, "obstacle", None)
        if (
            obstacle is not None
            and self.obstacle_visual in ("capsule", "invisible", "hidden")
        ):
            self.sync_capsule_group(
                stage,
                f"{self.root_path}/Obstacles",
                np.asarray(getattr(obstacle, "position", []), dtype=np.float64).reshape(
                    -1, 2
                ),
                radius=float(obstacle.settings.get("radius", 0.4)),
                height=float(obstacle.settings.get("height", 1.0)),
                prefix="obs",
                UsdGeom=UsdGeom,
                UsdPhysics=UsdPhysics,
                Gf=Gf,
                visible=self.obstacle_visual == "capsule",
            )

        pedestrian = getattr(traffic, "pedestrian", None)
        if pedestrian is None:
            return
        if self.pedestrian_visual == "character":
            poses = (
                pedestrian.poses()
                if hasattr(pedestrian, "poses")
                else np.zeros((0, 3), dtype=np.float64)
            )
            velocity = np.asarray(
                getattr(pedestrian, "velocity", []), dtype=np.float64
            ).reshape(-1, 2)
            radius = float(pedestrian.settings.get("radius", 0.25))
            height = float(pedestrian.settings.get("height", 1.7))
            skins = getattr(pedestrian, "skins", None)
            self.sync_character_group(
                stage,
                f"{self.root_path}/Pedestrians",
                np.asarray(poses, dtype=np.float64).reshape(-1, 3),
                velocity=velocity,
                radius=radius,
                height=height,
                prefix="ped",
                dt=dt if dt > 0.0 else (self.sync_period or 1.0 / 30.0),
                skins=skins,
                UsdGeom=UsdGeom,
                UsdPhysics=UsdPhysics,
                Gf=Gf,
            )
        elif self.pedestrian_visual in ("capsule", "invisible", "hidden"):
            self.sync_capsule_group(
                stage,
                f"{self.root_path}/Pedestrians",
                np.asarray(
                    getattr(pedestrian, "position", []), dtype=np.float64
                ).reshape(-1, 2),
                radius=float(pedestrian.settings.get("radius", 0.2)),
                height=float(pedestrian.settings.get("height", 1.7)),
                prefix="ped",
                UsdGeom=UsdGeom,
                UsdPhysics=UsdPhysics,
                Gf=Gf,
                visible=self.pedestrian_visual == "capsule",
            )

    def character_asset_for_skin(self, skin: int, index: int) -> str:
        """Map HuNav ``skin`` (0=round-robin, 1..N=specific) to a character USD."""
        assets = self.character_assets
        if not assets:
            return ""
        value = int(skin)
        if value <= 0:
            return assets[index % len(assets)]
        # HuNav skins are 1-based into the character list.
        return assets[(value - 1) % len(assets)]

    def sync_character_group(
        self,
        stage: Any,
        group_path: str,
        poses: np.ndarray,
        *,
        velocity: np.ndarray,
        radius: float,
        height: float,
        prefix: str,
        dt: float,
        skins: Any = None,
        UsdGeom: Any,
        UsdPhysics: Any,
        Gf: Any,
    ) -> None:
        """Instance human character USDs and sync ``(x, y, yaw)``."""
        if not self.character_assets:
            if not self.character_warned:
                self.character_warned = True
                logger.warning(
                    "pedestrian_visual=character but no character USDs found; "
                    "set habitat.isaac.traffic.character_root to Isaac People/Characters"
                )
            return
        if not stage.GetPrimAtPath(group_path).IsValid():
            UsdGeom.Xform.Define(stage, group_path)
        count = int(poses.shape[0])
        skin_arr = (
            np.asarray(skins, dtype=np.int32).reshape(-1)
            if skins is not None
            else None
        )
        for index in range(count):
            path = f"{group_path}/{prefix}_{index:03d}"
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                skin = int(skin_arr[index]) if skin_arr is not None and index < len(skin_arr) else 0
                asset = self.character_asset_for_skin(skin, index)
                if not asset or not looks_like_character_usd(Path(asset)):
                    logger.warning("skip non-skin character asset %s", asset)
                    continue
                xform = UsdGeom.Xform.Define(stage, path)
                prim = xform.GetPrim()
                try:
                    prim.GetReferences().AddReference(asset)
                except Exception as exc:
                    logger.warning("failed to reference character %s: %s", asset, exc)
                    continue
                if self.collision_proxy:
                    self.ensure_collision_proxy(
                        stage,
                        f"{path}/CollisionProxy",
                        radius=radius,
                        height=height,
                        UsdGeom=UsdGeom,
                        UsdPhysics=UsdPhysics,
                    )
                self.animator.bind_character(stage, prim)
                logger.info("traffic character [%d] -> %s", index, asset)
            else:
                self.animator.bind_character(stage, prim)
            yaw = float(poses[index, 2]) + self.character_yaw_offset
            self.set_planar_pose(
                prim,
                float(poses[index, 0]),
                float(poses[index, 1]),
                yaw,
                height=self.character_z,
                UsdGeom=UsdGeom,
                Gf=Gf,
                rotate=True,
            )
        speeds = (
            np.linalg.norm(velocity[:count], axis=1)
            if velocity.size
            else np.zeros((count,), dtype=np.float64)
        )
        moving = speeds > 0.05
        mean_speed = float(np.mean(speeds[moving])) if np.any(moving) else 0.0
        self.animator.advance(
            stage,
            dt=float(dt),
            mean_speed=mean_speed,
            any_moving=bool(np.any(moving)),
        )

    @staticmethod
    def ensure_collision_proxy(
        stage: Any,
        path: str,
        *,
        radius: float,
        height: float,
        UsdGeom: Any,
        UsdPhysics: Any,
    ) -> None:
        """Invisible kinematic capsule so PhysX can bump the robot into people."""
        if stage.GetPrimAtPath(path).IsValid():
            return
        from pxr import Gf

        capsule = UsdGeom.Capsule.Define(stage, path)
        capsule.CreateRadiusAttr(float(radius))
        capsule.CreateHeightAttr(float(max(height, 2.0 * radius)))
        capsule.CreateAxisAttr("Z")
        prim = capsule.GetPrim()
        try:
            UsdGeom.Imageable(prim).MakeInvisible()
        except Exception:
            pass
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            body = UsdPhysics.RigidBodyAPI.Apply(prim)
            body.CreateRigidBodyEnabledAttr(True)
            body.CreateKinematicEnabledAttr(True)
        # Center capsule on the character (feet at parent origin).
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.5 * float(height)))

    def sync_capsule_group(
        self,
        stage: Any,
        group_path: str,
        positions: np.ndarray,
        *,
        radius: float,
        height: float,
        prefix: str,
        UsdGeom: Any,
        UsdPhysics: Any,
        Gf: Any,
        visible: bool = True,
    ) -> None:
        """Create / update kinematic capsules for a Traffic agent group."""
        if not stage.GetPrimAtPath(group_path).IsValid():
            UsdGeom.Xform.Define(stage, group_path)
        count = int(positions.shape[0])
        for index in range(count):
            path = f"{group_path}/{prefix}_{index:03d}"
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                capsule = UsdGeom.Capsule.Define(stage, path)
                capsule.CreateRadiusAttr(float(radius))
                capsule.CreateHeightAttr(float(max(height, 2.0 * radius)))
                capsule.CreateAxisAttr("Z")
                if not capsule.GetPrim().HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
                if not capsule.GetPrim().HasAPI(UsdPhysics.RigidBodyAPI):
                    body = UsdPhysics.RigidBodyAPI.Apply(capsule.GetPrim())
                    body.CreateRigidBodyEnabledAttr(True)
                    body.CreateKinematicEnabledAttr(True)
                prim = capsule.GetPrim()
            try:
                imageable = UsdGeom.Imageable(prim)
                if visible:
                    imageable.MakeVisible()
                else:
                    imageable.MakeInvisible()
            except Exception:
                pass
            z = 0.5 * float(max(height, 2.0 * radius))
            self.set_planar_pose(
                prim,
                float(positions[index, 0]),
                float(positions[index, 1]),
                0.0,
                height=z,
                UsdGeom=UsdGeom,
                Gf=Gf,
                rotate=False,
            )

    @staticmethod
    def set_planar_pose(
        prim: Any,
        x: float,
        y: float,
        yaw: float,
        *,
        height: float,
        UsdGeom: Any,
        Gf: Any,
        rotate: bool = True,
    ) -> None:
        """Write translate (+ optional Z yaw) on a prim."""
        xform = UsdGeom.Xformable(prim)
        translate_op = None
        orient_op = None
        for op in xform.GetOrderedXformOps():
            name = op.GetOpName().lower()
            if "translate" in name and translate_op is None:
                translate_op = op
            if ("orient" in name or "rotate" in name) and orient_op is None:
                orient_op = op
        if translate_op is None or (rotate and orient_op is None):
            xform.ClearXformOpOrder()
            translate_op = xform.AddTranslateOp()
            if rotate:
                orient_op = xform.AddOrientOp()
        translate_op.Set(Gf.Vec3f(float(x), float(y), float(height)))
        if rotate and orient_op is not None:
            half = 0.5 * float(yaw)
            quat = Gf.Quatf(
                float(math.cos(half)),
                Gf.Vec3f(0.0, 0.0, float(math.sin(half))),
            )
            try:
                orient_op.Set(quat)
            except Exception:
                pass


def resolve_project_path(path: str) -> str:
    """Resolve a path relative to the autosim project root (absolute unchanged)."""
    text = str(path or "").strip()
    if not text:
        return ""
    candidate = Path(text).expanduser()
    if candidate.is_absolute():
        return str(candidate)
    try:
        from autosim.runtime.paths import project_root

        return str((project_root() / candidate).resolve())
    except Exception:
        return str(candidate.resolve())


def looks_like_character_usd(path: Path) -> bool:
    name = path.name.lower()
    if not name.endswith((".usd", ".usda", ".usdc")):
        return False
    stem = path.stem.lower()
    parent = path.parent.name.lower()
    # Only check file / parent folder names — full paths often contain "test".
    if any(part in stem or part in parent for part in SKIP_CHARACTER_NAME_PARTS):
        return False
    # Never spawn retarget sources as pedestrians (white / incomplete meshes).
    if "biped" in stem or stem.endswith("_setup"):
        return False
    # Prefer People character packs (F_/M_ names or folder matching stem).
    if stem.startswith(("f_", "m_", "female", "male")):
        return True
    if parent and parent not in {".", "characters"} and stem.startswith(parent[:4]):
        return True
    # Loose files directly under Characters/ are usually Biped_Setup — skip.
    if parent == "characters":
        return False
    return parent not in {"stage", "tests", "data", "ogn"}


def list_character_usds(root: str | Path) -> list[str]:
    """Collect character USD files under an Isaac People ``Characters`` folder."""
    base = Path(root).expanduser()
    if not base.is_dir():
        return []
    # Never recursively scan a whole Isaac install — only Characters layout.
    found: list[Path] = []
    for child in sorted(base.iterdir()):
        if not child.is_dir():
            continue
        if any(part in child.name.lower() for part in SKIP_CHARACTER_NAME_PARTS):
            continue
        usds = sorted(p for p in child.iterdir() if looks_like_character_usd(p))
        if usds:
            found.append(usds[0])
    if not found:
        # One-level files directly under Characters/ (no subfolders).
        found = sorted(
            p for p in base.iterdir() if p.is_file() and looks_like_character_usd(p)
        )
    return [str(p.resolve()) for p in found]


def discover_character_root(explicit: str = "") -> str:
    """Resolve People/Characters directory from config or Isaac install paths."""
    if explicit.strip():
        resolved = resolve_project_path(explicit)
        if Path(resolved).is_dir():
            return resolved
        logger.warning("character_root not a directory: %s", resolved)
    env_keys = (
        "AUTOSIM_PEOPLE_ROOT",
        "ISAAC_PEOPLE_ROOT",
        "ISAAC_PATH",
        "ISAACSIM_PATH",
        "ISAACLAB_PATH",
    )
    candidates: list[Path] = []
    for key in env_keys:
        value = str(os.environ.get(key) or "").strip()
        if not value:
            continue
        root = Path(value).expanduser()
        # Never treat the Isaac/Lab install root itself as Characters.
        if key in ("AUTOSIM_PEOPLE_ROOT", "ISAAC_PEOPLE_ROOT"):
            candidates.append(root)
            candidates.append(root / "Characters")
        candidates.extend(
            [
                root / "People" / "Characters",
                root / "Isaac" / "People" / "Characters",
                root / "assets" / "Isaac" / "People" / "Characters",
                root / "Assets" / "Isaac" / "People" / "Characters",
            ]
        )
    # Common Docker / Nucleus-local mounts.
    for base in (
        Path("/isaac-sim"),
        Path("/isaacsim"),
        Path("/opt/nvidia/isaac-sim"),
        Path("/workspace/isaaclab"),
    ):
        candidates.extend(
            [
                base / "People" / "Characters",
                base / "Assets" / "Isaac" / "People" / "Characters",
                base / "assets" / "Isaac" / "People" / "Characters",
            ]
        )
    try:
        from autosim.runtime.paths import project_root

        candidates.append(project_root() / "usd" / "people" / "Characters")
    except Exception:
        pass
    for path in candidates:
        if not path.is_dir():
            continue
        if list_character_usds(path):
            return str(path.resolve())
        if path.name.lower() == "characters" and any(path.iterdir()):
            return str(path.resolve())
    return ""


def people_sync_config_from_isaac(isaac_block: Mapping[str, Any] | None) -> dict[str, Any]:
    """Read ``habitat.isaac.traffic``."""
    block = {}
    if isinstance(isaac_block, Mapping):
        raw = isaac_block.get("traffic")
        if isinstance(raw, Mapping):
            block = dict(raw)
    yaw_offset = block.get("character_yaw_offset", None)
    if yaw_offset is None and block.get("character_yaw_offset_deg") is not None:
        yaw_offset = math.radians(float(block["character_yaw_offset_deg"]))
    if yaw_offset is None:
        yaw_offset = CHARACTER_YAW_OFFSET
    return {
        "usd_sync": bool(block.get("usd_sync", False)),
        "usd_root": str(block.get("usd_root") or "").strip(),
        "obstacle_visual": str(block.get("obstacle_visual") or "none").strip().lower(),
        "pedestrian_visual": str(
            block.get("pedestrian_visual") or "none"
        ).strip().lower(),
        "character_root": str(block.get("character_root") or "").strip(),
        "character_z": float(block.get("character_z", 0.0)),
        "character_yaw_offset": float(yaw_offset),
        "collision_proxy": bool(block.get("collision_proxy", True)),
        "sync_hz": float(block.get("sync_hz", 30.0)),
        "pedestrian_anim": bool(block.get("pedestrian_anim", True)),
        "anim_walk_mps": float(block.get("anim_walk_mps", 1.2)),
        "animation_root": str(block.get("animation_root") or "").strip(),
    }


def maybe_create_people_sync(
    stage_wrapper: Any,
    *,
    enabled: bool | None = None,
    root_path: str = "",
    config: Mapping[str, Any] | None = None,
) -> PeopleSync | None:
    """Build a sync helper when USD traffic visuals are requested."""
    cfg = dict(config or {})
    obstacle = str(cfg.get("obstacle_visual") or "none").strip().lower()
    pedestrian = str(cfg.get("pedestrian_visual") or "none").strip().lower()
    if enabled is None:
        enabled = bool(cfg.get("usd_sync", False))
    visuals = {"capsule", "invisible", "hidden", "character"}
    if not enabled or (obstacle not in visuals and pedestrian not in visuals):
        return None
    stage = getattr(stage_wrapper, "stage", None)
    if stage is None:
        return None
    assets: list[str] = []
    character_root = ""
    clip_assets: dict[str, str] = {}
    if pedestrian == "character":
        character_root = discover_character_root(str(cfg.get("character_root") or ""))
        assets = list_character_usds(character_root) if character_root else []
        if not assets:
            logger.warning(
                "pedestrian_visual=character: no USDs under %r "
                "(copy Isaac People/Characters or set character_root / AUTOSIM_PEOPLE_ROOT)",
                character_root or cfg.get("character_root") or "(auto)",
            )
        anim_root = discover_animation_root(
            character_root, str(cfg.get("animation_root") or "")
        )
        if anim_root:
            clip_assets = list_animation_usds(anim_root)
    path = str(root_path or cfg.get("usd_root") or "/Root/AutosimTraffic")
    anim_on = bool(cfg.get("pedestrian_anim", True)) and pedestrian == "character"
    animator = PeopleAnimator(
        enabled=anim_on,
        walk_mps=float(cfg.get("anim_walk_mps", 1.2)),
        clip_assets=clip_assets,
        character_root=character_root,
    )
    sync = PeopleSync(
        stage,
        root_path=path,
        enabled=True,
        obstacle_visual=obstacle,
        pedestrian_visual=pedestrian,
        character_assets=assets,
        character_z=float(cfg.get("character_z", 0.0)),
        character_yaw_offset=float(
            cfg.get("character_yaw_offset", CHARACTER_YAW_OFFSET)
        ),
        collision_proxy=bool(cfg.get("collision_proxy", True)),
        sync_hz=float(cfg.get("sync_hz", 30.0)),
        animator=animator,
    )
    logger.info(
        "Isaac traffic USD sync at %s (obstacle=%s pedestrian=%s characters=%d "
        "proxy=%s sync_hz=%.1f anim=%s clips=%s)",
        path,
        sync.obstacle_visual,
        sync.pedestrian_visual,
        len(assets),
        sync.collision_proxy,
        1.0 / sync.sync_period if sync.sync_period > 0 else 0.0,
        anim_on,
        sorted(clip_assets),
    )
    return sync
