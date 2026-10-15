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

"""Sync Traffic agents into the Isaac USD stage as kinematic capsules.

Obstacles and pedestrians share the same capsule path. CPU ``Traffic`` still
owns planar motion and ``/scan`` casting; this module only mirrors poses for
PhysX / RGB.
"""

from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Any, Mapping, Optional

import numpy as np

logger = logging.getLogger(__name__)


class TrafficUsdSync:
    """Mirror Traffic into USD kinematic capsules (obstacles + pedestrians)."""

    def __init__(
        self,
        stage: Any,
        *,
        root_path: str = "/Root/AutosimTraffic",
        enabled: bool = True,
        obstacle_visual: str = "capsule",
        pedestrian_visual: str = "capsule",
    ) -> None:
        self.stage = stage
        self.root_path = str(root_path).rstrip("/") or "/Root/AutosimTraffic"
        self.enabled = bool(enabled)
        self.obstacle_visual = str(obstacle_visual or "capsule").strip().lower()
        self.pedestrian_visual = str(pedestrian_visual or "capsule").strip().lower()
        self.ready = False

    def sync(self, traffic: Any) -> None:
        """Mirror Traffic poses into the open USD stage."""
        if not self.enabled or traffic is None or self.stage is None:
            return
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
        if obstacle is not None and self.obstacle_visual in (
            "capsule",
            "invisible",
            "hidden",
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
        if pedestrian is not None and self.pedestrian_visual in (
            "capsule",
            "invisible",
            "hidden",
        ):
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
        from autosim.paths import project_root

        return str((project_root() / candidate).resolve())
    except Exception:
        return str(candidate.resolve())


def traffic_config_from_isaac(isaac_block: Mapping[str, Any] | None) -> dict[str, Any]:
    """Read ``habitat.isaac.traffic`` with defaults (capsule-only USD sync)."""
    block = {}
    if isinstance(isaac_block, Mapping):
        raw = isaac_block.get("traffic")
        if isinstance(raw, Mapping):
            block = dict(raw)
    return {
        "usd_sync": bool(block.get("usd_sync", True)),
        "usd_root": str(block.get("usd_root") or "").strip(),
        "obstacle_visual": str(block.get("obstacle_visual") or "capsule").strip().lower(),
        "pedestrian_visual": str(
            block.get("pedestrian_visual") or "capsule"
        ).strip().lower(),
    }


def maybe_create_traffic_usd(
    stage_wrapper: Any,
    *,
    enabled: bool | None = None,
    root_path: str = "",
    config: Mapping[str, Any] | None = None,
) -> Optional[TrafficUsdSync]:
    """Build a sync helper when USD visualization of traffic is requested."""
    cfg = dict(config or {})
    if enabled is None:
        enabled = bool(cfg.get("usd_sync", True))
    if not enabled:
        return None
    stage = getattr(stage_wrapper, "stage", None)
    if stage is None:
        return None
    path = str(root_path or cfg.get("usd_root") or "/Root/AutosimTraffic")
    sync = TrafficUsdSync(
        stage,
        root_path=path,
        enabled=True,
        obstacle_visual=str(cfg.get("obstacle_visual") or "capsule"),
        pedestrian_visual=str(cfg.get("pedestrian_visual") or "capsule"),
    )
    logger.info(
        "Isaac traffic USD sync at %s (obstacle=%s pedestrian=%s)",
        path,
        sync.obstacle_visual,
        sync.pedestrian_visual,
    )
    return sync
