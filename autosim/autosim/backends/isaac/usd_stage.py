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

"""Open USD stage, resolve robot prim, apply planar poses."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Optional, Sequence

import numpy as np

from autosim.backends.isaac.app import ensure_simulation_app
from autosim.backends.isaac.coords import map_to_world

_LOG = logging.getLogger(__name__)

# InteriorAgent / 酷家乐 meshes often author a single displayColor/Opacity value
# with faceVarying interpolation; Hydra then spams "corrupted data in primvar".
_DISPLAY_PRIMVARS = frozenset({"displayColor", "displayOpacity"})


def require_isaac() -> None:
    """Ensure Isaac Kit is up so ``omni.usd`` can be imported.

    Raises:
        ImportError: Isaac Sim / Isaac Lab Python is unavailable, or the process
            was not started with Isaac's interpreter (see run.sh).
    """
    try:
        import isaacsim  # noqa: F401
    except ImportError as exc:
        raise ImportError(
            "habitat.backend=isaac requires Isaac Sim Python. "
            "Set ISAACLAB_PATH (default /workspace/isaaclab) and run via "
            "scripts/run.sh so it uses isaaclab.sh -p / python.sh."
        ) from exc
    ensure_simulation_app()
    try:
        import omni.usd  # noqa: F401
    except ImportError as exc:
        raise ImportError(
            "omni.usd unavailable after SimulationApp start. "
            "Use Isaac Lab's python (e.g. /workspace/isaaclab/isaaclab.sh -p)."
        ) from exc


def mute_hydra_primvar_warnings() -> None:
    """Drop Hydra channel below warning so residual primvar spam stays quiet."""
    try:
        import carb

        carb.settings.get_settings().set_string(
            "/log/channels/omni.hydra", "error"
        )
    except Exception:
        pass


def repair_display_primvars(stage: Any) -> int:
    """Rewrite length-1 faceVarying display* primvars to constant.

    Returns:
        Number of primvars corrected.
    """
    from pxr import UsdGeom

    fixed = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        api = UsdGeom.PrimvarsAPI(prim)
        for pv in api.GetPrimvars():
            try:
                name = pv.GetPrimvarName()
            except Exception:
                name = pv.GetBaseName()
            if name not in _DISPLAY_PRIMVARS:
                continue
            try:
                interp = pv.GetInterpolation()
            except Exception:
                continue
            if interp != UsdGeom.Tokens.faceVarying:
                continue
            try:
                value = pv.Get()
            except Exception:
                continue
            if value is None:
                continue
            try:
                length = len(value)
            except TypeError:
                length = 1
            if length != 1:
                continue
            try:
                pv.SetInterpolation(UsdGeom.Tokens.constant)
                fixed += 1
            except Exception:
                continue
    if fixed:
        _LOG.info("repaired %d displayColor/Opacity primvars (constant)", fixed)
    return fixed


class UsdStage:
    """Thin wrapper around an Omniverse USD stage + robot Xform."""

    def __init__(
        self,
        world_usd: Path,
        *,
        robot_usd: Optional[Path] = None,
        robot_prim_path: str = "/World/Robot",
        base_z: float = 0.0,
    ) -> None:
        require_isaac()
        import omni.usd
        from pxr import UsdGeom

        self.world_usd = Path(world_usd)
        if not self.world_usd.is_file():
            raise FileNotFoundError(f"Isaac USD world not found: {self.world_usd}")
        self.robot_usd = Path(robot_usd) if robot_usd else None
        self.robot_prim_path = robot_prim_path
        self.base_z = float(base_z)
        self._omni = omni.usd
        self._UsdGeom = UsdGeom
        mute_hydra_primvar_warnings()
        ctx = omni.usd.get_context()
        if not ctx.open_stage(str(self.world_usd)):
            raise RuntimeError(f"failed to open USD stage: {self.world_usd}")
        self.stage: Any = ctx.get_stage()
        if self.stage is None:
            raise RuntimeError("omni.usd context has no stage after open")
        # Fix InteriorAgent authoring before the first Hydra sync.
        repair_display_primvars(self.stage)
        self._ensure_robot_prim()
        self._robot_xform = UsdGeom.Xformable(
            self.stage.GetPrimAtPath(self.robot_prim_path)
        )

    def _ensure_robot_prim(self) -> None:
        from pxr import UsdGeom

        prim = self.stage.GetPrimAtPath(self.robot_prim_path)
        if prim and prim.IsValid():
            return
        if self.robot_usd is not None and self.robot_usd.is_file():
            robot = self.stage.DefinePrim(self.robot_prim_path, "Xform")
            robot.GetReferences().AddReference(str(self.robot_usd))
            return
        UsdGeom.Xform.Define(self.stage, self.robot_prim_path)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        translation, quat_wxyz = map_to_world(x, y, yaw, z=self.base_z)
        from pxr import Gf, UsdGeom

        ops = self._robot_xform.GetOrderedXformOps()
        if not ops:
            self._robot_xform.ClearXformOpOrder()
            t_op = self._robot_xform.AddTranslateOp()
            # Isaac / USD Xformable defaults to float precision for orient.
            o_op = self._robot_xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
        else:
            t_op = None
            o_op = None
            for op in ops:
                name = op.GetOpName()
                if "translate" in name.lower():
                    t_op = op
                elif "orient" in name.lower() or "rotate" in name.lower():
                    o_op = op
            if t_op is None:
                t_op = self._robot_xform.AddTranslateOp()
            if o_op is None:
                o_op = self._robot_xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
        # Match op precision (float is common on Isaac stages).
        try:
            t_op.Set(
                Gf.Vec3f(
                    float(translation[0]),
                    float(translation[1]),
                    float(translation[2]),
                )
            )
        except Exception:
            t_op.Set(
                Gf.Vec3d(
                    float(translation[0]),
                    float(translation[1]),
                    float(translation[2]),
                )
            )
        w, xq, yq, zq = [float(v) for v in quat_wxyz]
        try:
            o_op.Set(Gf.Quatf(w, Gf.Vec3f(xq, yq, zq)))
        except Exception:
            o_op.Set(Gf.Quatd(w, Gf.Vec3d(xq, yq, zq)))

    def close(self) -> None:
        try:
            ctx = self._omni.get_context()
            ctx.close_stage()
        except Exception:
            pass

    def sample_mesh_cloud(
        self,
        *,
        z_min: float = 0.0,
        z_max: float = 3.0,
        stride: int = 1,
        max_points: int = 2_000_000,
        skip_prim_prefixes: Sequence[str] = (),
    ) -> np.ndarray:
        """Sample world-frame mesh vertices for ``/overall/map``.

        InteriorAgent scenes keep geometry in referenced Mesh USDs; room
        polygons alone only outline walls. Vertex sampling recovers furniture
        and full wall extents.
        """
        return sample_mesh_cloud(
            self.stage,
            z_min=z_min,
            z_max=z_max,
            stride=stride,
            max_points=max_points,
            skip_prim_prefixes=(self.robot_prim_path, *skip_prim_prefixes),
        )


def sample_mesh_cloud(
    stage: Any,
    *,
    z_min: float = 0.0,
    z_max: float = 3.0,
    stride: int = 1,
    max_points: int = 2_000_000,
    skip_prim_prefixes: Sequence[str] = (),
) -> np.ndarray:
    """Collect ``UsdGeom.Mesh`` points in world frame (z-up metres)."""
    from pxr import Usd, UsdGeom

    if stage is None:
        return np.zeros((0, 3), dtype=np.float32)
    skip = tuple(str(p) for p in skip_prim_prefixes if str(p).strip())
    step = max(1, int(stride))
    chunks: list[np.ndarray] = []
    total = 0
    time = Usd.TimeCode.Default()
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if any(path == pref or path.startswith(pref.rstrip("/") + "/") for pref in skip):
            continue
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        try:
            points_attr = mesh.GetPointsAttr()
            points = points_attr.Get(time) if points_attr else None
        except Exception:
            continue
        if not points:
            continue
        try:
            xform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(time)
        except Exception:
            continue
        local = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        if local.size == 0:
            continue
        if step > 1:
            local = local[::step]
        # USD uses row vectors: p_world = p_local * matrix.
        matrix = np.asarray(xform, dtype=np.float64).reshape(4, 4)
        ones = np.ones((local.shape[0], 1), dtype=np.float64)
        world = np.hstack([local, ones]) @ matrix
        world = world[:, :3]
        z = world[:, 2]
        keep = (
            np.isfinite(world).all(axis=1)
            & (z >= float(z_min))
            & (z <= float(z_max))
        )
        world = world[keep]
        if world.size == 0:
            continue
        chunks.append(world.astype(np.float32))
        total += int(world.shape[0])
        if total >= int(max_points):
            break
    if not chunks:
        return np.zeros((0, 3), dtype=np.float32)
    cloud = np.vstack(chunks)
    if cloud.shape[0] > int(max_points):
        cloud = cloud[: int(max_points)]
    _LOG.info(
        "sampled USD mesh cloud: %d points (z=[%.2f, %.2f], stride=%d)",
        cloud.shape[0],
        float(z_min),
        float(z_max),
        step,
    )
    return cloud
