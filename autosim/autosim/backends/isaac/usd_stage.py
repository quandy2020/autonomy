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
import math
from pathlib import Path
from typing import Any, Optional, Sequence

import numpy as np

from autosim.backends.isaac.app import ensure_simulation_app, stop_timeline
from autosim.backends.isaac.coords import map_to_world

logger = logging.getLogger(__name__)

# InteriorAgent / 酷家乐 meshes often author a single displayColor/Opacity value
# with faceVarying interpolation; Hydra then spams "corrupted data in primvar".
DISPLAY_PRIMVARS = frozenset({"displayColor", "displayOpacity"})


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
            if name not in DISPLAY_PRIMVARS:
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
        logger.info("repaired %d displayColor/Opacity primvars (constant)", fixed)
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
        self.omni_module = omni.usd
        self.UsdGeom = UsdGeom
        mute_hydra_primvar_warnings()
        # Stop any warm-start play on the anonymous default stage before swap;
        # otherwise SimulationManager tries create_simulation_view with no stage.
        stop_timeline()
        ctx = omni.usd.get_context()
        if not ctx.open_stage(str(self.world_usd)):
            raise RuntimeError(f"failed to open USD stage: {self.world_usd}")
        self.stage: Any = ctx.get_stage()
        if self.stage is None:
            raise RuntimeError("omni.usd context has no stage after open")
        # Fix InteriorAgent authoring before the first Hydra / PhysX sync.
        repair_display_primvars(self.stage)
        try:
            from autosim.backends.isaac.physics import (
                ensure_physics_scene,
                strip_nested_rigid_bodies,
            )

            ensure_physics_scene(self.stage)
            strip_nested_rigid_bodies(
                self.stage, skip_prefix=self.robot_prim_path
            )
        except Exception as exc:
            logger.warning("early physics scene repair skipped: %s", exc)
        self.ensure_robot_prim()
        self.robot_xform = UsdGeom.Xformable(
            self.stage.GetPrimAtPath(self.robot_prim_path)
        )

    def ensure_robot_prim(self) -> None:
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

        ops = self.robot_xform.GetOrderedXformOps()
        if not ops:
            self.robot_xform.ClearXformOpOrder()
            t_op = self.robot_xform.AddTranslateOp()
            # Isaac / USD Xformable defaults to float precision for orient.
            o_op = self.robot_xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
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
                t_op = self.robot_xform.AddTranslateOp()
            if o_op is None:
                o_op = self.robot_xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
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
        # PhysX / Fabric often ignore raw USD xform writes on kinematic bodies;
        # push the same pose through Isaac prim APIs when available.
        self._sync_isaac_prim_pose(translation, quat_wxyz)

    def _sync_isaac_prim_pose(
        self, translation: np.ndarray, quat_wxyz: np.ndarray
    ) -> None:
        """Teleport kinematic robot via Isaac Rigid/XForm prim (Fabric-aware)."""
        pos = np.asarray(translation, dtype=np.float64).reshape(3)
        orn = np.asarray(quat_wxyz, dtype=np.float64).reshape(4)
        view = getattr(self, "_isaac_pose_view", None)
        if view is None:
            try:
                from isaacsim.core.prims import SingleRigidPrim

                view = SingleRigidPrim(
                    prim_path=self.robot_prim_path, name="autosim_robot_rb"
                )
                try:
                    view.initialize()
                except Exception:
                    pass
                self._isaac_pose_view = view
            except Exception:
                try:
                    from isaacsim.core.prims import SingleXFormPrim

                    view = SingleXFormPrim(
                        prim_path=self.robot_prim_path, name="autosim_robot_xf"
                    )
                    try:
                        view.initialize()
                    except Exception:
                        pass
                    self._isaac_pose_view = view
                except Exception:
                    self._isaac_pose_view = False
                    return
        if view is False:
            return
        try:
            view.set_world_pose(position=pos, orientation=orn)
        except Exception as exc:
            logger.debug("isaac prim set_world_pose failed: %s", exc)
            return
        # arena-isaac geom.move: teleports must not preserve momentum.
        self._zero_isaac_prim_velocities(view)

    @staticmethod
    def _zero_isaac_prim_velocities(view: Any) -> None:
        """Clear linear/angular velocity after a kinematic teleport."""
        try:
            set_vel = getattr(view, "set_velocities", None)
            if callable(set_vel):
                set_vel(np.zeros(3, dtype=np.float64), np.zeros(3, dtype=np.float64))
                return
        except Exception:
            pass
        try:
            set_lin = getattr(view, "set_linear_velocity", None)
            set_ang = getattr(view, "set_angular_velocity", None)
            if callable(set_lin):
                set_lin(np.zeros(3, dtype=np.float64))
            if callable(set_ang):
                set_ang(np.zeros(3, dtype=np.float64))
        except Exception as exc:
            logger.debug("isaac prim zero velocity skipped: %s", exc)

    def close(self) -> None:
        try:
            ctx = self.omni_module.get_context()
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
        surface_spacing_m: float = 0.05,
        skip_prim_prefixes: Sequence[str] = (),
        skip_path_substrings: Sequence[str] = (),
        only_path_substrings: Sequence[str] = (),
    ) -> np.ndarray:
        """Sample world-frame surface points for ``/overall/map``.

        Densifies triangle interiors (not only vertices) so walls look solid.
        """
        return sample_mesh_cloud(
            self.stage,
            z_min=z_min,
            z_max=z_max,
            stride=stride,
            max_points=max_points,
            surface_spacing_m=surface_spacing_m,
            skip_prim_prefixes=(self.robot_prim_path, *skip_prim_prefixes),
            skip_path_substrings=skip_path_substrings,
            only_path_substrings=only_path_substrings,
        )


def sample_triangle_surface(
    v0: np.ndarray,
    v1: np.ndarray,
    v2: np.ndarray,
    spacing: float,
) -> np.ndarray:
    """Uniform barycentric samples on a triangle (includes vertices)."""
    e01 = float(np.linalg.norm(v1 - v0))
    e02 = float(np.linalg.norm(v2 - v0))
    e12 = float(np.linalg.norm(v2 - v1))
    longest = max(e01, e02, e12, 1e-6)
    step = max(float(spacing), 1e-3)
    n = max(1, int(math.ceil(longest / step)))
    pts: list[np.ndarray] = []
    for i in range(n + 1):
        for j in range(n + 1 - i):
            u = i / n
            v = j / n
            w = 1.0 - u - v
            pts.append(w * v0 + u * v1 + v * v2)
    return np.asarray(pts, dtype=np.float64)


def densify_mesh_local(
    local: np.ndarray,
    counts: Sequence[int] | None,
    indices: Sequence[int] | None,
    spacing: float,
) -> np.ndarray:
    """Fan-triangulate faces and sample each triangle at ``spacing`` metres."""
    if counts is None or indices is None or len(counts) == 0:
        return local
    idx = np.asarray(indices, dtype=np.int64).reshape(-1)
    verts = np.asarray(local, dtype=np.float64).reshape(-1, 3)
    n_verts = verts.shape[0]
    chunks: list[np.ndarray] = [verts]
    cursor = 0
    for count in counts:
        n = int(count)
        if n < 3 or cursor + n > idx.shape[0]:
            cursor += max(n, 0)
            continue
        face = idx[cursor : cursor + n]
        cursor += n
        if np.any(face < 0) or np.any(face >= n_verts):
            continue
        v0 = verts[int(face[0])]
        for k in range(1, n - 1):
            v1 = verts[int(face[k])]
            v2 = verts[int(face[k + 1])]
            edge = max(
                float(np.linalg.norm(v1 - v0)),
                float(np.linalg.norm(v2 - v0)),
                float(np.linalg.norm(v2 - v1)),
            )
            # Tiny faces: keep the three corners only (already in verts).
            if edge < 1.5 * float(spacing):
                continue
            chunks.append(sample_triangle_surface(v0, v1, v2, spacing))
    return np.vstack(chunks) if chunks else verts


def sample_mesh_cloud(
    stage: Any,
    *,
    z_min: float = 0.0,
    z_max: float = 3.0,
    stride: int = 1,
    max_points: int = 2_000_000,
    surface_spacing_m: float = 0.05,
    skip_prim_prefixes: Sequence[str] = (),
    skip_path_substrings: Sequence[str] = (),
    only_path_substrings: Sequence[str] = (),
) -> np.ndarray:
    """Collect dense surface samples from ``UsdGeom.Mesh`` (z-up metres).

    Large wall quads often have only 4 vertices — vertex-only clouds look like
    wireframes. Face densification at ``surface_spacing_m`` fills solid walls.
    """
    from pxr import Usd, UsdGeom

    if stage is None:
        return np.zeros((0, 3), dtype=np.float32)
    skip = tuple(str(p) for p in skip_prim_prefixes if str(p).strip())
    skip_sub = tuple(s.lower() for s in skip_path_substrings if str(s).strip())
    only_sub = tuple(s.lower() for s in only_path_substrings if str(s).strip())
    step = max(1, int(stride))
    spacing = max(float(surface_spacing_m), 0.0)
    chunks: list[np.ndarray] = []
    total = 0
    time = Usd.TimeCode.Default()
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        path_l = path.lower()
        if any(path == pref or path.startswith(pref.rstrip("/") + "/") for pref in skip):
            continue
        if skip_sub and any(s in path_l for s in skip_sub):
            continue
        if only_sub and not any(s in path_l for s in only_sub):
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
        if spacing > 0.0:
            try:
                counts = mesh.GetFaceVertexCountsAttr().Get(time)
                indices = mesh.GetFaceVertexIndicesAttr().Get(time)
            except Exception:
                counts, indices = None, None
            local = densify_mesh_local(local, counts, indices, spacing)
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
    logger.info(
        "sampled USD mesh cloud: %d points (z=[%.2f, %.2f], spacing=%.3f, stride=%d)",
        cloud.shape[0],
        float(z_min),
        float(z_max),
        spacing,
        step,
    )
    return cloud
