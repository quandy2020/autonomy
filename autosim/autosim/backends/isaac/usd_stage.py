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

from pathlib import Path
from typing import Any, Optional

from autosim.backends.isaac.app import ensure_simulation_app
from autosim.backends.isaac.coords import map_to_world


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
        ctx = omni.usd.get_context()
        if not ctx.open_stage(str(self.world_usd)):
            raise RuntimeError(f"failed to open USD stage: {self.world_usd}")
        self.stage: Any = ctx.get_stage()
        if self.stage is None:
            raise RuntimeError("omni.usd context has no stage after open")
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
