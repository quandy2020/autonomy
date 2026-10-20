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

"""Open Beni floating-base Articulation in Isaac PhysX (balance + fall).

Imports ``beni.urdf`` via ``URDFParseAndImportFile`` (``fix_base=False``),
wraps ``SingleArticulation``, and maps ONNX 7-DoF actions onto joint drives.
"""

from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from autosim.backends.isaac.articulation import (
    ensure_articulation_dynamic,
    find_articulation_root,
    kit_log,
    prim_exists,
    robot_tree_root,
    urdf_renamed_for_import,
)
from autosim.backends.isaac.transforms import map_to_world, world_to_map
from autosim.robot.policy.beni import (
    ACTION_JOINTS,
    ACTION_SCALE,
    DEFAULT_Q,
    JOINT_CHILD_LINK,
)

logger = logging.getLogger(__name__)

# PhysX DriveAPI gains (small wheeled-biped; tune if MuJoCo→PhysX drifts).
_DRIVE_STIFFNESS = {
    "L1": 40.0,
    "R1": 40.0,
    "L2": 40.0,
    "R2": 40.0,
    "head_yaw_joint": 20.0,
}
_DRIVE_DAMPING = {
    "L1": 2.0,
    "R1": 2.0,
    "L2": 2.0,
    "R2": 2.0,
    "head_yaw_joint": 1.0,
    "LW": 0.8,
    "RW": 0.8,
}

# InteriorAgent worlds already author ``/beni/base_footprint`` with Physics /
# Robot / Sensor variant payloads. Importing Open Beni under that path corrupts
# PhysX (doubled base_footprint, static joints). Use a unique robot name.
BENI_IMPORT_NAME = "AutosimBeni"

def find_beni_articulation_on_stage(stage: Any) -> str | None:
    """Stage-wide search for Beni ``ArticulationRootAPI`` (any mount path)."""
    from pxr import UsdPhysics

    hits: list[str] = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            continue
        path = str(prim.GetPath())
        lower = path.lower()
        if any(
            key in lower
            for key in ("autosimbeni", "beni", "base_footprint", "robot")
        ):
            hits.append(path)
    if not hits:
        return None
    # Prefer our unique import name over InteriorAgent's reserved /beni.
    for prefer in (
        f"/{BENI_IMPORT_NAME}/",
        f"/{BENI_IMPORT_NAME}",
        "/Root/Robot/",
        "/World/Robot/",
    ):
        for path in hits:
            if path == prefer.rstrip("/") or path.startswith(
                prefer if prefer.endswith("/") else prefer + "/"
            ):
                return path
    for path in hits:
        if BENI_IMPORT_NAME.lower() in path.lower():
            return path
    # Last: ignore bare InteriorAgent /beni variant stubs when possible.
    for path in hits:
        if path.startswith("/beni"):
            continue
        return path
    return hits[0]

def deactivate_autosim_beni(stage: Any) -> None:
    """Hide leftover ``/AutosimBeni`` after falling back to the kinematic capsule."""
    for path in (f"/{BENI_IMPORT_NAME}", f"/Root/{BENI_IMPORT_NAME}"):
        try:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid() and prim.IsActive():
                prim.SetActive(False)
                kit_log(f"deactivated leftover articulation tree {path}")
        except Exception:
            continue

def deactivate_scene_beni_stub(stage: Any) -> None:
    """Disable InteriorAgent's reserved ``/beni`` variant stub (not our robot)."""
    stub = stage.GetPrimAtPath("/beni/base_footprint")
    if stub and stub.IsValid():
        try:
            vsets = stub.GetVariantSets()
            for name, value in (
                ("Physics", "None"),
                ("Robot", "None"),
                ("Sensor", "None"),
            ):
                if vsets.HasVariantSet(name):
                    vsets.GetVariantSet(name).SetVariantSelection(value)
        except Exception as exc:  # noqa: BLE001
            logger.debug("beni stub variant clear skipped: %s", exc)
    for path in ("/beni", "/beni/base_footprint"):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        try:
            if prim.IsActive():
                prim.SetActive(False)
                logger.info("deactivated InteriorAgent robot stub %s", path)
        except Exception as exc:  # noqa: BLE001
            logger.debug("could not deactivate %s: %s", path, exc)

def import_beni_urdf(
    urdf_path: str | Path,
    *,
    dest_usd: str | Path | None = None,
    stage_prim: str = "/Root/Robot",
    stage: Any = None,
) -> str:
    """Parse+import Beni URDF into the current stage; return articulation prim.

    InteriorAgent scenes already define ``/beni/base_footprint`` with PhysX /
    Robot variant payloads. Importing Open Beni under that name merges into the
    stub and breaks joints. We rename the URDF robot to ``AutosimBeni`` so the
    importer creates ``/AutosimBeni/...`` instead.

    Args:
        urdf_path: Path to ``beni.urdf``.
        dest_usd: Ignored (API compat).
        stage_prim: Unused for mount (API compat).
        stage: Optional ``Usd.Stage``.

    Returns:
        Articulation root prim path (e.g. ``/AutosimBeni/base_footprint``).
    """
    del dest_usd, stage_prim
    import omni.kit.commands
    import omni.usd

    path = Path(urdf_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Beni URDF missing: {path}")

    if stage is None:
        stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("no open USD stage for Beni URDF import")

    deactivate_scene_beni_stub(stage)

    # Drop leftover empty placeholders / failed MovePrim duplicates from prior runs.
    for stale in ("/Root/Robot", "/Root/Robot_01", "/World/Robot"):
        prim = stage.GetPrimAtPath(stale)
        if not prim or not prim.IsValid():
            continue
        if find_articulation_root(stage, stale):
            continue
        try:
            stage.RemovePrim(stale)
            logger.info("removed empty robot placeholder %s", stale)
        except Exception:
            pass

    # Sibling file so mesh paths (meshes/*.obj) still resolve.
    import_urdf = urdf_renamed_for_import(path, BENI_IMPORT_NAME)

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status or import_config is None:
        raise RuntimeError("URDFCreateImportConfig failed")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False  # floating base → gravity / fall
    import_config.distance_scale = 1.0
    try:
        import_config.make_default_prim = False
    except Exception:
        pass

    status, reported = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(import_urdf),
        import_config=import_config,
        get_articulation_root=True,
    )
    if not status:
        raise RuntimeError(f"URDFParseAndImportFile failed for {import_urdf}")
    reported_path = str(reported or "").strip()
    logger.info("URDFParseAndImportFile reported %s", reported_path or "(none)")

    try:
        from autosim.backends.isaac.app import update_app

        for _ in range(3):
            update_app()
    except Exception:
        pass

    from pxr import UsdPhysics

    mount = f"/{BENI_IMPORT_NAME}"
    art: str | None = None
    if reported_path and prim_exists(stage, reported_path):
        art = find_articulation_root(stage, reported_path)
        if art is None:
            prim = stage.GetPrimAtPath(reported_path)
            if prim and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                art = reported_path
    if art is None:
        for candidate in (
            f"{mount}/base_footprint",
            mount,
        ):
            if prim_exists(stage, candidate):
                art = find_articulation_root(stage, candidate)
                if art:
                    break
    if art is None:
        art = find_beni_articulation_on_stage(stage)
    if not art:
        raise RuntimeError(
            f"Beni articulation prim missing after URDF import "
            f"(reported={reported_path!r}, expected under {mount})"
        )

    ensure_articulation_dynamic(stage, art)
    kit_log(
        f"imported Beni URDF → articulation at {art} (tree {robot_tree_root(art)})"
    )
    return art

def configure_joint_drives(stage: Any, articulation_root: str) -> int:
    """Set angular DriveAPI stiffness/damping on Beni joints under ``root``."""
    from pxr import Usd, UsdPhysics

    configured = 0
    root = stage.GetPrimAtPath(articulation_root)
    if not root or not root.IsValid():
        return 0
    # Joints may live under root or root/joints.
    for prim in Usd.PrimRange(root):
        name = prim.GetName()
        if name not in ACTION_JOINTS and name not in (
            "L1",
            "R1",
            "L2",
            "R2",
            "LW",
            "RW",
            "head_yaw_joint",
        ):
            continue
        if not prim.IsA(UsdPhysics.Joint) and "Joint" not in prim.GetTypeName():
            # Importer often uses PhysicsRevoluteJoint etc.
            type_name = prim.GetTypeName()
            if "Joint" not in type_name and "Revolute" not in type_name and "Continuous" not in type_name:
                continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            try:
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            except Exception:
                continue
        joint = name
        if joint in ("LW", "RW"):
            drive.GetStiffnessAttr().Set(0.0)
            drive.GetDampingAttr().Set(float(_DRIVE_DAMPING.get(joint, 0.8)))
        else:
            drive.GetStiffnessAttr().Set(float(_DRIVE_STIFFNESS.get(joint, 40.0)))
            drive.GetDampingAttr().Set(float(_DRIVE_DAMPING.get(joint, 2.0)))
        configured += 1
    logger.info("configured %d Beni joint drives under %s", configured, articulation_root)
    return configured

class BeniArticulation:
    """PhysX floating-base Beni driven by Open Beni ONNX actions."""

    def __init__(self, prim_path: str) -> None:
        self.prim_path = str(prim_path)
        self.art: Any = None
        self._dof_index: dict[str, int] = {}
        self._ready = False

    @property
    def ready(self) -> bool:
        return bool(self._ready and self.art is not None)

    def initialize(self) -> bool:
        """Bind ``SingleArticulation`` after PhysX World is playing."""
        try:
            from isaacsim.core.prims import SingleArticulation
        except ImportError:
            try:
                from isaacsim.core.api.articulations import Articulation as SingleArticulation
            except ImportError as exc:
                logger.warning("SingleArticulation unavailable: %s", exc)
                return False

        # Resolve path if the reported root was stale; wait for USD compose.
        if self.prim_path.endswith("/base_footprint/base_footprint"):
            self.prim_path = self.prim_path[: -len("/base_footprint")]
        try:
            import omni.usd
            from autosim.backends.isaac.app import update_app

            stage = omni.usd.get_context().get_stage()
            for _ in range(5):
                if stage and prim_exists(stage, self.prim_path):
                    break
                update_app()
            if stage is not None:
                found = None
                if not prim_exists(stage, self.prim_path):
                    parent = self.prim_path.rsplit("/", 1)[0] or "/"
                    found = find_articulation_root(stage, parent)
                if found is None and BENI_IMPORT_NAME not in self.prim_path:
                    found = find_beni_articulation_on_stage(stage)
                if found and found != self.prim_path:
                    logger.info(
                        "Beni articulation path remapped %s → %s",
                        self.prim_path,
                        found,
                    )
                    self.prim_path = found
                if not prim_exists(stage, self.prim_path):
                    logger.warning(
                        "Beni articulation prim missing: %s", self.prim_path
                    )
                    return False
        except Exception as exc:  # noqa: BLE001
            logger.debug("articulation prim resolve skipped: %s", exc)

        art = None
        last_exc: BaseException | None = None
        for attempt in range(4):
            try:
                art = SingleArticulation(prim_path=self.prim_path, name="beni_art")
                art.initialize()
                last_exc = None
                break
            except Exception as exc:  # noqa: BLE001
                last_exc = exc
                art = None
                try:
                    from autosim.backends.isaac.app import update_app

                    update_app()
                except Exception:
                    pass
        if art is None:
            logger.warning(
                "Beni Articulation.initialize failed: %s", last_exc
            )
            return False
        if hasattr(art, "is_physics_handle_valid") and not art.is_physics_handle_valid():
            logger.warning("Beni articulation physics handle invalid at %s", self.prim_path)
            return False
        try:
            art.enable_gravity()
        except Exception:
            pass
        self.art = art
        self._dof_index = {}
        names = list(getattr(art, "dof_names", None) or [])
        for name in ACTION_JOINTS:
            try:
                self._dof_index[name] = int(art.get_dof_index(name))
            except Exception:
                # Fuzzy match (importer may rename).
                for dof in names:
                    if name.lower() in str(dof).lower() or str(dof).lower() in name.lower():
                        try:
                            self._dof_index[name] = int(art.get_dof_index(dof))
                        except Exception:
                            self._dof_index[name] = names.index(dof)
                        break
        missing = [n for n in ACTION_JOINTS if n not in self._dof_index]
        if missing:
            logger.warning("Beni articulation missing DOFs %s (have %s)", missing, names)
        self._ready = len(self._dof_index) >= 4
        kit_log(
            f"Beni Articulation ready at {self.prim_path} "
            f"({len(self._dof_index)}/{len(ACTION_JOINTS)} DOFs mapped)"
        )
        return self._ready

    def teleport(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        z: float = 0.11,
        joint_defaults: bool = True,
    ) -> None:
        """Reset root pose (and optionally standing joint angles)."""
        if not self.ready:
            return
        translation, quat_wxyz = map_to_world(x, y, yaw, z=z)
        try:
            self.art.set_world_pose(position=translation, orientation=quat_wxyz)
        except TypeError:
            try:
                self.art.set_world_pose(translation, quat_wxyz)
            except Exception as exc:  # noqa: BLE001
                logger.debug("set_world_pose failed: %s", exc)
        # Only clear velocity when the root is dynamic — kinematic writes spam
        # PhysX and eventually stop the simulation.
        try:
            import omni.usd
            from pxr import UsdPhysics

            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath(self.prim_path) if stage else None
            kinematic = True
            if prim and prim.IsValid() and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                attr = UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr()
                kinematic = bool(attr and attr.Get())
            if not kinematic:
                try:
                    self.art.set_world_velocity(np.zeros(6, dtype=np.float64))
                except Exception:
                    try:
                        self.art.set_linear_velocity(np.zeros(3))
                        self.art.set_angular_velocity(np.zeros(3))
                    except Exception:
                        pass
        except Exception:
            pass
        if joint_defaults:
            self.reset_standing_joints()

    def reset_standing_joints(self) -> None:
        """Snap actuated joints to Open Beni standing defaults."""
        if not self.ready:
            return
        n = int(self.art.num_dof)
        pos = np.zeros(n, dtype=np.float64)
        vel = np.zeros(n, dtype=np.float64)
        try:
            cur = np.asarray(self.art.get_joint_positions(), dtype=np.float64).reshape(-1)
            if cur.size == n:
                pos[:] = cur
        except Exception:
            pass
        for name, q0 in DEFAULT_Q.items():
            if name not in self._dof_index:
                continue
            pos[self._dof_index[name]] = float(q0)
        try:
            self.art.set_joint_positions(pos)
            self.art.set_joint_velocities(vel)
        except Exception as exc:  # noqa: BLE001
            logger.debug("reset_standing_joints failed: %s", exc)

    def read_pose(self) -> tuple[float, float, float, float]:
        """Return ``(x, y, yaw, z)`` from articulation root."""
        if not self.ready:
            return (0.0, 0.0, 0.0, 0.11)
        try:
            pos, orn = self.art.get_world_pose()
        except Exception:
            return (0.0, 0.0, 0.0, 0.11)
        x, y, yaw = world_to_map(np.asarray(pos), np.asarray(orn))
        z = float(np.asarray(pos).reshape(3)[2])
        return (float(x), float(y), float(yaw), z)

    def read_angular_velocity(self) -> tuple[float, float, float]:
        """Body / world angular velocity (rad/s)."""
        if not self.ready:
            return (0.0, 0.0, 0.0)
        try:
            w = np.asarray(self.art.get_angular_velocity(), dtype=np.float64).reshape(-1)
            if w.size >= 3:
                return (float(w[0]), float(w[1]), float(w[2]))
        except Exception:
            pass
        try:
            v = np.asarray(self.art.get_world_velocity(), dtype=np.float64).reshape(-1)
            if v.size >= 6:
                return (float(v[3]), float(v[4]), float(v[5]))
        except Exception:
            pass
        return (0.0, 0.0, 0.0)

    def projected_gravity(self) -> tuple[float, float, float]:
        """Gravity direction in base frame (Open Beni obs layout)."""
        if not self.ready:
            return (0.0, 0.0, -1.0)
        try:
            from autosim.backends.isaac.transforms import quat_wxyz_to_matrix

            _pos, orn = self.art.get_world_pose()
            wxyz = np.asarray(orn, dtype=np.float64).reshape(4)
            r = quat_wxyz_to_matrix(wxyz)
            g = r.T @ np.array([0.0, 0.0, -1.0], dtype=np.float64)
            n = float(np.linalg.norm(g))
            if n > 1e-9:
                g = g / n
            return (float(g[0]), float(g[1]), float(g[2]))
        except Exception:
            return (0.0, 0.0, -1.0)

    def sync_joint_state(
        self, q: dict[str, float], dq: dict[str, float]
    ) -> None:
        """Copy measured joint positions/velocities into policy dicts."""
        if not self.ready:
            return
        try:
            pos = np.asarray(self.art.get_joint_positions(), dtype=np.float64).reshape(-1)
            vel = np.asarray(self.art.get_joint_velocities(), dtype=np.float64).reshape(-1)
        except Exception:
            return
        for name, index in self._dof_index.items():
            if 0 <= index < pos.size:
                q[name] = float(pos[index])
            if 0 <= index < vel.size:
                dq[name] = float(vel[index])

    def apply_policy_action(self, action: Sequence[float]) -> None:
        """Map clipped ONNX action[7] → joint position / wheel velocity targets."""
        if not self.ready:
            return
        from isaacsim.core.utils.types import ArticulationAction

        n = int(self.art.num_dof)
        positions = [None] * n
        velocities = [None] * n
        a = np.asarray(action, dtype=np.float64).reshape(-1)
        for index, name in enumerate(ACTION_JOINTS):
            if name not in self._dof_index or index >= a.size:
                continue
            dof = self._dof_index[name]
            scale = ACTION_SCALE[name]
            if name in ("LW", "RW"):
                velocities[dof] = float(np.clip(a[index], -1.0, 1.0) * scale)
                positions[dof] = None
            else:
                positions[dof] = float(DEFAULT_Q[name] + np.clip(a[index], -1.0, 1.0) * scale)
                velocities[dof] = None
        try:
            self.art.apply_action(
                ArticulationAction(
                    joint_positions=positions,
                    joint_velocities=velocities,
                )
            )
        except Exception as exc:  # noqa: BLE001
            logger.debug("apply_policy_action failed: %s", exc)

    def display_angles(self) -> dict[str, float]:
        """Mesh-relative joint angles for Autoviz TF (wheels omitted)."""
        if not self.ready:
            return {}
        q: dict[str, float] = {}
        dq: dict[str, float] = {}
        self.sync_joint_state(q, dq)
        out: dict[str, float] = {}
        for name, link in JOINT_CHILD_LINK.items():
            if name in ("LW", "RW") or name not in q:
                continue
            out[link] = float(q[name] - DEFAULT_Q.get(name, 0.0))
        return out

    def fallen(self, *, roll_pitch_limit: float = 1.0, z_min: float = 0.03) -> bool:
        """Heuristic: base nearly flat on floor or extreme tilt."""
        x, y, yaw, z = self.read_pose()
        del x, y, yaw
        if z < float(z_min):
            return True
        g = self.projected_gravity()
        # Upright ⇒ g ≈ (0,0,-1); tip when |gx|,|gy| large.
        tilt = math.sqrt(g[0] * g[0] + g[1] * g[1])
        return tilt > float(roll_pitch_limit) * 0.7
