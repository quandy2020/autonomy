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

"""Isaac PhysX bootstrap: World, scene colliders, kinematic robot capsule."""

from __future__ import annotations

import logging
from typing import Any, Mapping, Optional

_LOG = logging.getLogger(__name__)

_DEFAULTS = {
    "enabled": True,
    "block_motion": True,
    "dt": 1.0 / 60.0,
    "robot_radius": 0.12,
    "robot_height": 0.28,
    "freeze_scene": True,
    "add_ground": False,
    "ensure_scene_colliders": True,
}


def physics_config_from_habitat(habitat: Mapping[str, Any]) -> dict[str, Any]:
    """Merge ``habitat.isaac.physics`` with defaults."""
    isaac = habitat.get("isaac") if isinstance(habitat.get("isaac"), Mapping) else {}
    block = {}
    if isinstance(isaac, Mapping):
        raw = isaac.get("physics")
        if isinstance(raw, Mapping):
            block = dict(raw)
        elif "physics" not in isaac and isinstance(habitat.get("physics"), Mapping):
            block = dict(habitat["physics"])
    out = dict(_DEFAULTS)
    out.update(block)
    out["enabled"] = bool(out.get("enabled", True))
    out["block_motion"] = bool(out.get("block_motion", True))
    out["freeze_scene"] = bool(out.get("freeze_scene", True))
    out["add_ground"] = bool(out.get("add_ground", False))
    out["ensure_scene_colliders"] = bool(out.get("ensure_scene_colliders", True))
    out["dt"] = float(out.get("dt", 1.0 / 60.0))
    out["robot_radius"] = float(out.get("robot_radius", 0.12))
    out["robot_height"] = float(out.get("robot_height", 0.28))
    return out


class IsaacPhysics:
    """Owns ``isaacsim.core.api.world.World`` and robot / scene collision setup."""

    def __init__(
        self,
        *,
        world: Any,
        robot_prim_path: str,
        config: Mapping[str, Any],
        render: bool = True,
    ) -> None:
        self._world = world
        self.robot_prim_path = str(robot_prim_path)
        self.config = dict(config)
        self.render = bool(render)
        self.enabled = True

    @property
    def block_motion(self) -> bool:
        return bool(self.config.get("block_motion", True))

    @classmethod
    def attach(
        cls,
        stage_wrapper: Any,
        *,
        robot_prim_path: str,
        config: Mapping[str, Any],
        render: bool = True,
    ) -> Optional["IsaacPhysics"]:
        """Create PhysicsScene + robot collider; return ``None`` when disabled."""
        if not config.get("enabled", True):
            return None
        try:
            from isaacsim.core.api.world import World
        except ImportError as exc:
            _LOG.warning("isaacsim.core.api.world unavailable: %s", exc)
            return None

        stage = getattr(stage_wrapper, "stage", None)
        if stage is None:
            return None

        if config.get("ensure_scene_colliders", True):
            ensure_scene_colliders(stage, skip_prefix=robot_prim_path)
        if config.get("freeze_scene", True):
            freeze_scene_rigid_bodies(stage, skip_prefix=robot_prim_path)
        if config.get("add_ground", False):
            add_ground_plane(stage)

        attach_robot_capsule(
            stage,
            robot_prim_path,
            radius=float(config.get("robot_radius", 0.12)),
            height=float(config.get("robot_height", 0.28)),
        )

        dt = float(config.get("dt", 1.0 / 60.0))
        try:
            world = World(
                physics_dt=dt,
                rendering_dt=dt,
                stage_units_in_meters=1.0,
                physics_prim_path="/physicsScene",
                set_defaults=False,
            )
        except Exception as exc:  # noqa: BLE001 — Kit already has a World
            _LOG.warning("World() failed (%s); retrying via instance()", exc)
            try:
                world = World.instance()
            except Exception as exc2:  # noqa: BLE001
                _LOG.warning("physics World unavailable: %s", exc2)
                return None
        try:
            world.reset()
        except Exception as exc:  # noqa: BLE001
            _LOG.warning("world.reset() failed: %s", exc)

        _LOG.info(
            "Isaac PhysX enabled (dt=%.4f, robot_r=%.3f, block_motion=%s)",
            dt,
            float(config.get("robot_radius", 0.12)),
            bool(config.get("block_motion", True)),
        )
        return cls(
            world=world,
            robot_prim_path=robot_prim_path,
            config=config,
            render=render,
        )

    def step(self) -> None:
        """Advance one PhysX + (optional) render frame."""
        world = self._world
        if world is None:
            return
        try:
            world.step(render=self.render)
        except Exception as exc:  # noqa: BLE001
            _LOG.debug("world.step failed: %s", exc)
            try:
                from autosim.backends.isaac.app import update_app

                update_app()
            except Exception:
                pass

    def overlaps(self, x: float, y: float, *, z: float = 0.15) -> bool:
        """Return True when a robot-sized sphere at ``(x,y,z)`` hits geometry.

        Uses PhysX scene query; returns False when the query interface is not
        ready (e.g. before the first play).
        """
        radius = float(self.config.get("robot_radius", 0.12)) * 0.95
        return overlap_sphere((float(x), float(y), float(z)), radius)

    def close(self) -> None:
        world = self._world
        self._world = None
        if world is None:
            return
        try:
            stop = getattr(world, "stop", None)
            if callable(stop):
                stop()
        except Exception:
            pass


def count_collision_apis(stage: Any) -> int:
    """Count prims with ``UsdPhysics.CollisionAPI``."""
    from pxr import UsdPhysics

    n = 0
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            n += 1
    return n


def freeze_scene_rigid_bodies(stage: Any, *, skip_prefix: str) -> int:
    """Force non-robot rigid bodies to kinematic so furniture does not fall."""
    from pxr import UsdPhysics

    skip = str(skip_prefix).rstrip("/")
    fixed = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path == skip or path.startswith(skip + "/"):
            continue
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        api = UsdPhysics.RigidBodyAPI(prim)
        try:
            attr = api.GetKinematicEnabledAttr()
            if not attr:
                attr = api.CreateKinematicEnabledAttr(True)
            else:
                attr.Set(True)
            fixed += 1
        except Exception:
            continue
    if fixed:
        _LOG.info("froze %d scene rigid bodies as kinematic", fixed)
    return fixed


def ensure_scene_colliders(stage: Any, *, skip_prefix: str) -> int:
    """If the stage has no colliders, mark meshes static (e.g. simple_room)."""
    if count_collision_apis(stage) > 0:
        return 0
    try:
        from omni.physx.scripts import utils as physx_utils
    except ImportError:
        return _ensure_colliders_usdphysics(stage, skip_prefix=skip_prefix)

    from pxr import UsdGeom

    skip = str(skip_prefix).rstrip("/")
    added = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path == skip or path.startswith(skip + "/"):
            continue
        if not prim.IsA(UsdGeom.Mesh):
            continue
        try:
            physx_utils.setStaticCollider(prim, "convexHull")
            added += 1
        except Exception:
            continue
    if added:
        _LOG.info("added static colliders on %d meshes", added)
    return added


def _ensure_colliders_usdphysics(stage: Any, *, skip_prefix: str) -> int:
    from pxr import UsdGeom, UsdPhysics

    skip = str(skip_prefix).rstrip("/")
    added = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path == skip or path.startswith(skip + "/"):
            continue
        if not prim.IsA(UsdGeom.Mesh):
            continue
        try:
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
            if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                mesh_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
                mesh_api.CreateApproximationAttr("convexHull")
            added += 1
        except Exception:
            continue
    return added


def add_ground_plane(stage: Any, *, path: str = "/World/AutosimGround") -> None:
    """Add a thin static ground box when the scene has no floor collider."""
    from pxr import Gf, UsdGeom, UsdPhysics

    if stage.GetPrimAtPath(path).IsValid():
        return
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, -0.05))
    xform.AddScaleOp().Set(Gf.Vec3f(200.0, 200.0, 0.1))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    _LOG.info("added ground collision plane at %s", path)


def attach_robot_capsule(
    stage: Any,
    robot_prim_path: str,
    *,
    radius: float,
    height: float,
) -> str:
    """Create a Z-up capsule collider under the robot and mark it kinematic."""
    from pxr import Gf, UsdGeom, UsdPhysics

    robot = stage.GetPrimAtPath(robot_prim_path)
    if not robot or not robot.IsValid():
        raise RuntimeError(f"robot prim missing: {robot_prim_path}")

    capsule_path = f"{robot_prim_path.rstrip('/')}/AutosimCollision"
    if not stage.GetPrimAtPath(capsule_path).IsValid():
        capsule = UsdGeom.Capsule.Define(stage, capsule_path)
        capsule.CreateRadiusAttr(float(radius))
        capsule.CreateHeightAttr(float(max(height, 2.0 * radius)))
        capsule.CreateAxisAttr("Z")
        xform = UsdGeom.Xformable(capsule.GetPrim())
        xform.ClearXformOpOrder()
        # Center the capsule above the footprint so it clears the floor slightly.
        z = 0.5 * float(max(height, 2.0 * radius)) + 0.02
        xform.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, float(z)))
        # Invisible guide geometry — collision only.
        try:
            UsdGeom.Imageable(capsule.GetPrim()).MakeInvisible()
        except Exception:
            pass

    # Rigid body on robot root (kinematic: cmd_vel / set_pose still drives pose).
    if not robot.HasAPI(UsdPhysics.RigidBodyAPI):
        rb = UsdPhysics.RigidBodyAPI.Apply(robot)
        rb.CreateRigidBodyEnabledAttr(True)
        rb.CreateKinematicEnabledAttr(True)
    else:
        rb = UsdPhysics.RigidBodyAPI(robot)
        rb.GetKinematicEnabledAttr().Set(True)
        rb.CreateRigidBodyEnabledAttr(True)

    capsule_prim = stage.GetPrimAtPath(capsule_path)
    if capsule_prim and capsule_prim.IsValid() and not capsule_prim.HasAPI(
        UsdPhysics.CollisionAPI
    ):
        UsdPhysics.CollisionAPI.Apply(capsule_prim)

    _LOG.info(
        "robot kinematic capsule at %s (r=%.3f h=%.3f)",
        capsule_path,
        float(radius),
        float(height),
    )
    return capsule_path


def overlap_sphere(position: tuple[float, float, float], radius: float) -> bool:
    """PhysX sphere overlap; False when PhysX is not playing yet."""
    try:
        import omni.physx
    except ImportError:
        return False
    try:
        query = omni.physx.get_physx_scene_query_interface()
    except Exception:
        return False

    hit = {"ok": False}

    def _report(_hit: Any) -> bool:
        hit["ok"] = True
        return False  # stop after first contact

    try:
        # Newer PhysX bindings: overlap_sphere(radius, pos, callback)
        query.overlap_sphere(
            float(radius),
            (float(position[0]), float(position[1]), float(position[2])),
            _report,
        )
    except TypeError:
        try:
            query.overlap_sphere(
                float(radius),
                (float(position[0]), float(position[1]), float(position[2])),
                _report,
                None,
            )
        except Exception:
            return False
    except Exception:
        return False
    return bool(hit["ok"])
