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

logger = logging.getLogger(__name__)

PHYSICS_DEFAULTS = {
    "enabled": True,
    "block_motion": True,
    "dt": 1.0 / 60.0,
    "robot_radius": 0.12,
    "robot_height": 0.28,
    "freeze_scene": True,
    "add_ground": False,
    "ensure_scene_colliders": True,
    "strip_nested_rigid_bodies": True,
    "physics_scene_path": "/physicsScene",
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
    out = dict(PHYSICS_DEFAULTS)
    out.update(block)
    out["enabled"] = bool(out.get("enabled", True))
    out["block_motion"] = bool(out.get("block_motion", True))
    out["freeze_scene"] = bool(out.get("freeze_scene", True))
    out["add_ground"] = bool(out.get("add_ground", False))
    out["ensure_scene_colliders"] = bool(out.get("ensure_scene_colliders", True))
    out["strip_nested_rigid_bodies"] = bool(
        out.get("strip_nested_rigid_bodies", True)
    )
    out["dt"] = float(out.get("dt", 1.0 / 60.0))
    out["robot_radius"] = float(out.get("robot_radius", 0.12))
    out["robot_height"] = float(out.get("robot_height", 0.28))
    out["physics_scene_path"] = str(
        out.get("physics_scene_path") or "/physicsScene"
    )
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
        self.physics_world = world
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
        """Create PhysicsScene + robot collider.

        When ``enabled: false`` but ``render`` is True (RGB-D on), still builds a
        minimal ``World`` so Replicator Camera frames update. Without it,
        ``update_app()`` alone yields empty RGB/depth (Autoviz "no data").
        """
        collision = bool(config.get("enabled", True))
        if not collision and not render:
            return None
        try:
            from isaacsim.core.api.world import World
        except ImportError as exc:
            logger.warning("isaacsim.core.api.world unavailable: %s", exc)
            return None

        stage = getattr(stage_wrapper, "stage", None)
        if stage is None:
            return None

        physics_prim_path = str(
            config.get("physics_scene_path") or "/physicsScene"
        )
        ensure_physics_scene(stage, path=physics_prim_path)

        # Let payload references finish composing before we mutate physics APIs.
        for _ in range(3):
            try:
                from autosim.backends.isaac.app import update_app

                update_app()
            except Exception:
                break

        if collision:
            if config.get("strip_nested_rigid_bodies", True):
                strip_nested_rigid_bodies(stage, skip_prefix=robot_prim_path)
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

            # Re-strip once more immediately before World so async payload
            # RigidBodyAPIs cannot sneak past the first pass.
            if config.get("strip_nested_rigid_bodies", True):
                strip_nested_rigid_bodies(stage, skip_prefix=robot_prim_path)

        dt = float(config.get("dt", 1.0 / 60.0))
        try:
            world = World(
                physics_dt=dt,
                rendering_dt=dt,
                stage_units_in_meters=1.0,
                physics_prim_path=physics_prim_path,
                set_defaults=False,
            )
        except Exception as exc:  # noqa: BLE001 — Kit already has a World
            logger.warning("World() failed (%s); retrying via instance()", exc)
            try:
                world = World.instance()
            except Exception as exc2:  # noqa: BLE001
                logger.warning("physics World unavailable: %s", exc2)
                return None
        try:
            world.reset()
        except Exception as exc:  # noqa: BLE001
            logger.warning("world.reset() failed: %s", exc)

        out_cfg = dict(config)
        if not collision:
            # Free teleop: no capsule / scene queries; World exists for RTX only.
            out_cfg["block_motion"] = False
            logger.info(
                "Isaac render-only World (physics.enabled=false, RGB-D on, dt=%.4f)",
                dt,
            )
        else:
            logger.info(
                "Isaac PhysX enabled (dt=%.4f, robot_r=%.3f, block_motion=%s)",
                dt,
                float(config.get("robot_radius", 0.12)),
                bool(config.get("block_motion", True)),
            )
        return cls(
            world=world,
            robot_prim_path=robot_prim_path,
            config=out_cfg,
            render=render,
        )

    def step(self, *, render: bool = False) -> None:
        """Advance one PhysX step.

        Default ``render=False``: the 50Hz control loop must not pay for a full
        RTX frame every tick (that drops RGB well below 5Hz on InteriorAgent).
        Call ``render_frame()`` only when sampling the camera.
        """
        world = self.physics_world
        if world is None:
            return
        try:
            world.step(render=bool(render))
        except Exception as exc:  # noqa: BLE001
            logger.debug("world.step failed: %s", exc)
            try:
                from autosim.backends.isaac.app import update_app

                update_app()
            except Exception:
                pass

    def render_frame(self) -> None:
        """One RTX / Kit frame for RGB-D (no extra PhysX substep when possible)."""
        world = self.physics_world
        if world is None:
            return
        try:
            render = getattr(world, "render", None)
            if callable(render):
                render()
                return
        except Exception as exc:  # noqa: BLE001
            logger.debug("world.render failed: %s", exc)
        self.step(render=True)

    def overlaps(self, x: float, y: float, *, z: float = 0.15) -> bool:
        """Return True when a robot-sized sphere at ``(x,y,z)`` hits geometry.

        Uses PhysX scene query; returns False when the query interface is not
        ready (e.g. before the first play).
        """
        radius = float(self.config.get("robot_radius", 0.12)) * 0.95
        return overlap_sphere((float(x), float(y), float(z)), radius)

    def close(self) -> None:
        world = self.physics_world
        self.physics_world = None
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


def ensure_physics_scene(stage: Any, *, path: str = "/physicsScene") -> str:
    """Define a ``PhysicsScene`` prim so SimulationManager can create a view."""
    from pxr import UsdPhysics

    target = str(path or "/physicsScene")
    prim = stage.GetPrimAtPath(target)
    if prim and prim.IsValid() and prim.GetTypeName() == "PhysicsScene":
        return target
    UsdPhysics.Scene.Define(stage, target)
    logger.info("created PhysicsScene at %s", target)
    return target


def strip_nested_rigid_bodies(stage: Any, *, skip_prefix: str) -> int:
    """Remove nested ``RigidBodyAPI`` under another rigid body.

    InteriorAgent door/window/cabinet USD nests RigidBodyAPI on both the
    asset root and ``group_0000``. PhysX schema then errors about missing
    xformStack reset — and it keys off API *presence*, so setting
    ``rigidBodyEnabled=False`` is not enough. Drop the child API; keep the
    outermost body (later frozen kinematic). Mesh CollisionAPI stays.
    """
    from pxr import UsdPhysics

    skip = str(skip_prefix).rstrip("/")
    removed = 0
    # Collect first: mutating during Traverse can skip siblings.
    nested_paths: list[str] = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path == skip or path.startswith(skip + "/"):
            continue
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        parent = prim.GetParent()
        while parent and parent.IsValid() and str(parent.GetPath()) not in ("/", ""):
            if parent.HasAPI(UsdPhysics.RigidBodyAPI):
                nested_paths.append(path)
                break
            parent = parent.GetParent()
    for path in nested_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        try:
            if not prim.RemoveAPI(UsdPhysics.RigidBodyAPI):
                # Fallback for stubborn referenced opinions.
                api = UsdPhysics.RigidBodyAPI(prim)
                attr = api.GetRigidBodyEnabledAttr()
                if not attr:
                    api.CreateRigidBodyEnabledAttr(False)
                else:
                    attr.Set(False)
                continue
            removed += 1
        except Exception:
            continue
    if removed:
        logger.info("removed %d nested scene RigidBodyAPI(s)", removed)
    return removed


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
        logger.info("froze %d scene rigid bodies as kinematic", fixed)
    return fixed


def ensure_scene_colliders(stage: Any, *, skip_prefix: str) -> int:
    """If the stage has no colliders, mark meshes static (e.g. simple_room)."""
    if count_collision_apis(stage) > 0:
        return 0
    try:
        from omni.physx.scripts import utils as physx_utils
    except ImportError:
        return ensure_colliders_usdphysics(stage, skip_prefix=skip_prefix)

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
        logger.info("added static colliders on %d meshes", added)
    return added


def ensure_colliders_usdphysics(stage: Any, *, skip_prefix: str) -> int:
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
    logger.info("added ground collision plane at %s", path)


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

    # Single RigidBody on the robot root (kinematic). Collision lives on the
    # child capsule — never put a second RigidBodyAPI under the root or PhysX
    # errors with "missing xformstack reset" and RGB mount becomes unreliable.
    capsule_prim = stage.GetPrimAtPath(capsule_path)
    if capsule_prim and capsule_prim.IsValid():
        try:
            if capsule_prim.HasAPI(UsdPhysics.RigidBodyAPI):
                capsule_prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
        except Exception:
            pass
        if not capsule_prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(capsule_prim)

    if not robot.HasAPI(UsdPhysics.RigidBodyAPI):
        rb = UsdPhysics.RigidBodyAPI.Apply(robot)
        rb.CreateRigidBodyEnabledAttr(True)
        rb.CreateKinematicEnabledAttr(True)
    else:
        rb = UsdPhysics.RigidBodyAPI(robot)
        rb.CreateRigidBodyEnabledAttr(True)
        rb.GetKinematicEnabledAttr().Set(True)

    logger.info(
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

    overlap = {"ok": False}

    def report_overlap(contact: Any) -> bool:
        del contact
        overlap["ok"] = True
        return False  # stop after first contact

    try:
        # Newer PhysX bindings: overlap_sphere(radius, pos, callback)
        query.overlap_sphere(
            float(radius),
            (float(position[0]), float(position[1]), float(position[2])),
            report_overlap,
        )
    except TypeError:
        try:
            query.overlap_sphere(
                float(radius),
                (float(position[0]), float(position[1]), float(position[2])),
                report_overlap,
                None,
            )
        except Exception:
            return False
    except Exception:
        return False
    return bool(overlap["ok"])
