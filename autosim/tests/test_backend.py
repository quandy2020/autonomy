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

"""WorldBackend protocol + registry tests (no Isaac / Habitat required)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from autosim.backend import WorldBackend, create_backend
from autosim.backend.registry import BACKENDS, ensure_builtins
from autosim.config import Config
from tests.fake_simulator import FakeSimulator

ROOT = Path(__file__).resolve().parents[1]


def test_fake_simulator_is_world_backend():
    sim = FakeSimulator()
    assert isinstance(sim, WorldBackend)


def test_registry_builtins():
    ensure_builtins()
    assert set(BACKENDS) >= {"habitat", "fake", "isaac"}


def test_create_fake_backend():
    settings = Config.load(ROOT / "config" / "fake.yaml")
    world = create_backend(settings.data)
    assert isinstance(world, WorldBackend)
    world.set_pose(1.0, 2.0, 0.5)
    assert world.pose() == pytest.approx((1.0, 2.0, 0.5))
    ranges = world.laser_ranges(-3.14, 3.14, 36, 10.0)
    assert ranges.shape == (36,)
    color, depth = world.color_depth()
    assert color.ndim == 3 and depth.ndim == 2
    cloud = world.sample_depth_points()
    assert cloud.ndim == 2 and cloud.shape[1] == 3
    world.close()


def test_isaac_create_raises_without_omni():
    settings = Config.load(ROOT / "config" / "isaac.yaml")
    try:
        import omni.usd  # noqa: F401

        pytest.skip("Isaac Sim present; ImportError path not exercised")
    except ImportError:
        pass
    with pytest.raises(ImportError, match="Isaac"):
        create_backend(settings.data)


def test_coords_roundtrip():
    from autosim.backends.isaac.coords import map_to_world, world_to_map

    t, q = map_to_world(1.0, -2.0, 0.75, z=0.01)
    x, y, yaw = world_to_map(t, q)
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(-2.0)
    assert yaw == pytest.approx(0.75, abs=1e-6)
    assert isinstance(t, np.ndarray)


def test_isaac_snap_camera_frequency_nearest_divisor():
    """arena-isaac _snap_rate: 30→30 on 60Hz, 25→20 (nearest divisor)."""
    from autosim.backends.isaac.sensors import IsaacSensors

    assert IsaacSensors.snap_camera_frequency(30.0, default_render_hz=60) == 30
    assert IsaacSensors.snap_camera_frequency(25.0, default_render_hz=60) == 20
    assert IsaacSensors.snap_camera_frequency(15.0, default_render_hz=60) == 15
    assert IsaacSensors.safe_camera_frequency(30.0, default_render_hz=60) == 30


def test_densify_mesh_local_fills_large_triangle():
    """A 2×2 m wall triangle must yield far more than 3 vertices."""
    from autosim.backends.isaac.usd_stage import densify_mesh_local

    verts = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 0.0, 2.0]], dtype=np.float64)
    dense = densify_mesh_local(verts, counts=[3], indices=[0, 1, 2], spacing=0.25)
    assert dense.shape[0] > 20
    assert dense.shape[1] == 3


def test_isaac_sensors_sync_world_pose_follows_robot():
    """sync_world_pose pushes eye height + yaw into Camera.set_world_pose."""
    from autosim.backends.isaac.sensors import IsaacSensors

    class _FakeCam:
        def __init__(self):
            self.poses = []

        def set_world_pose(self, position=None, orientation=None, camera_axes=None):
            self.poses.append(
                (
                    np.asarray(position, dtype=np.float64).copy(),
                    np.asarray(orientation, dtype=np.float64).copy(),
                    camera_axes,
                )
            )

        def destroy(self):
            pass

    sensors = IsaacSensors(width=64, height=48, eye_height=0.5)
    cam = _FakeCam()
    sensors.attach(camera=cam)
    sensors.sync_world_pose(1.0, 2.0, 0.0, base_z=0.01)
    assert len(cam.poses) == 1
    pos, orn, axes = cam.poses[0]
    assert float(pos[0]) == pytest.approx(1.0)
    assert float(pos[1]) == pytest.approx(2.0)
    assert float(pos[2]) == pytest.approx(0.51)
    assert float(orn[0]) == pytest.approx(1.0)  # w
    assert axes == "world"

    # URDF-style XY offset must rotate with yaw (Beni camera_link).
    beni = IsaacSensors(
        width=64, height=48, eye_offset=(0.07, 0.03, -0.016)
    )
    cam2 = _FakeCam()
    beni.attach(camera=cam2)
    beni.sync_world_pose(0.0, 0.0, 0.0, base_z=0.11)
    px, py, pz = cam2.poses[0][0]
    assert float(px) == pytest.approx(0.07)
    assert float(py) == pytest.approx(0.03)
    assert float(pz) == pytest.approx(0.094)


def test_isaac_sensors_reads_mounted_camera():
    """IsaacSensors prefers Camera.get_rgb/get_depth over placeholder frames."""
    from autosim.backends.isaac.sensors import IsaacSensors

    class _FakeCam:
        def get_rgb(self, device=None):
            del device
            img = np.zeros((48, 64, 3), dtype=np.uint8)
            img[..., 0] = 200
            return img

        def get_depth(self, device=None):
            del device
            return np.full((48, 64), 1.25, dtype=np.float32)

        def destroy(self):
            pass

    sensors = IsaacSensors(width=64, height=48, eye_height=0.5)
    sensors.attach(camera=_FakeCam())
    color, depth = sensors.color_depth()
    assert color.shape == (48, 64, 3)
    assert int(color[0, 0, 0]) == 200
    assert depth.shape == (48, 64)
    assert float(depth[0, 0]) == pytest.approx(1.25)
    sensors.close()


def test_isaac_sensors_transposes_wh_depth_and_sanitizes_far_hits():
    """Isaac annotators are (W, H, 1); far-clip outliers must become NaN."""
    from autosim.backends.isaac.sensors import IsaacSensors

    class _FakeCam:
        def get_rgb(self, device=None):
            del device
            # Documented Isaac layout: (width, height, C).
            img = np.zeros((64, 48, 3), dtype=np.uint8)
            img[0, 0, 1] = 111
            return img

        def get_depth(self, device=None):
            del device
            depth = np.full((64, 48, 1), 2.0, dtype=np.float32)
            depth[0, 0, 0] = 1.0
            depth[10, 5, 0] = 1.0e5  # far clip → HW[5, 10]
            depth[11, 5, 0] = np.inf  # → HW[5, 11]
            return depth

        def destroy(self):
            pass

    sensors = IsaacSensors(width=64, height=48, eye_height=0.5)
    sensors.attach(camera=_FakeCam())
    color, depth = sensors.color_depth()
    assert color.shape == (48, 64, 3)
    assert int(color[0, 0, 1]) == 111
    assert depth.shape == (48, 64)
    assert float(depth[0, 0]) == pytest.approx(1.0)
    assert float(depth[0, 1]) == pytest.approx(2.0)
    assert np.isnan(depth[5, 10])
    assert np.isnan(depth[5, 11])
    sensors.close()


def test_physics_config_from_habitat_defaults():
    from autosim.backends.isaac.physics import physics_config_from_habitat

    cfg = physics_config_from_habitat({"isaac": {}})
    assert cfg["enabled"] is True
    assert cfg["mode"] == "capsule"
    assert cfg["block_motion"] is True
    assert cfg["strip_nested_rigid_bodies"] is True
    assert cfg["physics_scene_path"] == "/physicsScene"
    assert cfg["robot_radius"] == pytest.approx(0.12)

    cfg2 = physics_config_from_habitat(
        {"isaac": {"physics": {"enabled": False, "robot_radius": 0.2}}}
    )
    assert cfg2["enabled"] is False
    assert cfg2["robot_radius"] == pytest.approx(0.2)


def test_physics_config_articulation_forces_collision_world():
    """Floating-base Beni needs PhysX + scene colliders, not free teleop."""
    from autosim.backends.isaac.physics import physics_config_from_habitat

    cfg = physics_config_from_habitat(
        {
            "isaac": {
                "physics": {
                    "mode": "articulation",
                    "enabled": False,
                    "block_motion": True,
                    "ensure_scene_colliders": False,
                }
            }
        }
    )
    assert cfg["mode"] == "articulation"
    assert cfg["enabled"] is True
    assert cfg["block_motion"] is False
    assert cfg["ensure_scene_colliders"] is True
    assert cfg["strip_nested_rigid_bodies"] is True

    bad = physics_config_from_habitat(
        {"isaac": {"physics": {"mode": "unknown"}}}
    )
    assert bad["mode"] == "capsule"


def test_strip_nested_rigid_bodies_disables_child():
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom, UsdPhysics

    from autosim.backends.isaac.physics import (
        ensure_physics_scene,
        strip_nested_rigid_bodies,
    )

    stage = Usd.Stage.CreateInMemory()
    ensure_physics_scene(stage)
    root = UsdGeom.Xform.Define(stage, "/Root/door")
    child = UsdGeom.Xform.Define(stage, "/Root/door/group_0000")
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(child.GetPrim())
    UsdPhysics.RigidBodyAPI(root.GetPrim()).CreateRigidBodyEnabledAttr(True)
    UsdPhysics.RigidBodyAPI(child.GetPrim()).CreateRigidBodyEnabledAttr(True)

    n = strip_nested_rigid_bodies(stage, skip_prefix="/World/Robot")
    assert n == 1
    assert child.GetPrim().HasAPI(UsdPhysics.RigidBodyAPI) is False
    assert root.GetPrim().HasAPI(UsdPhysics.RigidBodyAPI) is True


def test_isaac_simulator_set_pose_block_motion():
    """Overlapping PhysX query must keep the previous planar pose."""
    from autosim.backends.isaac.simulator import IsaacSimulator

    class StageStub:
        def __init__(self):
            self.poses = []

        def set_pose(self, x, y, yaw):
            self.poses.append((x, y, yaw))

        def close(self):
            pass

    class PhysicsStub:
        block_motion = True

        def overlaps(self, x, y, z=0.15):
            del z
            return float(x) > 5.0

        def step(self):
            pass

        def close(self):
            pass

    class SensorsStub:
        def close(self):
            pass

    sim = IsaacSimulator(
        settings={"habitat": {}},
        stage=StageStub(),
        sensors=SensorsStub(),
        urdf=None,
        width=64,
        height=48,
        eye_height=0.5,
    )
    sim.physics = PhysicsStub()
    sim.set_pose(1.0, 2.0, 0.3, force=True)
    assert sim.pose() == (1.0, 2.0, 0.3)
    sim.set_pose(9.0, 2.0, 0.0)  # blocked
    assert sim.pose() == (1.0, 2.0, 0.3)
    assert sim.stage.poses[-1] == (1.0, 2.0, 0.3)
    sim.set_pose(3.0, 1.0, 0.5)  # free
    assert sim.pose() == (3.0, 1.0, 0.5)


def test_floorplan_map_from_rooms_json():
    """InteriorAgent rooms.json → free cells inside, OccupancyGrid on /map."""
    from pathlib import Path

    from autosim.backends.isaac.floorplan_map import FloorplanMap

    rooms_path = (
        Path(__file__).resolve().parents[1]
        / "usd"
        / "worlds"
        / "InteriorAgent"
        / "kujiale_0021"
        / "rooms.json"
    )
    if not rooms_path.is_file():
        pytest.skip("kujiale_0021 rooms.json not present")
    rooms = FloorplanMap.load_rooms(rooms_path)
    assert len(rooms) >= 1
    built = FloorplanMap(
        {"grid": {"resolution": 0.1, "z_min": 0.1, "z_max": 2.5}}
    ).build_from_rooms(rooms)
    cloud, grid, res, ox, oy, width, height = built.sample()
    assert cloud.shape[1] == 3 and cloud.shape[0] > 1000  # dense walls + floor
    assert float(cloud[:, 2].min()) <= 0.15
    assert float(cloud[:, 2].max()) >= 2.0
    assert grid.dtype == np.int8
    assert width > 10 and height > 10
    assert float(res) == pytest.approx(0.1)
    assert (grid == 0).any() and (grid == 100).any()
    # Spawn from isaac.yaml living-room centroid should be free.
    sx, sy = 7.93, -0.39
    col = int((sx - ox) / res)
    row = int((sy - oy) / res)
    assert 0 <= row < height and 0 <= col < width
    assert int(grid[row, col]) == 0
    # USD mesh override path used by IsaacSimulator for /overall/map.
    fake = np.array([[1.0, 2.0, 0.5], [3.0, 4.0, 1.5]], dtype=np.float32)
    built.set_cloud(fake)
    cloud2, *_ = built.sample()
    assert cloud2.shape == (2, 3)
    assert float(cloud2[0, 0]) == pytest.approx(1.0)


def test_robot_body_band_ignores_floor_and_overhangs():
    """Ground-robot slab: clearance…body; overhang config uses robot_height_m."""
    from autosim.backends.isaac.floorplan_map import FloorplanMap

    z0, z1, inf = FloorplanMap.robot_body_band(
        {"robot_clearance_m": 0.05, "robot_height_m": 0.35, "mesh_inflate_m": 0.12}
    )
    assert z0 == pytest.approx(0.05)
    assert z1 == pytest.approx(0.40)  # 0.35 + 0.05 margin
    assert inf == pytest.approx(0.12)


def test_floorplan_free_openings_clears_door_cells():
    """Door footprints punch through rooms.json wall rings."""
    from pathlib import Path

    from autosim.backends.isaac.floorplan_map import FloorplanMap

    rooms_path = (
        Path(__file__).resolve().parents[1]
        / "usd"
        / "worlds"
        / "InteriorAgent"
        / "kujiale_0021"
        / "rooms.json"
    )
    if not rooms_path.is_file():
        pytest.skip("kujiale_0021 rooms.json not present")
    rooms = FloorplanMap.load_rooms(rooms_path)
    built = FloorplanMap({"grid": {"resolution": 0.05}}).build_from_rooms(rooms)
    _, grid0, res, ox, oy, _, _ = built.sample()
    # Known door leaf near living-room / balcony wall.
    dx, dy = 2.45, -0.43
    col = int((dx - ox) / res)
    row = int((dy - oy) / res)
    assert int(grid0[row, col]) == 100
    door = np.array(
        [[dx, dy, 0.08], [dx + 0.02, dy, 0.10], [dx, dy + 0.02, 0.12]],
        dtype=np.float32,
    )
    freed = built.free_openings(door, z_min=0.0, z_max=2.2, inflate_m=0.10)
    assert freed > 0
    _, grid1, *_ = built.sample()
    assert int(grid1[row, col]) == 0


def test_floorplan_mark_elevated_obstacles_carves_furniture():
    """Body-band hits occupied; floor + high overhang stay free."""
    from pathlib import Path

    from autosim.backends.isaac.floorplan_map import FloorplanMap

    rooms_path = (
        Path(__file__).resolve().parents[1]
        / "usd"
        / "worlds"
        / "InteriorAgent"
        / "kujiale_0021"
        / "rooms.json"
    )
    if not rooms_path.is_file():
        pytest.skip("kujiale_0021 rooms.json not present")
    rooms = FloorplanMap.load_rooms(rooms_path)
    built = FloorplanMap({"grid": {"resolution": 0.1}}).build_from_rooms(rooms)
    _, grid0, res, ox, oy, _, _ = built.sample()
    sx, sy = 7.93, -0.39
    # Body hit (sofa) + floor crumb + high tabletop overhang.
    cloud = np.array(
        [
            [sx + 1.5, sy - 1.0, 0.25],
            [sx + 1.55, sy - 1.05, 0.30],
            [sx, sy, 0.02],
            [sx - 1.0, sy, 1.20],  # overhang — must NOT occupy
        ],
        dtype=np.float32,
    )
    changed = built.mark_elevated_obstacles(
        cloud, z_min=0.05, z_max=0.40, inflate_m=0.1
    )
    assert changed > 0
    _, grid1, _, _, _, _, _ = built.sample()
    col_s = int((sx - ox) / res)
    row_s = int((sy - oy) / res)
    assert int(grid1[row_s, col_s]) == 0  # spawn + floor still free
    col_f = int((sx + 1.5 - ox) / res)
    row_f = int((sy - 1.0 - oy) / res)
    assert int(grid1[row_f, col_f]) == 100
    col_o = int((sx - 1.0 - ox) / res)
    row_o = int((sy - oy) / res)
    assert int(grid1[row_o, col_o]) == 0  # overhang ignored
    assert int(grid0[row_s, col_s]) == 0


def test_repair_display_primvars_constantizes_facevarying():
    """Length-1 faceVarying displayColor must become constant (InteriorAgent)."""
    pytest.importorskip("pxr")
    from autosim.backends.isaac.usd_stage import repair_display_primvars
    from pxr import Sdf, Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    mesh = UsdGeom.Mesh.Define(stage, "/World/Mesh")
    mesh.CreatePointsAttr([(-1, 0, 0), (1, 0, 0), (0, 1, 0)])
    mesh.CreateFaceVertexCountsAttr([3])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "displayColor",
        Sdf.ValueTypeNames.Color3fArray,
        UsdGeom.Tokens.faceVarying,
    )
    pv.Set([(0.8, 0.2, 0.1)])
    assert repair_display_primvars(stage) == 1
    assert pv.GetInterpolation() == UsdGeom.Tokens.constant
