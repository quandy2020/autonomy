# Copyright 2026 The Openbot Authors (duyongquan)

"""Isaac traffic fusion + policy driver unit tests (no Kit required)."""

from __future__ import annotations

import numpy as np
import pytest


def test_arena_offset_keeps_spawn_disk():
    from autosim.scene.arena import Arena

    arena = Arena(
        10.0,
        8.0,
        radius=0.3,
        spawn_radius=1.0,
        too_small="small",
        center=(5.0, 1.0),
        spawn_center=(7.0, 0.0),
    )
    rng = np.random.default_rng(0)
    pts = arena.sample(rng, 50)
    assert pts.shape == (50, 2)
    # Inset half-extents: 0.5*10 - 0.3 = 4.7 around center (5, 1).
    assert np.all(pts[:, 0] >= 5.0 - 4.7 - 1e-6)
    assert np.all(pts[:, 0] <= 5.0 + 4.7 + 1e-6)
    dist = np.linalg.norm(pts - np.array([7.0, 0.0]), axis=1)
    assert np.all(dist >= 1.0 + 0.3 - 1e-6)


def test_traffic_from_floorplan_style_grid():
    from autosim.scene.traffic import Traffic

    # 4x4 free center, walls on border (occupancy 100).
    grid = np.full((20, 20), 100, dtype=np.int8)
    grid[2:18, 2:18] = 0
    packed = (grid, 0.5, 0.0, 0.0, 20, 20)
    traffic = Traffic.from_block(
        {
            "x_length": 10.0,
            "y_length": 10.0,
            "center_x": 5.0,
            "center_y": 5.0,
            "spawn_x": 5.0,
            "spawn_y": 5.0,
            "spawn_radius": 0.8,
            "obstacle": {
                "enabled": True,
                "count": 3,
                "radius": 0.3,
                "height": 1.0,
                "speed_min": 0.1,
                "speed_max": 0.2,
                "seed": 1,
            },
            "pedestrian": {
                "enabled": True,
                "count": 2,
                "radius": 0.2,
                "height": 1.7,
                "speed": 0.8,
                "seed": 2,
            },
        },
        packed,
    )
    assert traffic.obstacle.position.shape[0] == 3
    assert traffic.pedestrian.position.shape[0] == 2
    traffic.step(0.05)
    cloud = traffic.cloud(0.25)
    assert cloud.shape[1] == 3 and cloud.shape[0] > 0
    origin = (5.0, 5.0, 0.5)
    dirs = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64)
    hits = traffic.nearest_batch(origin, dirs, 20.0)
    assert hits.shape == (2,)


def test_isaac_merge_nearer_and_laser_static_fallback():
    from autosim.backends.isaac.world import IsaacWorld

    class StageStub:
        def set_pose(self, *a):
            del a

        def close(self):
            pass

    class SensorsStub:
        def laser_ranges(self, *a, **k):
            del a, k
            return np.full(4, np.nan, dtype=np.float32)

        def lidar_points(self, *a):
            del a
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=np.float32)

        def close(self):
            pass

    class TrafficStub:
        def nearest_batch(self, origin, dirs, t_max):
            del origin, t_max
            out = np.full(dirs.shape[0], np.nan, dtype=np.float64)
            out[0] = 1.5
            return out

    grid = np.full((40, 40), 0, dtype=np.int8)
    grid[:, 30] = 100  # wall at x≈15 when origin 0 res 0.5
    sim = IsaacWorld(
        settings={"habitat": {"sensors": {"lidar_2d": {"range_max": 30.0}}}},
        stage=StageStub(),
        sensors=SensorsStub(),
        urdf=None,
        width=64,
        height=48,
        eye_height=0.5,
    )
    sim.scenario_grid = (grid, 0.5, 0.0, 0.0, 40, 40)
    sim.x, sim.y, sim.yaw = 5.0, 10.0, 0.0
    sim.traffic = TrafficStub()
    ranges = sim.laser_ranges(-0.1, 0.1, 4, 20.0)
    assert ranges.shape == (4,)
    assert float(ranges[0]) == pytest.approx(1.5)


def test_policy_config_and_disabled_without_model():
    from autosim.robot.policy import PolicyDriver, policy_config_from_habitat

    cfg = policy_config_from_habitat(
        {"policy": {"enabled": True, "model_path": "", "max_vx": 0.3}}
    )
    assert cfg["enabled"] is True
    assert cfg["max_vx"] == pytest.approx(0.3)
    assert PolicyDriver.maybe_create({"policy": {"enabled": False}}) is None
    # Missing file → disabled driver.
    assert (
        PolicyDriver.maybe_create(
            {"policy": {"enabled": True, "model_path": "/no/such/policy.onnx"}}
        )
        is None
    )


def test_agents_usd_character_config_and_listing(tmp_path):
    import math

    from autosim.backends.isaac.animation import (
        anim_rate,
        classify_clip_name,
        list_animation_usds,
    )
    from autosim.backends.isaac.people import (
        CHARACTER_YAW_OFFSET,
        list_character_usds,
        maybe_create_people_sync,
        people_sync_config_from_isaac,
    )

    cfg = people_sync_config_from_isaac(
        {
            "traffic": {
                "usd_sync": True,
                "obstacle_visual": "none",
                "pedestrian_visual": "character",
                "character_root": "usd/people/Characters",
                "character_yaw_offset_deg": -90,
                "collision_proxy": True,
                "sync_hz": 20,
                "pedestrian_anim": True,
                "anim_walk_mps": 1.0,
            }
        }
    )
    assert cfg["usd_sync"] is True
    assert cfg["pedestrian_visual"] == "character"
    assert cfg["character_root"] == "usd/people/Characters"
    assert abs(cfg["character_yaw_offset"] - math.radians(-90)) < 1e-9
    assert cfg["collision_proxy"] is True
    assert cfg["sync_hz"] == 20.0
    assert cfg["pedestrian_anim"] is True
    assert abs(CHARACTER_YAW_OFFSET - math.radians(-90)) < 1e-9

    defaults = people_sync_config_from_isaac({"traffic": {}})
    assert defaults["usd_sync"] is False
    assert defaults["pedestrian_visual"] == "none"
    assert maybe_create_people_sync(None, config=defaults) is None

    person = tmp_path / "female_adult_01"
    person.mkdir()
    (person / "female_adult_01.usd").write_text("#usda 1.0\n")
    (tmp_path / "Biped_Setup.usd").write_text("#usda 1.0\n")
    assets = list_character_usds(tmp_path)
    assert len(assets) == 1
    assert assets[0].endswith("female_adult_01.usd")

    anim_dir = tmp_path / "Animations"
    anim_dir.mkdir()
    (anim_dir / "Male_Walk.usd").write_text("#usda 1.0\n")
    (anim_dir / "Female_Idle.usd").write_text("#usda 1.0\n")
    clips = list_animation_usds(anim_dir)
    assert clips["walk"].endswith("Male_Walk.usd")
    assert clips["idle"].endswith("Female_Idle.usd")
    assert classify_clip_name("jog_loop") == "walk"
    assert anim_rate(1.2, walk_mps=1.2, moving=True) == pytest.approx(1.0)
    assert anim_rate(0.0, walk_mps=1.2, moving=False) == 0.0
