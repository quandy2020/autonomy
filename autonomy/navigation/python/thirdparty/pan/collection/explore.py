"""Exploration tours: naokiyokoyama/frontier_exploration's agent over HM3D scenes -> HDF5 shards.

The agent reveals a fog of war, detects frontiers, picks one by A* completion time and point-turns
along the geodesic toward it. The repo at `third_party/frontier_exploration/` stays untouched: its
modules import under a stub package entry that bypasses its package `__init__.py`.

One spawned worker per GPU runs its (scene, seed) tasks; scenes are dealt to the workers
round-robin, and one simulator serves all seeds of a scene.

Usage: pan-explore --num-scenes 10 --seeds-per-scene 5 --gpu-ids 4,5,6,7
"""
from __future__ import annotations

import math
import multiprocessing as mp
import os
import sys
import time
import traceback
import types
from collections import Counter
from typing import Dict, List, Optional, Sequence

import h5py
import habitat_sim
import numpy as np
import quaternion as npq

from thirdparty.pan.planner.env import SIM_ACTIONS, agent_matrix, camera_matrix, find_path, make_sim
from thirdparty.pan.util.constants import (FORWARD_STEP_M, HABITAT_DATA_ROOT, HM3D_SCENE_DATASET_CONFIG, REPO_ROOT,
                                   SENSOR_HEIGHT, SENSOR_HFOV_DEG, SENSOR_RES, TURN_STEP_DEG, Action)
from thirdparty.pan.util.geometry import heading_from_quaternion, raster_cell

# a stub package entry skips frontier_exploration/__init__.py (habitat-lab and gym imports)
_FE = sys.modules["thirdparty.frontier_exploration"] = types.ModuleType("thirdparty.frontier_exploration")
_FE.__path__ = [str(REPO_ROOT / "thirdparty/frontier_exploration")]
from thirdparty.frontier_exploration.frontier_detection import detect_frontier_waypoints  # noqa: E402
from thirdparty.frontier_exploration.utils.fog_of_war import reveal_fog_of_war  # noqa: E402
from thirdparty.frontier_exploration.utils.general_utils import wrap_heading  # noqa: E402
from thirdparty.frontier_exploration.utils.path_utils import (a_star_search, completion_time_heuristic,  # noqa: E402
                                                   heading_error, path_time_cost)

SCENE_TOPDOWN_MPP = 0.05                # metres per cell of the fog, scene_topdown and coverage rasters
# A* costs count actions, each FORWARD_STEP_M or TURN_RAD; a heading error beyond TURN_RAD turns the agent.
TURN_RAD = math.radians(TURN_STEP_DEG)
MIN_FRONTIER_CLUSTER_CELLS = 20         # smallest unexplored region worth a trip, in 0.10 m cells
COLLECT_EXPLORE_FOV_RANGE_M = 6.0       # fog-of-war reveal range
MIN_REGION_AREA_PX = MIN_FRONTIER_CLUSTER_CELLS * (0.10 ** 2) / (SCENE_TOPDOWN_MPP ** 2)
VISIBILITY_DIST_PX = COLLECT_EXPLORE_FOV_RANGE_M / SCENE_TOPDOWN_MPP
MIN_ISLAND_AREA_M2 = 8.0                # smallest navmesh island a start may lie on
FLOOR_BAND_M = 0.3                      # half-height of the dominant floor's band
MIN_KEEP_FRAMES_EXPLORE = 60            # tours with fewer frames are dropped
MIN_KEEP_PATH_M_EXPLORE = 10.0          # metres; tours that travel less are dropped
MAX_STEPS = 3000                        # loop iterations, blacklist passes included
NEW_TARGET_EPS_M = 0.3                  # a target this far from the last one logs a new goal and leg
# A pursuit that for STUCK_STEPS iterations neither moves STUCK_DIST_M nor reveals a fog cell
# blacklists its target; frontier waypoints within BLACKLIST_RADIUS_M of it are dropped.
STUCK_STEPS = 2 * math.ceil(360 / TURN_STEP_DEG)    # two full point-turns
STUCK_DIST_M = 2 * FORWARD_STEP_M       # above turning in place and collision sliding
BLACKLIST_RADIUS_M = 2 * FORWARD_STEP_M     # absorbs step-to-step frontier-waypoint jitter


# ---------------------------------------------------------------- floors
def find_dominant_floor(pathfinder, n_samples: int = 500, min_density: float = 0.04) -> Optional[float]:
    """Height of the densest band of navigable points; None with fewer than 16 valid samples or
    when no band holds `min_density` of them."""
    ys = []
    for _ in range(n_samples * 2):                 # over-sample to absorb invalid returns
        p = pathfinder.get_random_navigable_point()
        if p is None or not np.isfinite(p).all():
            continue
        ys.append(p[1])
        if len(ys) >= n_samples:
            break
    if len(ys) < 16:
        return None
    ys = np.asarray(ys, dtype=np.float64)
    lo, hi = ys.min() - 1e-3, ys.max() + 1e-3
    hist, edges = np.histogram(ys, bins=max(8, int(np.ceil((hi - lo) / (FLOOR_BAND_M / 2)))),
                               range=(lo, hi))
    peak = int(hist.argmax())
    if hist[peak] / len(ys) < min_density:
        return None
    return float(0.5 * (edges[peak] + edges[peak + 1]))


def _sample_explore_start(pathfinder, floor_y: float, max_tries: int = 50) -> Optional[np.ndarray]:
    """A navigable point within FLOOR_BAND_M of `floor_y` on an island of at least
    MIN_ISLAND_AREA_M2."""
    for _ in range(max_tries):
        for _ in range(200):
            p = pathfinder.get_random_navigable_point()
            if (p is not None and np.isfinite(p).all()
                    and floor_y - FLOOR_BAND_M <= float(p[1]) <= floor_y + FLOOR_BAND_M):
                candidate = np.asarray(p, dtype=np.float64)
                break
        else:
            continue
        if pathfinder.island_area(pathfinder.get_island(candidate)) >= MIN_ISLAND_AREA_M2:
            return candidate
    return None


# ---------------------------------------------------------------- shards
def record_frame(frames: Dict[str, list], sim, agent) -> habitat_sim.AgentState:
    """Append the rendered rgb and depth and the agent-to-world matrix to `frames` -> the agent's
    state, read after rendering."""
    obs = sim.get_sensor_observations()
    state = agent.get_state()
    frames["rgb"].append(obs["rgb"][..., :3].copy())             # (H, W, 3) uint8 of habitat's RGBA
    frames["depth"].append(obs["depth"].astype(np.float16))      # (H, W) metres
    frames["agent_to_world"].append(agent_matrix(state, np.float32))
    return state


def write_shard(output_h5: str, pathfinder, frames: Dict[str, list], floor_y: float,
                goals: Sequence[np.ndarray], geodesic_path: np.ndarray, attrs: dict,
                coverage: np.ndarray) -> dict:
    """Write one tour shard -> its attributes (`attrs` plus the frame count, start-to-last-goal
    distance, floor, raster and sensor ones) and `output_h5`.

        rgb                (N, H, W, 3) u8                                  gzip
        scene_topdown      (R, C) u8 navigable at floor_y                   gzip
        coverage_topdown   (R, C) u8 `coverage`                             gzip
        depth              (N, H, W) f16 metres
        pose               (N, 4, 4) f32 agent-to-world
        camera_pose_world  (N, 4, 4) f32 camera-to-world
        waypoints          (G, 3) f32 `goals`: the start point, then each new frontier target
        geodesic_path      (M, 3) f32 the concatenated geodesic legs toward those targets
    """
    agent_to_world = np.stack(frames["agent_to_world"], 0)
    lo, _ = pathfinder.get_bounds()
    attrs = dict(attrs, n_frames=len(agent_to_world),
                 euclidean_distance=float(np.linalg.norm(goals[-1] - goals[0])),
                 floor_y=floor_y, floor_y_lo=floor_y - FLOOR_BAND_M, floor_y_hi=floor_y + FLOOR_BAND_M,
                 scene_topdown_mpp=SCENE_TOPDOWN_MPP, scene_xmin=float(lo[0]), scene_zmin=float(lo[2]),
                 sensor_height=SENSOR_HEIGHT, hfov_deg=SENSOR_HFOV_DEG, image_h=SENSOR_RES[0],
                 image_w=SENSOR_RES[1])
    compressed = {"rgb": np.stack(frames["rgb"], 0),
                  "scene_topdown": np.asarray(pathfinder.get_topdown_view(SCENE_TOPDOWN_MPP, floor_y),
                                              dtype=np.uint8),
                  "coverage_topdown": coverage}
    uncompressed = {"depth": np.stack(frames["depth"], 0), "pose": agent_to_world,
                    "camera_pose_world": camera_matrix(agent_to_world),
                    "waypoints": np.stack(goals, 0).astype(np.float32), "geodesic_path": geodesic_path}
    os.makedirs(os.path.dirname(output_h5), exist_ok=True)
    with h5py.File(output_h5, "w") as f:
        for name, data in compressed.items():
            f.create_dataset(name, data=data, compression="gzip", compression_opts=4)
        for name, data in uncompressed.items():
            f.create_dataset(name, data=data)
        f.attrs.update(attrs)
    return dict(attrs, output_h5=output_h5)


# ---------------------------------------------------------------- exploration
def explore_one(sim: habitat_sim.Simulator, scene_glb: str, seed: int, output_h5: str) -> Optional[dict]:
    """Explore the scene loaded in `sim` from a random start and write its shard -> its attributes;
    None when the scene has no dominant floor or start, or the tour is filtered."""
    rng = np.random.default_rng(seed)
    pathfinder = sim.pathfinder
    pathfinder.seed(int(seed))
    floor_y = find_dominant_floor(pathfinder)
    if floor_y is None:
        return None
    start = _sample_explore_start(pathfinder, floor_y)
    if start is None:
        return None

    start_island = pathfinder.get_island(start)
    island_grid = np.asarray(pathfinder.get_topdown_island_view(SCENE_TOPDOWN_MPP, floor_y), dtype=np.int64)
    top_down = (island_grid == start_island).astype(np.uint8)
    lo, _ = pathfinder.get_bounds()
    x_min, z_min = float(lo[0]), float(lo[2])

    agent = sim.get_agent(0)
    state = agent.get_state()
    state.position = start
    start_yaw = float(rng.uniform(0, 2 * np.pi))
    state.rotation = npq.from_rotation_vector([0.0, start_yaw, 0.0])
    agent.set_state(state)

    frames = {"rgb": [], "depth": [], "agent_to_world": []}
    start_point = np.asarray(agent.get_state().position, dtype=np.float64)
    goals, geodesic_legs = [start_point], []
    path_length, last_target = 0.0, None
    fog = np.zeros_like(top_down)
    first_frontier_found = False
    default_turn = Action.TURN_LEFT if bool(rng.integers(0, 2)) else Action.TURN_RIGHT
    reason = "max_steps"
    blacklist: list = []
    stall_position, stall_fog, stall_steps = start_point[[0, 2]], 0, 0

    previous, acted = start_point, True
    for _ in range(MAX_STEPS):
        # one frame per pose: none after a blacklist pass
        state = record_frame(frames, sim, agent) if acted else agent.get_state()
        acted = False
        point = np.asarray(state.position, dtype=np.float64)
        position = point[[0, 2]]
        path_length += float(np.hypot(point[0] - previous[0], point[2] - previous[2]))
        previous = point
        # frontier_exploration's heading convention
        fe_heading = float(wrap_heading(heading_from_quaternion(state.rotation) + np.pi))

        agent_cell = np.array(raster_cell(position, (x_min, z_min), SCENE_TOPDOWN_MPP))
        fog = reveal_fog_of_war(top_down, fog, agent_cell, fe_heading, fov=SENSOR_HFOV_DEG,
                                max_line_len=VISIBILITY_DIST_PX)
        fog_cells = int(fog.sum())
        if float(np.hypot(*(position - stall_position))) > STUCK_DIST_M or fog_cells > stall_fog:
            stall_position, stall_fog, stall_steps = position, fog_cells, 0
        else:
            stall_steps += 1
        # (n, 2) (col, row) pixel coordinates; also zeroes the fog off the island, in place
        px = detect_frontier_waypoints(top_down, fog, area_thresh=MIN_REGION_AREA_PX).reshape(-1, 2)
        waypoints = np.stack([(px[:, 0] + 0.5) * SCENE_TOPDOWN_MPP + x_min, np.full(len(px), floor_y),
                              (px[:, 1] + 0.5) * SCENE_TOPDOWN_MPP + z_min], 1)
        if blacklist and len(waypoints):
            d2 = ((waypoints[:, None, [0, 2]] - np.asarray(blacklist)[None]) ** 2).sum(-1)
            keep = d2.min(axis=1) > BLACKLIST_RADIUS_M ** 2
            if not keep.any():
                reason = "frontiers_blacklisted"
                break
            waypoints = waypoints[keep]

        if len(waypoints) == 0:
            if first_frontier_found:
                reason = "frontier_exhausted"
                break
            action = default_turn
        else:
            first_frontier_found = True
            idx, min_cost = a_star_search(
                waypoints,
                lambda x: completion_time_heuristic(x, point, fe_heading, FORWARD_STEP_M, TURN_RAD),
                lambda x: path_time_cost(x, point, fe_heading, FORWARD_STEP_M, TURN_RAD, sim))
            if idx is None or not np.isfinite(min_cost):
                reason = "no_reachable_frontier"
                break
            target = waypoints[idx]

            if stall_steps >= STUCK_STEPS:
                blacklist.append(target[[0, 2]])
                # recounted: detection may have zeroed fog cells since fog_cells
                stall_position, stall_fog, stall_steps = position, int(fog.sum()), 0
                continue

            path, _ = find_path(pathfinder, point, target)
            if path is None or len(path) < 2:
                reason = "no_path_to_target"
                break
            if last_target is None or float(np.linalg.norm(target - last_target)) > NEW_TARGET_EPS_M:
                goals.append(target)
                geodesic_legs.append(path.astype(np.float32))
                last_target = target

            error = heading_error(point, path[1], fe_heading)
            if error > TURN_RAD:
                action = Action.TURN_RIGHT
            elif error < -TURN_RAD:
                action = Action.TURN_LEFT
            else:
                action = Action.FORWARD

        agent.act(SIM_ACTIONS[action])
        acted = True

    if len(frames["rgb"]) < MIN_KEEP_FRAMES_EXPLORE or path_length < MIN_KEEP_PATH_M_EXPLORE:
        return None
    seen_frac = float((fog & top_down).sum()) / max(1, int(top_down.sum()))
    coverage = top_down + (top_down & (fog > 0))            # 0 off-island, 1 unseen, 2 observed
    geodesic_path = np.concatenate(geodesic_legs) if geodesic_legs else np.zeros((0, 3), np.float32)
    attrs = dict(strategy="exploration", scene_id=scene_id_of(scene_glb), scene_glb=scene_glb, seed=seed,
                 geodesic_distance=path_length,         # metres travelled
                 agent_start_pos=start_point.astype(np.float32), agent_start_yaw=start_yaw,
                 coverage_frac=seen_frac, n_frontier_goals=len(goals) - 1, termination_reason=reason,
                 reachable_island_area_m2=float(pathfinder.island_area(start_island)),
                 n_blacklisted=len(blacklist))
    return write_shard(output_h5, pathfinder, frames, floor_y, goals, geodesic_path, attrs, coverage)


# ---------------------------------------------------------------- parallel driver
_WORKER_DONE = "__worker_done__"     # a worker's last message on the results queue


def scene_glb_path(scene_root: str, scene_id: str) -> str:
    """HM3D layout: <scene_root>/<id>/<short_id>.basis.glb, short_id = the id after its first dash."""
    short = scene_id.split("-", 1)[-1]
    return os.path.join(scene_root, scene_id, f"{short}.basis.glb")


def scene_id_of(scene_glb: str) -> str:
    """The scene id of a `scene_glb_path`: its directory name."""
    return os.path.basename(os.path.dirname(scene_glb))


def count_statuses(results: List[dict]) -> Dict[str, int]:
    return dict(Counter(r["status"] for r in results))


def _worker_main(gpu_id: int, tasks: List, scene_dataset_config: str, out_root: str, results_q) -> None:
    """Run one worker's tasks in a spawned child, one simulator per scene."""
    current_scene, sim = None, None
    try:
        for scene_glb, seed in tasks:
            out_h5 = os.path.join(out_root, scene_id_of(scene_glb), f"seed{seed:05d}.h5")
            if os.path.exists(out_h5):
                results_q.put({"scene_glb": scene_glb, "seed": seed, "output_h5": out_h5,
                               "status": "skipped_existing"})
                continue
            if scene_glb != current_scene:
                if sim is not None:
                    sim.close()
                sim = make_sim(scene_glb, scene_dataset_config, gpu_device_id=gpu_id)
                current_scene = scene_glb
            t0 = time.time()
            try:
                summary = explore_one(sim, scene_glb, seed, out_h5)
            except Exception as exc:
                results_q.put({"scene_glb": scene_glb, "seed": seed, "status": "error",
                               "error": f"{type(exc).__name__}: {exc}",
                               "traceback": traceback.format_exc()})
                continue
            if summary is None:
                summary = {"scene_glb": scene_glb, "seed": seed, "status": "filtered"}
            results_q.put({"status": "ok", **summary, "elapsed_s": time.time() - t0})
    finally:
        if sim is not None:
            sim.close()
        results_q.put((_WORKER_DONE, gpu_id))


def run_parallel(scene_glbs: Sequence[str], scene_dataset_config: str, seeds_per_scene: int,
                 out_root: str, gpu_ids: Sequence[int], seed_offset: int = 0) -> List[dict]:
    """Explore every (scene, seed) with one worker per gpu_id -> the workers' result dicts; scene i
    goes to worker i % len(gpu_ids)."""
    os.makedirs(out_root, exist_ok=True)
    worker_tasks: Dict[int, List] = {gid: [] for gid in gpu_ids}
    for i, glb in enumerate(scene_glbs):
        worker_tasks[gpu_ids[i % len(gpu_ids)]].extend((glb, seed_offset + k) for k in range(seeds_per_scene))
    total = sum(len(t) for t in worker_tasks.values())
    print(f"explore: {len(scene_glbs)} scenes x {seeds_per_scene} seeds = {total} tasks "
          f"across {len(gpu_ids)} workers (gpu_ids={list(gpu_ids)})")

    ctx = mp.get_context("spawn")           # habitat-sim's CUDA/EGL state does not survive fork
    results_q = ctx.Queue()
    procs = [ctx.Process(target=_worker_main, args=(gid, tasks, scene_dataset_config, out_root, results_q))
             for gid, tasks in worker_tasks.items()]
    for p in procs:
        p.start()
    results: List[dict] = []
    done_workers, t_start = 0, time.time()
    while done_workers < len(procs):
        item = results_q.get()
        if isinstance(item, tuple) and item[0] == _WORKER_DONE:
            done_workers += 1
            print(f"worker (gpu={item[1]}) finished. ({done_workers}/{len(procs)} workers done)")
            continue
        results.append(item)
        n = len(results)
        if n % 10 == 0 or n == total:
            elapsed = time.time() - t_start
            rate = n / max(elapsed, 1e-6)
            print(f"[{n}/{total}] elapsed={elapsed:.0f}s rate={rate:.2f}/s "
                  f"eta={(total - n) / max(rate, 1e-6):.0f}s   {count_statuses(results)}")
    for p in procs:
        p.join(timeout=30)
        if p.is_alive():
            p.terminate()
    return results


# ---------------------------------------------------------------- CLI
def main():
    import argparse
    p = argparse.ArgumentParser(description="Collect exploration trajectories across HM3D scenes.")
    p.add_argument("--scene-root", default=str(HABITAT_DATA_ROOT / "raw/scene_datasets/hm3d/minival"))
    p.add_argument("--dataset-config", default=str(HM3D_SCENE_DATASET_CONFIG))
    p.add_argument("--out-root", default=str(HABITAT_DATA_ROOT / "traj/exploration_v1"))
    p.add_argument("--num-scenes", type=int, default=10)
    p.add_argument("--seeds-per-scene", type=int, default=5)
    p.add_argument("--gpu-ids", default=None, help="comma-separated CUDA device ids; default: one per "
                                                   "CUDA_VISIBLE_DEVICES entry, else 0")
    p.add_argument("--seed-offset", type=int, default=0)
    args = p.parse_args()

    dirs = [d for d in sorted(os.listdir(args.scene_root))
            if os.path.isdir(os.path.join(args.scene_root, d))]
    glbs = [g for g in (scene_glb_path(args.scene_root, d) for d in dirs) if os.path.exists(g)]
    print(f"using {min(len(glbs), args.num_scenes)} scenes (of {len(dirs)} dirs; "
          f"{len(dirs) - len(glbs)} lack a basis.glb)")
    if args.gpu_ids:
        gpu_ids = [int(x) for x in args.gpu_ids.split(",")]
    else:
        visible = os.environ.get("CUDA_VISIBLE_DEVICES", "").strip()
        gpu_ids = list(range(len(visible.split(",")))) if visible else [0]

    t0 = time.time()
    results = run_parallel(glbs[:args.num_scenes], args.dataset_config, args.seeds_per_scene,
                           args.out_root, gpu_ids, seed_offset=args.seed_offset)
    print(f"\n=== DONE in {time.time() - t0:.0f}s ===")
    print("per-status:", count_statuses(results))
    ok = [r for r in results if r.get("status") == "ok"]
    if ok:
        nf = np.array([r["n_frames"] for r in ok])
        gd = np.array([r["geodesic_distance"] for r in ok])
        cf = np.array([r["coverage_frac"] for r in ok])
        print(f"n_frames      mean={nf.mean():.1f} median={np.median(nf):.0f} min={nf.min()} max={nf.max()}")
        print(f"path_length   mean={gd.mean():.2f}m median={np.median(gd):.2f}m "
              f"min={gd.min():.2f}m max={gd.max():.2f}m")
        print(f"coverage_frac mean={cf.mean():.3f} min={cf.min():.3f} max={cf.max():.3f}")


if __name__ == "__main__":
    main()
