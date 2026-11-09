"""Tour dataset: per item, one tour's context (keyframe tokens and poses) and a set of query
states with expert action-chunk targets toward one shared goal.

Queries are states subsampled across a few expert trajectories to the item's goal; a
`state_noise` fraction is moved off its trajectory and relabelled by the expert. Episode starts
and goals lie on the tour's floor, island and observed coverage; the expert routes on the full
navmesh. `EvalBank` replays a frozen bank written by `collection.bank`.

An item is (tokens (K, P, Dt) float16, kf_pose (K, 3), query_pose (T, 3), goal_position (2,),
targets (T, chunk) int64); `collate` packs items into a `Batch`.
"""
from __future__ import annotations

import glob
import json
import os
from dataclasses import dataclass
from typing import Dict, List, NamedTuple, Optional

import h5py
import numpy as np
import torch
from torch.utils.data import Dataset

from thirdparty.pan.util.config import arg
from thirdparty.pan.util.geometry import R_ground, wrap_angle
from thirdparty.pan.util.keyframe_gate import COVERAGE

from .env import (Episode, EpisodeConfig, TourScene, detour_ratio, load_pathfinder, snap_point,
                  tour_island)
from .expert import (chunk_targets_from_state, chunk_targets_from_trajectory, sample_episode,
                     sample_goal)

EVAL_SEED = 0                           # the one context draw every evaluation uses
TOUR_GLOB = os.path.join("*", "seed*.h5")  # <scene id>/seed*.h5 (HM3D, MP3D, Gibson)
PAD_TARGET = -100                       # targets at padded queries; F.cross_entropy's ignore_index
_COVERAGE_SPEC = json.loads(COVERAGE.to_json())


def jitter_(poses: np.ndarray, position_m: float, yaw_rad: float, rng: np.random.Generator) -> None:
    """In place on (N, 3) poses: independent uniform error, +-position_m on x and on z,
    +-yaw_rad on yaw."""
    n = len(poses)
    if position_m:
        poses[:, 0] += rng.uniform(-position_m, position_m, n).astype(poses.dtype)
        poses[:, 1] += rng.uniform(-position_m, position_m, n).astype(poses.dtype)
    if yaw_rad:
        poses[:, 2] = wrap_angle(poses[:, 2] + rng.uniform(-yaw_rad, yaw_rad, n).astype(poses.dtype))


@dataclass
class DataConfig:
    token_root: str = arg("dataset/habitat/tokens/train",
                          "token stores, <root>/<scene>/seed*.h5")
    eta: int = arg(10, "coverage-gate eta of the context keyframes; 0 = every frame")
    max_kf: int = arg(768, "context keyframes beyond this are uniformly subsampled")
    queries: int = arg(64, "query states per item, subsampled across its trajectories")
    episodes_per_item: int = arg(6, "expert trajectories per item, all to the item's goal")
    state_noise: float = arg(0.5, "fraction of queries moved off the trajectory and relabelled by "
                                  "the expert")
    state_noise_pos: float = arg(0.2, "metres, Gaussian std of that move (before the navmesh snap)")
    state_noise_yaw: float = arg(0.5, "radians, uniform half-width of that heading change")
    query_noise_pos: float = arg(0.1, "metres, uniform half-width of the error on the query position "
                                      "the model reads; targets stay those of the true state")
    query_noise_yaw: float = arg(0.2, "radians, uniform half-width of the error on the query yaw the "
                                      "model reads")
    kf_noise_pos: float = arg(0.1, "metres, uniform half-width of independent jitter on each "
                                   "keyframe's stored position")
    kf_noise_yaw: float = arg(0.2, "radians, uniform half-width of independent jitter on each "
                                   "keyframe's stored yaw")
    rot_aug: bool = arg(False, "re-express each item in a frame rotated about +y by a uniform random "
                               "angle")
    # set by TrainConfig from the model and the run
    chunk: int = 8
    zero_tokens: bool = False           # tokens are not read (zero_content)
    seed: int = 0


# ---------------------------------------------------------------- token stores
def read_tour_meta(path: str) -> dict:
    """A token store's per-tour arrays and attributes, everything but the tokens."""
    with h5py.File(path, "r") as f:
        a = f.attrs
        meta = dict(pose=f["pose"][:], coverage=f["coverage_topdown"][:],
                    origin_position=(float(a["scene_xmin"]), float(a["scene_zmin"])),
                    mpp=float(a["mpp"]), floor_y=float(a["floor_y"]),
                    start_point=np.asarray(a["agent_start_pos"], np.float64),
                    scene_glb=str(a["scene_glb"]))
        if "keep" in f:
            meta.update(keep=f["keep"][:], keep_etas=tuple(int(e) for e in a.get("keep_etas", ())),
                        keep_spec=a.get("keep_spec"))
    return meta


def select_keyframes(meta: dict, eta: int, max_kf: int) -> np.ndarray:
    """Frame indices of the tour's coverage-gate keyframes at `eta` (every frame when `eta` is 0),
    uniformly subsampled to at most `max_kf`."""
    relabel = " (write it with collection.tokens --labels-only)"
    assert "keep" in meta, f"token store has no keep column{relabel}"
    assert meta["keep_spec"] is not None and json.loads(meta["keep_spec"]) == _COVERAGE_SPEC, (
        f"keep was written with gate spec {meta['keep_spec']}, not util.keyframe_gate.COVERAGE{relabel}")
    assert eta == 0 or eta in meta["keep_etas"], f"keep has no eta {eta} (has {meta['keep_etas']})"
    idx = (np.arange(len(meta["pose"])) if eta == 0
           else np.nonzero(meta["keep"][:, meta["keep_etas"].index(eta)])[0])
    if len(idx) > max_kf:
        idx = idx[np.linspace(0, len(idx) - 1, max_kf).astype(np.int64)]
    return idx


class TourStore:
    """The token stores under one root, addressed by tour (the store's path relative to the
    root). Metadata, pathfinders and islands are cached per process."""

    def __init__(self, cfg: DataConfig):
        self.root, self.cfg = cfg.token_root, cfg
        self.tours: List[str] = sorted(os.path.relpath(p, self.root) for p in
                                       glob.glob(os.path.join(self.root, TOUR_GLOB)))
        if not self.tours:
            raise FileNotFoundError(f"no token stores under {self.root}")
        self._meta: Dict[str, dict] = {}
        self._pathfinders: Dict[str, object] = {}   # scene .glb -> pathfinder
        self._islands: Dict[str, int] = {}          # tour -> the island it started on

    def meta(self, tour: str) -> dict:
        if tour not in self._meta:
            self._meta[tour] = read_tour_meta(os.path.join(self.root, tour))
        return self._meta[tour]

    def keyframes(self, tour: str) -> np.ndarray:
        return select_keyframes(self.meta(tour), self.cfg.eta, self.cfg.max_kf)

    def context(self, tour: str, rng: np.random.Generator):
        """The tour's keyframes -> (tokens (K, P, Dt) float16, poses (K, 3) float32 under the
        keyframe noise, drawn from `rng`)."""
        idx = self.keyframes(tour)
        if self.cfg.zero_tokens:
            tokens = np.zeros((len(idx), 1, 1), np.float16)
        else:
            with h5py.File(os.path.join(self.root, tour), "r") as f:
                tokens = f["tokens"][idx]
        kf_pose = self.meta(tour)["pose"][idx].astype(np.float32)
        jitter_(kf_pose, self.cfg.kf_noise_pos, self.cfg.kf_noise_yaw, rng)
        return tokens, kf_pose

    def eval_context(self, tour: str):
        """The tour's context under the fixed EVAL_SEED keyframe noise."""
        return self.context(tour, np.random.default_rng(EVAL_SEED))

    def scene(self, tour: str, seed_rng: Optional[np.random.Generator] = None) -> TourScene:
        """The tour's navmesh scene; a newly loaded pathfinder is seeded from `seed_rng`."""
        meta = self.meta(tour)
        pf = self._pathfinders.get(meta["scene_glb"])
        if pf is None:
            pf = self._pathfinders[meta["scene_glb"]] = load_pathfinder(meta["scene_glb"])
            if seed_rng is not None:
                pf.seed(int(seed_rng.integers(1 << 31)))
        if tour not in self._islands:
            self._islands[tour] = tour_island(pf, meta["start_point"])
        return TourScene(pf, self._islands[tour], meta["floor_y"], meta["coverage"],
                         meta["origin_position"], meta["mpp"])


# ---------------------------------------------------------------- items
def _item(tokens, kf_pose, query_pose, goal_position, targets) -> tuple:
    return (torch.from_numpy(np.ascontiguousarray(tokens)), torch.from_numpy(kf_pose),
            torch.from_numpy(np.asarray(query_pose, np.float32)),
            torch.from_numpy(np.asarray(goal_position, np.float32)),
            torch.from_numpy(np.asarray(targets, np.int64)))


def rotate_frame(theta: float, kf_pose: np.ndarray, query_pose: np.ndarray,
                 goal_position: np.ndarray) -> None:
    """In place: rotate poses and the goal position by theta about +y; targets are egocentric
    and unchanged."""
    R = R_ground(theta, np.float32)
    for pose in (kf_pose, query_pose):
        pose[:, :2] = pose[:, :2] @ R.T
        pose[:, 2] = wrap_angle(pose[:, 2] + theta)
    goal_position[:] = goal_position @ R.T


class TourDataset(Dataset):
    """Training items over the store's tours: item i comes from tour i or, when that yields no queries,
    from the first of 8 random tours that does (RuntimeError if none)."""

    def __init__(self, cfg: DataConfig):
        self.cfg, self.episode_cfg = cfg, EpisodeConfig()
        self.store = TourStore(cfg)
        self._rng = np.random.default_rng(cfg.seed)

    def reseed(self, seed: int) -> None:
        self._rng = np.random.default_rng(seed)

    def __len__(self) -> int:
        return len(self.store.tours)

    def sample_queries(self, tour: str, rng: np.random.Generator):
        """Expert trajectories from random starts to one random goal -> (poses (T, 3),
        goal_position (2,), targets (T, chunk)), or None. Queries are up to `queries` of their
        states, a `state_noise` fraction moved and relabelled; a moved state the expert has
        no route from keeps its recorded state and labels."""
        cfg, ecfg = self.cfg, self.episode_cfg
        scene = self.store.scene(tour, seed_rng=rng)
        goal = sample_goal(scene, ecfg)
        if goal is None:
            return None
        episodes: List[Episode] = [e for e in (sample_episode(scene, rng, ecfg, goal, max_tries=20)
                                               for _ in range(cfg.episodes_per_item)) if e is not None]
        if not episodes:
            return None
        pool = [(e, s) for e, episode in enumerate(episodes) for s in range(len(episode.poses))]
        if len(pool) > cfg.queries:
            pool = [pool[j] for j in rng.choice(len(pool), cfg.queries, replace=False)]
        poses = np.empty((len(pool), 3), np.float32)
        targets = np.empty((len(pool), cfg.chunk), np.int64)
        for q, (e, s) in enumerate(pool):
            episode = episodes[e]
            poses[q] = episode.poses[s]
            targets[q] = chunk_targets_from_trajectory(episode.actions, s, cfg.chunk)
            if rng.random() < cfg.state_noise:
                x, z, yaw = (float(v) for v in episode.poses[s])
                point = np.array([x, episode.start_point[1], z], np.float64)
                moved = snap_point(scene.pathfinder, point + np.array(
                    [rng.normal(0, cfg.state_noise_pos), 0.0, rng.normal(0, cfg.state_noise_pos)]))
                if scene.on_floor(moved, ecfg):
                    point = moved
                yaw = float(yaw + rng.uniform(-cfg.state_noise_yaw, cfg.state_noise_yaw))
                chunk = chunk_targets_from_state(scene.pathfinder, point, yaw, episode.goal_point,
                                                 ecfg, cfg.chunk)
                if chunk is not None:
                    poses[q], targets[q] = [point[0], point[2], yaw], chunk
        return poses, goal[[0, 2]].astype(np.float32), targets

    def __getitem__(self, i: int) -> tuple:
        cfg, rng = self.cfg, self._rng
        for j in [i] + list(rng.integers(0, len(self), size=8)):
            tour = self.store.tours[j]
            queries = self.sample_queries(tour, rng)
            if queries is not None:
                break
        else:
            raise RuntimeError(f"no episode accepted near item {i}")
        tokens, kf_pose = self.store.context(tour, rng)
        poses, goal_position, targets = queries
        if cfg.rot_aug:
            rotate_frame(float(rng.uniform(0.0, 2 * np.pi)), kf_pose, poses, goal_position)
        jitter_(poses, cfg.query_noise_pos, cfg.query_noise_yaw, rng)
        return _item(tokens, kf_pose, poses, goal_position, targets)


class EvalBank(Dataset):
    """A frozen bank (collection.bank): per row a tour, one closed-loop episode, and clean
    query states with their chunk targets. Given a store, row i is an item in its tour's
    evaluation context."""

    def __init__(self, path: str, store: Optional[TourStore] = None):
        z = np.load(path, allow_pickle=False)
        self.tours = [str(t) for t in z["tour"]]
        self.start_point, self.start_yaw = z["start3d"], z["start_yaw"]
        self.goal_point, self.geodesic = z["goal3d"], z["geo0"]
        self.query_pose, self.goal_position, self.targets = z["qpose"], z["goal"], z["targets"]
        self.store = store

    def __len__(self) -> int:
        return len(self.tours)

    def episode(self, i: int) -> Episode:
        return Episode(self.start_point[i], float(self.start_yaw[i]), self.goal_point[i],
                       float(self.geodesic[i]))

    def detour_ratio(self, i: int) -> float:
        return detour_ratio(self.start_point[i], self.goal_point[i], self.geodesic[i])

    def __getitem__(self, i: int) -> tuple:
        tokens, kf_pose = self.store.eval_context(self.tours[i])
        return _item(tokens, kf_pose, self.query_pose[i], self.goal_position[i],
                     self.targets[i][:, :self.store.cfg.chunk])


# ---------------------------------------------------------------- batches
class Batch(NamedTuple):
    kf_tokens: torch.Tensor             # (n_valid, P, Dt) PACKED: real keyframes, row-major over kf_valid
    kf_pose: torch.Tensor               # (B, K, 3)
    kf_valid: torch.Tensor              # (B, K) bool
    query_pose: torch.Tensor            # (B, T, 3)
    goal_position: torch.Tensor         # (B, 2)
    targets: torch.Tensor               # (B, T, chunk) int64; PAD_TARGET at padded queries

    def to(self, device) -> "Batch":
        return Batch(*(t.to(device, non_blocking=True) for t in self))


def collate(items: List[tuple]) -> Batch:
    """Pad poses and queries to the batch maximum; tokens ship packed, never padded."""
    tokens, kf_poses, query_poses, goal_positions, labels = zip(*items)
    B, K, T = len(items), max(len(t) for t in tokens), max(len(q) for q in query_poses)
    kf_pose, kf_valid = torch.zeros(B, K, 3), torch.zeros(B, K, dtype=torch.bool)
    query_pose = torch.zeros(B, T, 3)
    targets = torch.full((B, T, labels[0].shape[1]), PAD_TARGET, dtype=torch.int64)
    for i, (item_tokens, pose, queries, label) in enumerate(zip(tokens, kf_poses, query_poses, labels)):
        k, t = len(item_tokens), len(queries)
        kf_pose[i, :k], kf_valid[i, :k] = pose, True
        query_pose[i, :t], targets[i, :t] = queries, label
    return Batch(torch.cat(tokens), kf_pose, kf_valid, query_pose, torch.stack(goal_positions), targets)


def worker_init_fn(worker_id: int) -> None:
    dataset = torch.utils.data.get_worker_info().dataset
    dataset.reseed(dataset.cfg.seed + 1000 * (worker_id + 1))
