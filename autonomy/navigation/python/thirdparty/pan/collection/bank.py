"""Build a frozen episode bank over a token corpus.

Per tour, up to `--per-tour` expert episodes, each with `--queries` of its states (drawn with
replacement) and their chunk targets, all STOP where the expert has no route. One `--seed` stream
draws everything. One row per episode, Q = `--queries`:

    tour       (R,)            str   token store path under --token-root
    start3d    (R, 3)          f64   episode start navmesh point
    start_yaw  (R,)            f64   radians
    goal3d     (R, 3)          f64   goal navmesh point
    geo0       (R,)            f64   metres, start -> goal geodesic
    qpose      (R, Q, 3)       f32   query poses (x, z, yaw)
    goal       (R, 2)          f32   goal position
    targets    (R, Q, chunk)   i64   expert chunk targets

Usage: python -m pan.collection.bank --token-root <tokens> --out <bank.npz> --per-tour 2
"""
from __future__ import annotations

import argparse
import os

import numpy as np

from thirdparty.pan.planner.dataset import DataConfig, TourStore
from thirdparty.pan.planner.env import EpisodeConfig
from thirdparty.pan.planner.expert import chunk_targets_from_state, sample_episode, stop_padded

COLUMNS = ("tour", "start3d", "start_yaw", "goal3d", "geo0", "qpose", "goal", "targets")


def build_bank(token_root: str, out: str, per_tour: int, queries: int, chunk: int,
               max_tours: int, seed: int) -> None:
    store = TourStore(DataConfig(token_root=token_root))
    cfg = EpisodeConfig()
    rng = np.random.default_rng(seed)
    tours = store.tours
    if max_tours and len(tours) > max_tours:
        tours = [tours[i] for i in rng.permutation(len(tours))[:max_tours]]
    rows = []                           # one tuple per episode, in COLUMNS order
    for tour in sorted(tours):
        scene = store.scene(tour)
        scene.pathfinder.seed(int(rng.integers(1 << 31)))
        for _ in range(per_tour):
            episode = sample_episode(scene, rng, cfg)
            if episode is None:
                continue
            poses = episode.poses[rng.integers(0, len(episode.poses), size=queries)]
            targets = [chunk_targets_from_state(scene.pathfinder,
                                                np.array([p[0], episode.start_point[1], p[1]]),
                                                float(p[2]), episode.goal_point, cfg, chunk)
                       for p in poses]
            rows.append((tour, episode.start_point, episode.start_yaw, episode.goal_point,
                         episode.geodesic, poses, episode.goal_position.astype(np.float32),
                         np.stack([stop_padded((), chunk) if t is None else t for t in targets])))
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    assert rows, f"no episode accepted over {len(tours)} tours"
    np.savez(out, **{k: np.stack(column) for k, column in zip(COLUMNS, zip(*rows))})
    print(f"wrote {out}: {len(rows)} rows over {len({row[0] for row in rows})} tours")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--token-root", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--per-tour", type=int, default=2)
    p.add_argument("--queries", type=int, default=16)
    p.add_argument("--chunk", type=int, default=8)
    p.add_argument("--max-tours", type=int, default=0, help="0 = all tours")
    p.add_argument("--seed", type=int, default=0)
    build_bank(**vars(p.parse_args()))


if __name__ == "__main__":
    main()
