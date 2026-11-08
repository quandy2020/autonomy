"""Closed-loop evaluation of a planner checkpoint on a frozen bank.

Per bank row, the tour's map is encoded once for the episode's goal and the policy moves the
habitat agent (`--stepper sim`, the default) or the bare navmesh, with identical motion. Each query
decodes an action chunk and executes its first `exec_horizon` actions before querying again.
Reports success, SPL and stop rate, overall and by detour ratio (geodesic / euclidean).

Usage: python -m pan.planner.eval --ckpt logs/<run>/checkpoints/final.pt \
    --token-root <val_tokens> --bank <bank.npz> --gpu 0 --out logs/<run>/eval.json
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import os
from typing import Iterator, List, Optional, Tuple

import numpy as np
import torch

from .dataset import EvalBank, TourStore
from .env import EpisodeConfig, Stepper, policy_rollout
from .train import AMP, effective_rank, load_planner

DETOUR_BUCKETS = (("ratio<1.2", 0.0, 1.2), ("1.2-1.5", 1.2, 1.5), ("ratio>=1.5", 1.5, float("inf")))


def float_tensor(x, device, *shape) -> torch.Tensor:
    return torch.from_numpy(np.asarray(x, np.float32)).view(*shape).to(device)


@torch.no_grad()
def summarize_frames(model, tokens: torch.Tensor, device):
    """Frozen tokens (K, P, Dt) of one map, on `device` -> (entries (1, K, D), kf_valid (1, K))."""
    kf_valid = torch.ones(1, tokens.shape[0], dtype=torch.bool, device=device)
    with torch.autocast(device.type, dtype=AMP):
        return model.summarize(tokens, kf_valid), kf_valid


@torch.no_grad()
def summarize_tour(model, store: TourStore, tour: str, device):
    """A tour's keyframe summaries, the goal-independent prefix of its map encoding ->
    (entries (1, K, D), kf_pose (1, K, 3), kf_valid (1, K))."""
    tokens, kf_pose = store.eval_context(tour)
    tokens = torch.from_numpy(np.ascontiguousarray(tokens)).to(device)
    entries, kf_valid = summarize_frames(model, tokens, device)
    return entries, torch.from_numpy(kf_pose)[None].to(device), kf_valid


@torch.no_grad()
def encode_for_goal(model, summaries, goal_position, device):
    """`summarize_tour`'s (entries, kf_pose, kf_valid) -> the map memory for `goal_position` (x, z)."""
    with torch.autocast(device.type, dtype=AMP):
        return model.encode_map(*summaries, float_tensor(goal_position, device, 1, 2))


class ChunkCommitPolicy:
    """Decode an action chunk at the agent's pose, execute its first `exec_horizon` actions,
    then query again. A STOP inside the committed prefix ends the episode when executed."""

    def __init__(self, model, memory, device, exec_horizon: int = 4):
        self.model, self.memory, self.device = model, memory, device
        self.exec_horizon = exec_horizon
        self.queue: List[int] = []

    @torch.no_grad()
    def decode(self, pose, goal_position) -> torch.Tensor:
        """(chunk, A) float logits at `pose` (x, z, yaw) toward `goal_position` (x, z)."""
        with torch.autocast(self.device.type, dtype=AMP):
            logits = self.model.read(self.memory, float_tensor(pose, self.device, 1, 1, 3),
                                     float_tensor(goal_position, self.device, 1, 2))
        return logits[0, 0].float()

    def __call__(self, pose, goal_position, stepper=None) -> int:
        if not self.queue:
            self.queue.extend(self.decode(pose, goal_position).argmax(-1).tolist()[:self.exec_horizon])
        return self.queue.pop(0)


def row_result(bank: EvalBank, i: int, rollout: dict, **extra) -> dict:
    return dict(row=int(i), file=bank.tours[i], **rollout, ratio=bank.detour_ratio(i), **extra)


def summarize_results(results: List[dict]) -> dict:
    """Success, SPL and stop rate over `results`, overall and per DETOUR_BUCKETS ratio."""
    success = np.array([r["success"] for r in results], float)
    spl = np.array([r["spl"] for r in results], float)
    ratio = np.array([r["ratio"] for r in results], float)
    by_ratio = {}
    for name, lo, hi in DETOUR_BUCKETS:
        m = (ratio >= lo) & (ratio < hi)
        if m.any():
            by_ratio[name] = dict(n=int(m.sum()), success=float(success[m].mean()),
                                  spl=float(spl[m].mean()))
    return dict(n=len(results), success=float(success.mean()), spl=float(spl.mean()),
                stopped=float(np.mean([r["stopped"] for r in results])), by_ratio=by_ratio)


def bank_rows(rows: str, limit: int, n: int) -> np.ndarray:
    """Comma-separated `rows`, sorted, when given; else the first `limit` of `n` rows (0 = all)."""
    if rows:
        return np.array(sorted(int(x) for x in rows.split(",")))
    return np.arange(min(limit, n) if limit else n)


def tour_rows(model, store: TourStore, bank: EvalBank, rows, device, stepper: Stepper,
              simulate: bool) -> Iterator[Tuple[int, tuple]]:
    """(row, its tour's `summarize_tour`) over tour-sorted `rows`. Before a tour's first row, the
    tour is summarized and `stepper` enters its scene (`simulate`) or its bare navmesh."""
    tour = None
    for i in rows:
        if bank.tours[i] != tour:
            tour = bank.tours[i]
            summaries = summarize_tour(model, store, tour, device)
            if simulate:
                stepper.use_scene(store.meta(tour)["scene_glb"])
            else:
                stepper.use_pathfinder(store.scene(tour).pathfinder)
        yield i, summaries


@torch.no_grad()
def closed_loop_eval(model, store: TourStore, bank: EvalBank, device,
                     rows: Optional[np.ndarray] = None, record_trajectory: bool = False,
                     exec_horizon: int = 4, stepper: Optional[Stepper] = None) -> dict:
    """Aggregate metrics and per-row results over `rows` of the bank (default: all). Given a
    `stepper`, the habitat agent moves in each tour's scene; by default the agent moves on
    the tour's navmesh alone, with the same motion."""
    model.eval()
    cfg = EpisodeConfig()
    rows = np.arange(len(bank)) if rows is None else np.asarray(rows)
    simulate, stepper = stepper is not None, stepper or Stepper(cfg)
    results, ranks = [], []
    for i, summaries in tour_rows(model, store, bank, rows, device, stepper, simulate):
        episode = bank.episode(i)
        memory = encode_for_goal(model, summaries, episode.goal_position, device)
        n_kf = summaries[2].shape[1]
        if n_kf > 1:
            ranks.append(effective_rank(memory[0][0, :n_kf]))
        policy = ChunkCommitPolicy(model, memory, device, exec_horizon)
        results.append(row_result(bank, i, policy_rollout(stepper, policy, episode, cfg,
                                                          record_trajectory)))
    return dict(**summarize_results(results), exec_horizon=exec_horizon,
                entry_rank_median=float(np.median(ranks)) if ranks else 0.0, results=results)


# ---------------------------------------------------------------- CLI
def add_eval_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--ckpt", required=True)
    p.add_argument("--token-root", required=True)
    p.add_argument("--bank", required=True)
    p.add_argument("--gpu", type=int, default=0)
    p.add_argument("--eta", type=int, default=None, help="context gate eta (default: the checkpoint's)")
    p.add_argument("--max-kf", type=int, default=None)
    p.add_argument("--kf-noise-pos", type=float, default=0.0,
                   help="metres, uniform half-width of the jitter on each keyframe position (EVAL_SEED)")
    p.add_argument("--kf-noise-yaw", type=float, default=0.0,
                   help="radians, uniform half-width of the jitter on each keyframe yaw")
    p.add_argument("--exec-horizon", type=int, default=4,
                   help="actions executed per decoded chunk before querying again")
    p.add_argument("--limit", type=int, default=0, help="evaluate the first N rows only")


def load_eval_setup(args: argparse.Namespace):
    """-> (the checkpoint's model on the GPU, a token store under the checkpoint's data config with
    the flags' overrides, the bank, the device)."""
    device = torch.device(f"cuda:{args.gpu}")
    model, cfg = load_planner(args.ckpt)
    eta = cfg.data.eta if args.eta is None else args.eta  # 0 (every frame) is a value, not a fallback
    store = TourStore(dataclasses.replace(cfg.data, token_root=args.token_root, eta=eta,
                                          max_kf=args.max_kf or cfg.data.max_kf,
                                          kf_noise_pos=args.kf_noise_pos, kf_noise_yaw=args.kf_noise_yaw))
    return model.to(device).eval(), store, EvalBank(args.bank), device


def write_json(obj: dict, path: str) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as fh:
        json.dump(obj, fh)


def main():
    p = argparse.ArgumentParser()
    add_eval_args(p)
    p.add_argument("--record-traj", action="store_true")
    p.add_argument("--stepper", choices=("sim", "navmesh"), default="sim",
                   help="sim steps the habitat agent (one scene load per tour); navmesh uses "
                        "pathfinder.try_step alone. The motion is identical.")
    p.add_argument("--out", default=None)
    args = p.parse_args()

    model, store, bank, device = load_eval_setup(args)
    rows = bank_rows("", args.limit, len(bank))
    stepper = Stepper(EpisodeConfig(), args.gpu) if args.stepper == "sim" else None
    out = closed_loop_eval(model, store, bank, device, rows=rows, record_trajectory=args.record_traj,
                           exec_horizon=args.exec_horizon, stepper=stepper)
    if stepper is not None:
        out.update(scene_loads=stepper.n_loads, scene_load_seconds=round(stepper.load_seconds, 1))
        stepper.close()
    print(json.dumps({k: v for k, v in out.items() if k != "results"}, indent=2))
    if args.out:
        write_json(out, args.out)
        print("wrote", args.out)


if __name__ == "__main__":
    main()
