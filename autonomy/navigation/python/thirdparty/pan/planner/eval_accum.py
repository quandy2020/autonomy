"""Closed-loop evaluation with online context accumulation.

At each query, until `--add-cap` views are added, the agent renders its depth view; a view the
coverage gate keeps is encoded with the frozen DeFM encoder and its summary appended to the map,
which is re-encoded after a change. Under `--query-noise-*` a new entry is stamped at the believed
pose the model reads, otherwise under the map's keyframe noise. The gate starts from the cells the
tour's context keyframes claim in their recorded frames. `--gate-eta 0` adds every queried view;
`--no-accum` runs the same loop without additions.

Usage: python -m pan.planner.eval_accum --ckpt <ckpt> --token-root <val_tokens> \
    --bank <bank.npz> --seed-traj-root <val_shards> --gpu 0 --out logs/<run>/eval_accum.json
"""
from __future__ import annotations

import argparse
import json
import os
from typing import Optional, Tuple

import h5py
import numpy as np
import torch

from thirdparty.pan.util.keyframe_gate import CoverageGate, frame_cells

from .dataset import jitter_
from .env import EpisodeConfig, Stepper, policy_rollout
from .eval import (ChunkCommitPolicy, add_eval_args, bank_rows, encode_for_goal, float_tensor,
                   load_eval_setup, row_result, summarize_frames, summarize_results, tour_rows,
                   write_json)


class AccumulatingPolicy(ChunkCommitPolicy):
    """Chunk-commit decoding that grows the map with up to `add_cap` of the agent's own
    views, each offered to `gate` (None adds every view). The map starts from the tour's
    summaries and is encoded for the goal at the first query and after each addition.

    Under `query_noise` (metres, radians) the model reads, and new entries are stamped with,
    the believed pose; stepping and scoring stay exact. Without it, new entries carry the
    map's own `kf_noise` instead."""

    def __init__(self, model, summaries, encoder, device, rng: np.random.Generator, exec_horizon: int,
                 add_cap: int, gate: Optional[CoverageGate], kf_noise: Tuple[float, float],
                 query_noise: Tuple[float, float]):
        super().__init__(model, None, device, exec_horizon)
        self.entries, self.kf_pose, self.kf_valid = summaries
        self.encoder, self.rng, self.add_cap, self.gate = encoder, rng, add_cap, gate
        self.kf_noise, self.query_noise = kf_noise, query_noise
        self.n_added = self.n_rendered = 0

    def _reported(self, pose, noise) -> np.ndarray:
        """`pose` as reported under `noise`, as a fresh (3,) float32 array."""
        reported = np.array(pose, np.float32)[None]
        jitter_(reported, *noise, self.rng)
        return reported[0]

    def _add(self, depth: np.ndarray, stamp: np.ndarray) -> None:
        entry, valid = summarize_frames(self.model, self.encoder.encode(depth), self.device)
        self.entries = torch.cat([self.entries, entry], 1)
        self.kf_pose = torch.cat([self.kf_pose, float_tensor(stamp, self.device, 1, 1, 3)], 1)
        self.kf_valid = torch.cat([self.kf_valid, valid], 1)
        self.n_added += 1
        self.memory = None

    @torch.no_grad()
    def __call__(self, pose, goal_position, stepper: Stepper) -> int:
        if self.queue:
            return self.queue.pop(0)
        believed = self._reported(pose, self.query_noise)
        if self.n_added < self.add_cap:
            depth, c2w = stepper.observe()
            self.n_rendered += 1
            if self.gate is None or self.gate.offer(frame_cells(depth, c2w)):
                self._add(depth, believed if any(self.query_noise) else self._reported(pose, self.kf_noise))
        if self.memory is None:
            self.memory = encode_for_goal(self.model, (self.entries, self.kf_pose, self.kf_valid),
                                          goal_position, self.device)
        return super().__call__(believed, goal_position)


def tour_claims(shard: str, keyframes: np.ndarray, kf_pose: np.ndarray) -> set:
    """The cells a tour's context keyframes claim, from their recorded frames: the raw shard
    holds each frame's depth and camera-to-world."""
    with h5py.File(shard, "r") as f:
        depth = f["depth"][keyframes]
        c2w = f["camera_pose_world"][keyframes].astype(np.float64)
    assert np.allclose(c2w[:, [0, 2], 3], kf_pose[:, :2], atol=1e-3), (
        f"{shard}: recorded cameras do not match the context poses")
    claims: set = set()
    for d, m in zip(depth, c2w):
        claims.update(frame_cells(d, m)[1].tolist())
    return claims


def main():
    p = argparse.ArgumentParser()
    add_eval_args(p)
    p.add_argument("--query-noise-pos", type=float, default=0.0,
                   help="metres, uniform half-width of the error on the agent's believed position "
                        "at every query")
    p.add_argument("--query-noise-yaw", type=float, default=0.0,
                   help="radians, uniform half-width of the error on the agent's believed heading")
    p.add_argument("--add-cap", type=int, default=256, help="most views added per episode")
    p.add_argument("--gate-eta", type=int, default=10,
                   help="coverage gate eta for added views; 0 = add every queried view")
    p.add_argument("--seed-traj-root", default="",
                   help="raw exploration shards of the bank's tours (depth, camera_pose_world); the "
                        "gate starts from the cells their context keyframes claim")
    p.add_argument("--no-accum", action="store_true",
                   help="paired control: the same code path without accumulation")
    p.add_argument("--rows", default="", help="comma-separated bank rows (default: all)")
    p.add_argument("--save-traj", action="store_true",
                   help="store each row's executed trajectory in the output")
    p.add_argument("--out", required=True)
    args = p.parse_args()
    gated = not args.no_accum and args.gate_eta > 0
    if gated and not args.seed_traj_root:
        p.error("--seed-traj-root is required when added views are gated")

    model, store, bank, device = load_eval_setup(args)
    rows = bank_rows(args.rows, args.limit, len(bank))

    from thirdparty.pan.util.defm_engine import DeFMEncoder
    encoder = DeFMEncoder(device)
    episode_cfg = EpisodeConfig()
    stepper = Stepper(episode_cfg, args.gpu, sensors=("depth",))
    results, claims_tour = [], None
    for i, summaries in tour_rows(model, store, bank, rows, device, stepper, simulate=True):
        gate = None
        if gated:
            if bank.tours[i] != claims_tour:
                claims_tour = bank.tours[i]
                keyframes = store.keyframes(claims_tour)
                claims = tour_claims(os.path.join(args.seed_traj_root, claims_tour), keyframes,
                                     store.meta(claims_tour)["pose"][keyframes])
            gate = CoverageGate(args.gate_eta, claims)
        episode = bank.episode(i)
        policy = AccumulatingPolicy(model, summaries, encoder, device, np.random.default_rng(int(i)),
                                    exec_horizon=args.exec_horizon,
                                    add_cap=0 if args.no_accum else args.add_cap, gate=gate,
                                    kf_noise=(store.cfg.kf_noise_pos, store.cfg.kf_noise_yaw),
                                    query_noise=(args.query_noise_pos, args.query_noise_yaw))
        rollout = policy_rollout(stepper, policy, episode, episode_cfg, record_trajectory=True)
        trajectory = np.asarray(rollout["traj"], np.float32)
        path_len = (float(np.linalg.norm(np.diff(trajectory, axis=0), axis=1).sum())
                    if len(trajectory) > 1 else 0.0)
        result = row_result(bank, i, rollout, path_len=path_len, n_added=policy.n_added,
                            n_rendered=policy.n_rendered)
        if not args.save_traj:
            result["traj"] = None
        results.append(result)
        if len(results) % 25 == 0:
            print(f"[{len(results)}/{len(rows)}] running success "
                  f"{np.mean([x['success'] for x in results]):.3f}", flush=True)
    out = dict(**summarize_results(results), accum=not args.no_accum, add_cap=args.add_cap,
               gate_eta=args.gate_eta if not args.no_accum else None,
               kf_noise=[args.kf_noise_pos, args.kf_noise_yaw],
               query_noise=[args.query_noise_pos, args.query_noise_yaw],
               exec_horizon=args.exec_horizon, scene_loads=stepper.n_loads,
               scene_load_seconds=round(stepper.load_seconds, 1), results=results)
    stepper.close()
    write_json(out, args.out)
    print(json.dumps({k: v for k, v in out.items() if k != "results"}))


if __name__ == "__main__":
    main()
