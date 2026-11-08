"""Preprocess an exploration corpus into per-tour planner token stores.

For every tour shard, encode every frame's depth with the frozen DeFM encoder, label keyframes
with the coverage gate at ETAS, and store:

    tokens            (N, 256, 384) f16   DeFM patch tokens per frame
    pose              (N, 3)        f32   camera pose (x, z, yaw)
    keep              (N, 3)        bool  coverage-gate keep bits at keep_etas (frame 0 kept)
    coverage_topdown                      copied through for episode sampling
    attrs: scene_id, scene_glb, floor_y, scene_xmin, scene_zmin, mpp, keep_etas, keep_spec
           (the CoverageSpec, JSON), agent_start_pos, coverage_frac, n_frames, seed

`--labels-only` rewrites the keep bits of existing stores from their raw shards, without
re-encoding, and drops any earlier gate's columns. Never relabel stores a running job reads.

Usage: python -m pan.collection.tokens --corpus <shards> --out <tokens> --gpu 0 --shard 0:4
       python -m pan.collection.tokens --corpus <shards> --out <tokens> --labels-only --procs 16
"""
from __future__ import annotations

import argparse
import glob
import os
import time
from multiprocessing import Pool

import h5py
import numpy as np
import torch

from thirdparty.pan.planner.dataset import TOUR_GLOB
from thirdparty.pan.util.geometry import heading_from_rotation
from thirdparty.pan.util.keyframe_gate import COVERAGE, CoverageGate, frame_cells

ETAS = (10, 20, 30)


def gate_labels(depth: np.ndarray, c2w: np.ndarray) -> np.ndarray:
    """(N, H, W) depth, (N, 4, 4) camera-to-world -> (N, len(ETAS)) keep bits; each frame's cells
    are computed once for all etas."""
    gates = [CoverageGate(eta) for eta in ETAS]
    keep = np.zeros((len(depth), len(ETAS)), bool)
    for t in range(len(depth)):
        cells = frame_cells(depth[t], c2w[t])
        keep[t] = [gate.offer(cells, force=t == 0) for gate in gates]
    return keep


def _write_labels(f: h5py.File, keep: np.ndarray) -> None:
    """(Re)write the keep bits and their attributes, dropping any earlier gate's columns."""
    for name in [n for n in f if n.startswith("keep")]:
        del f[name]
    for name in [a for a in f.attrs if a.startswith(("keep", "gate_"))]:
        del f.attrs[name]
    f.create_dataset("keep", data=keep)
    f.attrs["keep_etas"] = list(ETAS)
    f.attrs["keep_spec"] = COVERAGE.to_json()


def _stats(keep: np.ndarray) -> dict:
    return {"n_frames": len(keep), **{f"keep_rate_{e}": float(keep[:, i].mean()) for i, e in enumerate(ETAS)}}


def process_tour(fp: str, out_fp: str, encoder, batch: int = 64) -> dict:
    """Encode and label the raw shard `fp` into the token store `out_fp` (written as .tmp, then
    renamed) -> its frame count and keep rates."""
    with h5py.File(fp, "r") as f:
        depth, coverage, attrs = f["depth"][:], f["coverage_topdown"][:], dict(f.attrs)
        c2w = f["camera_pose_world"][:].astype(np.float64)
    keep = gate_labels(depth, c2w)
    tokens = torch.cat([encoder.encode(depth[s:s + batch]).cpu()
                        for s in range(0, len(depth), batch)]).numpy()
    pose = np.array([[m[0, 3], m[2, 3], heading_from_rotation(m)] for m in c2w], np.float32)

    os.makedirs(os.path.dirname(out_fp), exist_ok=True)
    tmp = out_fp + ".tmp"
    with h5py.File(tmp, "w") as f:
        f.create_dataset("tokens", data=tokens, dtype=np.float16)
        f.create_dataset("pose", data=pose)
        f.create_dataset("coverage_topdown", data=coverage)
        _write_labels(f, keep)
        for k in ("scene_id", "scene_glb", "floor_y", "scene_xmin", "scene_zmin",
                  "coverage_frac", "n_frames", "seed"):
            if k in attrs:
                f.attrs[k] = attrs[k]
        f.attrs["mpp"] = attrs["scene_topdown_mpp"]
        f.attrs["agent_start_pos"] = attrs["agent_start_pos"]
    os.replace(tmp, out_fp)
    return _stats(keep)


def relabel_tour(job) -> dict:
    """Rewrite the keep bits of the existing store `job[1]` from its raw shard `job[0]` -> its frame
    count and keep rates."""
    fp, out_fp = job
    with h5py.File(fp, "r") as f:
        depth, c2w = f["depth"][:], f["camera_pose_world"][:].astype(np.float64)
    keep = gate_labels(depth, c2w)
    with h5py.File(out_fp, "r+") as f:
        assert len(f["pose"]) == len(depth), f"{out_fp}: {len(f['pose'])} store frames, {len(depth)} raw"
        _write_labels(f, keep)
    return _stats(keep)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--corpus", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--gpu", type=int, default=0)
    p.add_argument("--shard", default="0:1", help="i:n, process tours where idx mod n == i")
    p.add_argument("--labels-only", action="store_true",
                   help="rewrite the keep bits in existing stores, without re-encoding tokens")
    p.add_argument("--procs", type=int, default=1, help="processes for --labels-only")
    args = p.parse_args()

    si, sn = map(int, args.shard.split(":"))
    tours = sorted(glob.glob(os.path.join(args.corpus, TOUR_GLOB)))
    jobs = [(fp, os.path.join(args.out, os.path.relpath(fp, args.corpus)))
            for i, fp in enumerate(tours) if i % sn == si]
    t0 = time.time()
    if args.labels_only:
        jobs = [job for job in jobs if os.path.exists(job[1])]
        print(f"relabelling {len(jobs)} stores")
        with Pool(args.procs) as pool:
            for done, r in enumerate(pool.imap_unordered(relabel_tour, jobs, chunksize=2), 1):
                if done % 100 == 0:
                    print(f"[{done}/{len(jobs)}] {time.time() - t0:.0f}s {r}", flush=True)
        print(f"DONE {len(jobs)} stores in {time.time() - t0:.0f}s")
        return

    from thirdparty.pan.util.defm_engine import DeFMEncoder
    encoder = DeFMEncoder(torch.device(f"cuda:{args.gpu}"), amp_dtype=torch.bfloat16)
    print(f"shard {si}/{sn}: {len(jobs)} of {len(tours)} tours")
    done = skipped = 0
    for fp, out_fp in jobs:
        if os.path.exists(out_fp):
            skipped += 1
            continue
        r = process_tour(fp, out_fp, encoder)
        done += 1
        if done % 20 == 0:
            el = time.time() - t0
            print(f"[{done}/{len(jobs)}] {el:.0f}s {el / done:.1f}s/tour {r}", flush=True)
    print(f"DONE done={done} skipped={skipped} in {time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
