"""Train the VNT-PA planner.

The paper pipeline is ``python -m vntpa.cli.train`` (30k AdamW steps on HM3D
tokens). This script only keeps a synthetic smoke path behind ``--synthetic``.
"""

from __future__ import annotations

import argparse
import os
import sys

import yaml


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/train_vntpa.yaml")
    parser.add_argument(
        "--synthetic",
        action="store_true",
        help="train a tiny random-weight stub; not a paper checkpoint",
    )
    args, rest = parser.parse_known_args()
    if not args.synthetic:
        print(
            "tools/vntpa/train.py is not the paper trainer.\n"
            "Collect HM3D tokens then run:\n"
            "  python -m vntpa.cli.train --nproc 1 --accum 4 --wandb-project '' \\\n"
            "    --token-root $DATA/tokens/train --val-token-root $DATA/tokens/val \\\n"
            "    --val-bank $DATA/banks/bank_val.npz\n"
            "Pass --synthetic only for a random-weight compile smoke test.",
            file=sys.stderr,
        )
        raise SystemExit(2)

    import numpy as np
    import torch
    from model import VntpaPlanner

    with open(args.config, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    data = config["data"]
    model = VntpaPlanner(
        token_dim=data["token_dim"],
        pose_dim=data["pose_dim"],
        context_size=data["context_size"],
        chunk_length=data["chunk_length"],
    )
    optimizer = torch.optim.AdamW(model.parameters(), lr=config["train"]["learning_rate"])
    os.makedirs(config["train"]["checkpoint_dir"], exist_ok=True)
    steps = min(int(config["train"]["steps"]), 20)
    batch = config["train"].get("batch_size", 2)
    for step in range(steps):
        tensors = {
            "depth_feature": torch.randn(batch, data["token_count"], data["token_dim"]),
            "query": torch.randn(batch, data["pose_dim"]),
            "context": torch.randn(batch, data["context_size"], data["token_dim"]),
            "context_pose": torch.randn(batch, data["context_size"], data["pose_dim"]),
            "context_mask": torch.zeros(batch, data["context_size"], dtype=torch.bool),
            "actions": torch.randn(batch, data["chunk_length"], 3) * 0.1,
        }
        prediction = model(
            tensors["depth_feature"],
            tensors["query"],
            tensors["context"],
            tensors["context_pose"],
            tensors["context_mask"],
        )
        loss = torch.nn.functional.mse_loss(prediction, tensors["actions"])
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        print(f"step={step} loss={loss.item():.6f}")
    path = os.path.join(config["train"]["checkpoint_dir"], "last.pt")
    torch.save(model.state_dict(), path)
    print("saved", path, "(synthetic; not a VNT-PA checkpoint)")


if __name__ == "__main__":
    main()
