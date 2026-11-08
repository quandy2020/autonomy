"""Train the VNTPA planner on synthetic demonstrations."""

from __future__ import annotations

import argparse
import os

import numpy as np
import torch
import yaml

from model import VntpaPlanner


def make_batch(batch: int, token_count: int, token_dim: int, context_size: int,
               pose_dim: int, chunk_length: int) -> dict[str, torch.Tensor]:
    return {
        "depth_feature": torch.randn(batch, token_count, token_dim),
        "query": torch.randn(batch, pose_dim),
        "context": torch.randn(batch, context_size, token_dim),
        "context_pose": torch.randn(batch, context_size, pose_dim),
        "context_mask": torch.zeros(batch, context_size, dtype=torch.bool),
        "actions": torch.randn(batch, chunk_length, 3) * 0.1,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/train_vntpa.yaml")
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    data = config["data"]
    model = VntpaPlanner(token_dim=data["token_dim"], pose_dim=data["pose_dim"],
                         context_size=data["context_size"],
                         chunk_length=data["chunk_length"])
    optimizer = torch.optim.AdamW(model.parameters(), lr=config["train"]["learning_rate"])
    os.makedirs(config["train"]["checkpoint_dir"], exist_ok=True)
    for step in range(config["train"]["steps"]):
        batch = make_batch(config["train"]["batch_size"], data["token_count"],
                           data["token_dim"], data["context_size"], data["pose_dim"],
                           data["chunk_length"])
        prediction = model(batch["depth_feature"], batch["query"], batch["context"],
                           batch["context_pose"], batch["context_mask"])
        loss = torch.nn.functional.mse_loss(prediction, batch["actions"])
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        if step % 50 == 0:
            print(f"step={step} loss={loss.item():.6f}")
    path = os.path.join(config["train"]["checkpoint_dir"], "last.pt")
    torch.save(model.state_dict(), path)
    print("saved", path)


if __name__ == "__main__":
    main()
