"""Evaluate a VNTPA checkpoint on synthetic batches."""

from __future__ import annotations

import argparse

import torch
import yaml

from model import VntpaPlanner
from train import make_batch


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/eval_vntpa.yaml")
    parser.add_argument("--checkpoint", default="checkpoints/last.pt")
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    data = config["data"]
    model = VntpaPlanner(token_dim=data["token_dim"], pose_dim=data["pose_dim"],
                         context_size=data["context_size"],
                         chunk_length=data["chunk_length"])
    model.load_state_dict(torch.load(args.checkpoint, map_location="cpu"))
    model.eval()
    with torch.no_grad():
        batch = make_batch(config["eval"]["batch_size"], data["token_count"],
                           data["token_dim"], data["context_size"], data["pose_dim"],
                           data["chunk_length"])
        prediction = model(batch["depth_feature"], batch["query"], batch["context"],
                           batch["context_pose"], batch["context_mask"])
        loss = torch.nn.functional.mse_loss(prediction, batch["actions"])
    print(f"eval loss={loss.item():.6f}")


if __name__ == "__main__":
    main()
