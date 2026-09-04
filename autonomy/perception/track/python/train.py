#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors
"""Train the ground-robot track network (simple or planar MINCO)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch
from torch.utils.data import DataLoader

from config import load_config
from data.synthetic import SyntheticFollowDataset
from loss.track_loss import TrackLoss
from models.network import TrackNetwork
from primitive import Lattice, LatticeConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train ground-robot track network"
    )
    parser.add_argument("--config", type=str, default=None)
    parser.add_argument("--device", type=str, default=None)
    parser.add_argument("--epochs", type=int, default=None)
    parser.add_argument(
        "--trajectory-mode",
        type=str,
        default=None,
        choices=["simple", "minco"],
        help="Override config trajectory_mode",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = load_config(args.config)
    if args.epochs is not None:
        config["epochs"] = args.epochs
    if args.trajectory_mode is not None:
        config["trajectory_mode"] = args.trajectory_mode
    trajectory_mode = str(config.get("trajectory_mode", "simple"))
    device = torch.device(
        args.device or ("cuda" if torch.cuda.is_available() else "cpu")
    )

    dataset = SyntheticFollowDataset(config)
    loader = DataLoader(
        dataset,
        batch_size=int(config["batch_size"]),
        shuffle=True,
        num_workers=int(config.get("num_workers", 0)),
        drop_last=True,
    )

    network = TrackNetwork(
        hidden=int(config["hidden_dim"]),
        trajectory_mode=trajectory_mode,
    ).to(device)
    network.set_grid(int(config["vertical_num"]), int(config["horizon_num"]))
    lattice = Lattice(
        LatticeConfig(
            horizontal_bin_count=int(config["horizon_num"]),
            vertical_bin_count=int(config["vertical_num"]),
            camera_horizontal_field_of_view_deg=float(
                config["horizon_camera_fov"]
            ),
            planning_horizon_m=float(config["radio_range"]),
            max_linear_velocity_mps=float(config["vel_max"]),
            max_yaw_rate_rps=float(config["wz_max"]),
        ),
        device=device,
    )
    criterion = TrackLoss(config, lattice)
    optimizer = torch.optim.Adam(network.parameters(), lr=float(config["lr"]))

    checkpoint_directory = Path(config["checkpoint_dir"])
    checkpoint_directory.mkdir(parents=True, exist_ok=True)
    checkpoint_name = (
        "yopo_track_minco_last.pt"
        if trajectory_mode == "minco"
        else "yopo_track_last.pt"
    )

    for epoch in range(int(config["epochs"])):
        network.train()
        epoch_losses = []
        for batch in loader:
            depth = batch["depth"].to(device)
            observation = batch["observation"].to(device)
            target = batch["target_xy_m"].to(device)
            occupancy = batch["occupancy"].to(device)

            params_raw, score_logits, objectness_logits = network(
                depth, observation
            )
            losses = criterion(
                params_raw,
                score_logits,
                objectness_logits,
                target,
                occupancy,
            )
            optimizer.zero_grad(set_to_none=True)
            losses["total"].backward()
            optimizer.step()
            epoch_losses.append(float(losses["total"].item()))

        mean_loss = sum(epoch_losses) / max(len(epoch_losses), 1)
        print(
            f"epoch {epoch:03d}  mode={trajectory_mode}  loss={mean_loss:.4f}"
        )
        torch.save(
            {
                "model": network.state_dict(),
                "config": config,
                "epoch": epoch,
                "trajectory_mode": trajectory_mode,
            },
            checkpoint_directory / checkpoint_name,
        )

    metadata = {
        "config": config,
        "trajectory_mode": trajectory_mode,
        "inputs": ["depth", "observation"],
        "outputs": ["prediction"],
        "prediction_channels": 12 if trajectory_mode == "minco" else 6,
    }
    (checkpoint_directory / "meta.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )
    print(f"saved checkpoints to {checkpoint_directory}/{checkpoint_name}")


if __name__ == "__main__":
    main()
