"""Command-line entry point for Fathom RGB-D fine-tuning."""

import argparse
from pathlib import Path
from typing import Any

import torch
from torch.utils.data import DataLoader

from depth.api import load_model

from .dataset import RgbdManifestDataset
from .trainer import save_checkpoint, train_epoch


def parse_args() -> argparse.Namespace:
    """Parse fine-tuning configuration from the command line."""
    parser = argparse.ArgumentParser(description="Fine-tune a Fathom depth model")
    parser.add_argument("--checkpoint", required=True, help="Released checkpoint to fine-tune")
    parser.add_argument("--manifest", required=True, help="RGB-D JSONL manifest")
    parser.add_argument("--output", required=True, help="Directory for fine-tuned checkpoints")
    parser.add_argument("--epochs", type=int, default=1)
    parser.add_argument("--batch-size", type=int, default=1)
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--depth-scale", type=float, default=0.001)
    parser.add_argument("--dropout-probability", type=float, default=0.0)
    parser.add_argument("--mask-weight", type=float, default=0.1)
    parser.add_argument(
        "--device",
        default="cuda" if torch.cuda.is_available() else "cpu",
        help="Torch device for fine-tuning",
    )
    return parser.parse_args()


def main() -> None:
    """Fine-tune the released model and save one checkpoint per epoch."""
    args = parse_args()
    device = torch.device(args.device)
    checkpoint_path = Path(args.checkpoint)
    model_config = _load_model_config(checkpoint_path)
    model = load_model(checkpoint_path, device)
    dataset = RgbdManifestDataset(
        Path(args.manifest),
        dropout_probability=args.dropout_probability,
        depth_scale=args.depth_scale,
    )
    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=True)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.learning_rate)
    output_directory = Path(args.output)

    for epoch in range(1, args.epochs + 1):
        losses = train_epoch(model, loader, optimizer, device, args.mask_weight)
        print(
            f"epoch {epoch}: total={losses['total']:.6f} "
            f"depth={losses['depth']:.6f} mask={losses['mask']:.6f}"
        )
        save_checkpoint(
            output_directory / f"epoch-{epoch:04d}.pt",
            model,
            model_config,
            optimizer,
            step=epoch,
        )
        save_checkpoint(
            output_directory / "latest.pt",
            model,
            model_config,
            optimizer,
            step=epoch,
        )


def _load_model_config(checkpoint_path: Path) -> dict[str, Any]:
    """Read the released model configuration retained in fine-tuned checkpoints."""
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    return checkpoint["model_config"]


if __name__ == "__main__":
    main()
