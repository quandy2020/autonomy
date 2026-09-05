"""Closed-set home detector fine-tuning via an installed Ultralytics package."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Callable, Optional, Sequence

from .prompts import load_label_list

ULTRALYTICS_LICENSE_HINT = (
    "Hestia home training requires an installed ultralytics package. "
    "Ultralytics YOLO is typically AGPL-3.0 or commercial; deployers must "
    "choose terms compatible with their application. Hestia does not vendor "
    "Ultralytics or model weights."
)


def _default_factory(checkpoint: str) -> Any:
    try:
        from ultralytics import YOLO  # type: ignore
    except ImportError as exc:  # pragma: no cover - exercised in tests via message
        raise ImportError(ULTRALYTICS_LICENSE_HINT) from exc
    return YOLO(checkpoint)


def train_home_detector(
    checkpoint: str,
    data: str,
    output: Path,
    epochs: int,
    image_size: int,
    batch_size: int,
    *,
    device: str = "cpu",
    workers: int = 0,
    seed: int = 0,
    class_indices: Optional[Sequence[int]] = None,
    factory: Optional[Callable[[str], Any]] = None,
) -> Path:
    """
    Fine-tune a closed-set detector and return the best checkpoint path.

    `factory` injects a fake YOLO object in tests. Production uses installed
    Ultralytics and never vendors its sources.
    """
    if epochs <= 0:
        raise ValueError("epochs must be positive")
    if image_size <= 0 or batch_size <= 0:
        raise ValueError("image_size and batch_size must be positive")

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    create = factory or _default_factory
    model = create(checkpoint)

    train_kwargs: dict[str, Any] = {
        "data": data,
        "epochs": epochs,
        "imgsz": image_size,
        "batch": batch_size,
        "device": device,
        "workers": workers,
        "seed": seed,
        "project": str(output),
        "name": "home",
        "exist_ok": True,
    }
    if class_indices is not None:
        train_kwargs["classes"] = list(class_indices)

    model.train(**train_kwargs)

    best = output / "home" / "weights" / "best.pt"
    last = output / "home" / "weights" / "last.pt"
    if best.is_file():
        return best
    if last.is_file():
        return last
    # Fake factories may place a marker file directly under output.
    marker = output / "best.pt"
    if marker.is_file():
        return marker
    raise RuntimeError(
        f"training finished but no checkpoint was found under {output / 'home' / 'weights'}"
    )


def main(argv: Optional[Sequence[str]] = None) -> None:
    """CLI entry for home-ontology fine-tuning."""
    parser = argparse.ArgumentParser(
        description="Fine-tune a Hestia home closed-set detector with Ultralytics"
    )
    parser.add_argument("--checkpoint", required=True, help="Base YOLO checkpoint")
    parser.add_argument(
        "--data",
        required=True,
        help="Ultralytics data.yaml (use hestia.dataset.write_yolo_dataset to build)",
    )
    parser.add_argument("--output", required=True, help="Training project directory")
    parser.add_argument("--labels", help="Optional JSON/text label list for documentation")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)

    if args.labels:
        load_label_list(Path(args.labels))

    best = train_home_detector(
        args.checkpoint,
        args.data,
        Path(args.output),
        args.epochs,
        args.image_size,
        args.batch_size,
        device=args.device,
        workers=args.workers,
        seed=args.seed,
    )
    print(best)


if __name__ == "__main__":
    main()
