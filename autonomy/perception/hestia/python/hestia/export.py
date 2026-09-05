"""Fixed-profile ONNX export matching Hestia C++ detector contracts."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Callable, Optional, Sequence

from .prompts import load_label_list, write_label_sidecar
from .train_home import ULTRALYTICS_LICENSE_HINT, _default_factory

# Must stay aligned with autonomy/perception/hestia/detector.cpp
INPUT_NAME = "images"
OUTPUT_NAME = "output0"


def export_detector(
    checkpoint: str,
    output: Path,
    image_size: int,
    max_detections: int,
    labels: Sequence[str],
    *,
    factory: Optional[Callable[[str], Any]] = None,
) -> Path:
    """
    Export a closed-set home detector to fixed-profile ONNX.

    Expected C++ tensors:
      images  float32 [1, 3, image_size, image_size]
      output0 float32 [1, max_detections, 6]
    """
    return _export(
        checkpoint,
        output,
        image_size,
        max_detections,
        labels,
        sidecar_stem="home_labels",
        factory=factory,
    )


def export_open_detector(
    checkpoint: str,
    output: Path,
    image_size: int,
    max_detections: int,
    prompts: Sequence[str],
    *,
    factory: Optional[Callable[[str], Any]] = None,
) -> Path:
    """
    Export an already-trained open-vocabulary checkpoint to fixed-profile ONNX.

    v1 does not train open-vocab models in-tree; this only freezes a profile and
    writes the prompt sidecar that must match `HestiaOptions.open_prompts`.
    """
    return _export(
        checkpoint,
        output,
        image_size,
        max_detections,
        prompts,
        sidecar_stem="open_prompts",
        factory=factory,
    )


def _export(
    checkpoint: str,
    output: Path,
    image_size: int,
    max_detections: int,
    labels: Sequence[str],
    *,
    sidecar_stem: str,
    factory: Optional[Callable[[str], Any]],
) -> Path:
    if image_size <= 0 or max_detections <= 0:
        raise ValueError("image_size and max_detections must be positive")
    if not labels:
        raise ValueError("labels/prompts must not be empty")

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    create = factory or _default_factory
    model = create(checkpoint)

    export_kwargs = {
        "format": "onnx",
        "imgsz": image_size,
        "batch": 1,
        "max_det": max_detections,
        "dynamic": False,
        "simplify": True,
        "opset": 17,
    }
    result = model.export(**export_kwargs)

    exported = Path(str(result)) if result is not None else output
    if not exported.is_file():
        # Fake exporters may write directly to `output`.
        if output.is_file():
            exported = output
        else:
            raise RuntimeError(
                f"export finished but ONNX file was not found (got {result!r})"
            )
    if exported.resolve() != output.resolve():
        output.write_bytes(exported.read_bytes())
        exported = output

    write_label_sidecar(output.with_name(f"{output.stem}_{sidecar_stem}.json"), labels)
    meta = {
        "input_name": INPUT_NAME,
        "output_name": OUTPUT_NAME,
        "image_size": image_size,
        "max_detections": max_detections,
        "layout": "[1, max_detections, 6] = x1,y1,x2,y2,score,class_index",
        "labels": list(labels),
    }
    output.with_suffix(".meta.json").write_text(
        json.dumps(meta, indent=2) + "\n", encoding="utf-8"
    )
    return exported


def main(argv: Optional[Sequence[str]] = None) -> None:
    """CLI for fixed-profile Hestia detector export."""
    parser = argparse.ArgumentParser(
        description="Export a Hestia detector ONNX profile for C++ deployment"
    )
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output", required=True, help="Destination .onnx path")
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--max-detections", type=int, default=100)
    parser.add_argument(
        "--labels",
        required=True,
        help="Ordered JSON/text labels (home) or prompts (open)",
    )
    parser.add_argument(
        "--kind",
        choices=("home", "open"),
        default="home",
        help="home = closed-set export; open = open-vocab checkpoint wrap",
    )
    args = parser.parse_args(argv)

    labels = load_label_list(Path(args.labels))
    if args.kind == "home":
        path = export_detector(
            args.checkpoint,
            Path(args.output),
            args.image_size,
            args.max_detections,
            labels,
        )
    else:
        path = export_open_detector(
            args.checkpoint,
            Path(args.output),
            args.image_size,
            args.max_detections,
            labels,
        )
    print(path)
    print(ULTRALYTICS_LICENSE_HINT)


if __name__ == "__main__":
    main()
