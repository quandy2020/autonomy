# Copyright 2026 The Navigation Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Offline teacher labels for the DeFM retraining run.

The DeFM student has to reproduce the *same* two outputs as the official
RGB-D checkpoint, so the cheapest correct supervision is the official model's
own output on the collected frames. The teacher runs once, on the training
host, and never ships: the deployed graph is ``depth -> DeFM -> heads``.

Labels are written as ``<name>.npz`` with

``df``      ``float16 (S, S)``  normalised distance field, ``-log(d / 10)``
``labels``  ``uint8 (S, S)``    arg-max of the 11 gain logits
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np

from frontiernet.postprocess import denormalize_df
from frontiernet.preprocess import frontiernet_preprocess, frontiernet_preprocess_aligned

__all__ = ["TeacherLabeler", "label_frames", "load_official_model", "main"]


def load_official_model(weights: str, classes: int = 11, device: str = "cpu"):
    """Build ``TwoHeadUnet`` with the channel count the checkpoint was trained on."""
    import torch

    from frontiernet.checkpoint import read_state_dict, resolve_weights
    from frontiernet.unet import TwoHeadUnet

    weights = resolve_weights(weights)
    state = read_state_dict(weights)
    weight = state.get("encoder.conv1.weight")
    if weight is None:
        raise KeyError(f"{weights} has no encoder.conv1.weight")
    in_channels = int(weight.shape[1])
    model = TwoHeadUnet(classes=int(classes), in_channels=in_channels)
    model.load_state_dict(state, strict=True)
    return model.to(device).eval()


@dataclass
class TeacherLabeler:
    """Run the official RGB-D checkpoint on captured frames."""

    weights: str
    classes: int = 11
    device: str = "cuda"
    input_size: int = 224
    scale: float = 0.5
    batch_size: int = 8
    geometry: str = "aligned"

    def __post_init__(self) -> None:
        import torch

        self._torch = torch
        self.model = load_official_model(self.weights, self.classes, self.device)
        self.in_channels = int(self.model.encoder.conv1.in_channels)
        if self.in_channels != 4:
            raise ValueError(
                f"teacher expects {self.in_channels} input channels; the RGB-D "
                "checkpoint (4) is required to label depth+colour frames"
            )

    def _prepare(self, rgb: np.ndarray, depth: np.ndarray) -> np.ndarray:
        if self.geometry == "aligned":
            return frontiernet_preprocess_aligned(
                np.asarray(rgb, dtype=np.float32),
                np.asarray(depth, dtype=np.float32),
                size=self.input_size,
            )
        if self.geometry == "official":
            return frontiernet_preprocess(
                np.asarray(rgb, dtype=np.float32) / 255.0,
                np.asarray(depth, dtype=np.float32),
                scale=self.scale,
                size=self.input_size,
            )
        raise ValueError(f"unknown geometry {self.geometry!r}")

    def predict(self, rgb: np.ndarray, depth: np.ndarray):
        """Return ``(df, logits)`` numpy arrays for one frame."""
        torch = self._torch
        tensor = torch.from_numpy(self._prepare(rgb, depth)).unsqueeze(0)
        with torch.no_grad():
            df, logits = self.model(tensor.to(self.device, dtype=torch.float32))
        return (
            df.detach().float().cpu().numpy(),
            logits.detach().float().cpu().numpy(),
        )

    def predict_batch(self, frames: Sequence[Dict[str, np.ndarray]]):
        """Return ``(df, logits)`` for a batch of ``{"rgb", "depth"}`` dicts."""
        torch = self._torch
        batch = np.stack([self._prepare(frame["rgb"], frame["depth"]) for frame in frames])
        tensor = torch.from_numpy(batch).to(self.device, dtype=torch.float32)
        with torch.no_grad():
            df, logits = self.model(tensor)
        return (
            df.detach().float().cpu().numpy(),
            logits.detach().float().cpu().numpy(),
        )


def label_frames(
    frames_dir: str,
    out_dir: str,
    weights: str,
    *,
    classes: int = 11,
    device: str = "cuda",
    input_size: int = 224,
    scale: float = 0.5,
    batch_size: int = 8,
    limit: Optional[int] = None,
    overwrite: bool = False,
    df_threshold_m: float = 0.5,
    normalizer: float = 10.0,
    geometry: str = "aligned",
) -> Dict[str, object]:
    """Label every frame in ``frames_dir`` and return a summary dict."""
    if not os.path.isdir(frames_dir):
        raise FileNotFoundError(f"frames directory not found: {frames_dir}")
    os.makedirs(out_dir, exist_ok=True)
    names = sorted(
        os.path.splitext(entry)[0]
        for entry in os.listdir(frames_dir)
        if entry.endswith(".npz")
    )
    if limit:
        names = names[: int(limit)]
    if not names:
        raise FileNotFoundError(f"no .npz frames in {frames_dir}")

    labeler = TeacherLabeler(
        weights=weights,
        classes=classes,
        device=device,
        input_size=input_size,
        scale=scale,
        batch_size=batch_size,
        geometry=geometry,
    )

    histogram = np.zeros(int(classes), dtype=np.int64)
    frontier_pixels: List[int] = []
    df_min = float("inf")
    df_max = float("-inf")
    written = 0
    skipped = 0

    for start in range(0, len(names), int(batch_size)):
        chunk = names[start : start + int(batch_size)]
        pending = []
        for name in chunk:
            destination = os.path.join(out_dir, f"{name}.npz")
            if os.path.isfile(destination) and not overwrite:
                skipped += 1
                continue
            with np.load(os.path.join(frames_dir, f"{name}.npz")) as payload:
                if "rgb" not in payload:
                    raise KeyError(f"{name}.npz has no rgb; the teacher needs RGB-D")
                pending.append(
                    (
                        name,
                        {
                            "rgb": np.asarray(payload["rgb"]),
                            "depth": np.asarray(payload["depth"], dtype=np.float32),
                        },
                    )
                )
        if not pending:
            continue
        df_batch, logits_batch = labeler.predict_batch([frame for _, frame in pending])
        for offset, (name, _) in enumerate(pending):
            df = df_batch[offset, 0].astype(np.float32)
            labels = np.argmax(logits_batch[offset], axis=0).astype(np.uint8)
            metres = denormalize_df(df, normalizer)
            count = int((metres < float(df_threshold_m)).sum())
            np.savez_compressed(
                os.path.join(out_dir, f"{name}.npz"),
                df=df.astype(np.float16),
                labels=labels,
            )
            histogram += np.bincount(labels.reshape(-1), minlength=int(classes))[: int(classes)]
            frontier_pixels.append(count)
            df_min = min(df_min, float(df.min()))
            df_max = max(df_max, float(df.max()))
            written += 1

    summary = {
        "frames_dir": os.path.abspath(frames_dir),
        "labels_dir": os.path.abspath(out_dir),
        "weights": os.path.abspath(weights),
        "written": written,
        "skipped_existing": skipped,
        "input_size": int(input_size),
        "geometry": geometry,
        "class_histogram": histogram.tolist(),
        "frontier_pixels_mean": float(np.mean(frontier_pixels)) if frontier_pixels else 0.0,
        "frontier_pixels_min": int(np.min(frontier_pixels)) if frontier_pixels else 0,
        "frontier_pixels_max": int(np.max(frontier_pixels)) if frontier_pixels else 0,
        "df_min": df_min if frontier_pixels else 0.0,
        "df_max": df_max if frontier_pixels else 0.0,
    }
    with open(os.path.join(out_dir, "labels.json"), "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
    return summary


def main(argv: Optional[Sequence[str]] = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(
        prog="frontiernet.label", description="teacher labels for captured frames"
    )
    parser.add_argument("--frames", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--weights", required=True)
    parser.add_argument("--classes", type=int, default=11)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--input-size", type=int, default=224)
    parser.add_argument("--scale", type=float, default=0.5)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--df-threshold", type=float, default=0.5)
    parser.add_argument("--df-normalizer", type=float, default=10.0)
    parser.add_argument("--geometry", choices=("aligned", "official"), default="aligned")
    args = parser.parse_args(argv)
    summary = label_frames(
        args.frames,
        args.out,
        args.weights,
        classes=args.classes,
        device=args.device,
        input_size=args.input_size,
        scale=args.scale,
        batch_size=args.batch_size,
        limit=args.limit,
        overwrite=args.overwrite,
        df_threshold_m=args.df_threshold,
        normalizer=args.df_normalizer,
        geometry=args.geometry,
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
