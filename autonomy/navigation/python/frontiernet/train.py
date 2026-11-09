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

"""Retrain the FrontierNet decoder + heads on top of a frozen DeFM encoder.

The recipe is deliberately small: the ViT stays frozen, the adapter and the
official decoder/heads are trained to reproduce the teacher labels produced by
:mod:`frontiernet.label`. One epoch over a few hundred autosim frames is enough
to turn the previously empty frontier map into a usable one, because the
student starts from the teacher's own decoder weights.

Usage (inside ``SpaceHero``)::

    /opt/venv/bin/python -m frontiernet.train \
        --config configs/train/frontiernet_defm.yaml \
        --frames data/frontiernet/frames \
        --labels data/frontiernet/labels \
        --out data/frontiernet/run
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import time
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from frontiernet.version import __version__

__all__ = ["TrainSettings", "load_settings", "main", "train"]


@dataclass
class TrainSettings:
    """Everything one retraining run needs."""

    frames: str
    labels: str
    out: str
    official_weights: str
    defm_weights: Optional[str] = None
    epochs: int = 8
    batch_size: int = 4
    input_size: int = 224
    head_lr: float = 3e-4
    adapter_lr: float = 1e-3
    backbone_lr: float = 0.0
    weight_decay: float = 1e-4
    warmup_ratio: float = 0.05
    grad_clip: float = 1.0
    amp: bool = True
    device: str = "cuda"
    workers: int = 2
    val_fraction: float = 0.15
    seed: int = 0
    log_every: int = 10
    max_steps: int = 0
    flip: bool = True
    freeze_backbone: bool = True
    df_weight: float = 1.0
    seg_weight: float = 1.0
    frontier_boost: float = 4.0
    class_weight_power: float = 0.5
    resume: Optional[str] = None
    classes: int = 11
    df_normalizer: float = 10.0
    region_threshold_m: float = 0.5
    extra: Dict[str, object] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, object]:
        payload = asdict(self)
        payload.pop("extra", None)
        return payload


def _seed_everything(seed: int) -> None:
    import torch

    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def _build_model(settings: TrainSettings):
    import torch

    from frontiernet.checkpoint import load_frontiernet_heads, resolve_weights
    from frontiernet.model import FrontierNetDeFM

    defm_weights = (
        resolve_weights(settings.defm_weights) if settings.defm_weights else None
    )
    model = FrontierNetDeFM(
        classes=settings.classes,
        defm_weights=defm_weights,
        input_size=settings.input_size,
        freeze_backbone=settings.freeze_backbone,
    )
    report = load_frontiernet_heads(model, resolve_weights(settings.official_weights))
    if not report.ok:
        raise RuntimeError(f"head transplant failed: {report.summary()}")
    if settings.backbone_lr > 0.0:
        model.set_backbone_frozen(False)
    return model, report


def _build_optimizer(model, settings: TrainSettings):
    import torch

    groups = model.parameter_groups(
        head_lr=settings.head_lr,
        adapter_lr=settings.adapter_lr,
        backbone_lr=settings.backbone_lr if settings.backbone_lr > 0.0 else None,
    )
    if not groups:
        raise RuntimeError("no trainable parameters; check the freeze policy")
    return torch.optim.AdamW(
        groups, lr=settings.head_lr, weight_decay=settings.weight_decay
    )


def _build_scheduler(optimizer, total_steps: int, warmup_ratio: float):
    import torch

    warmup = max(1, int(total_steps * float(warmup_ratio)))

    def factor(step: int) -> float:
        if step < warmup:
            return float(step + 1) / float(warmup)
        progress = float(step - warmup) / float(max(1, total_steps - warmup))
        return 0.5 * (1.0 + math.cos(math.pi * min(1.0, progress)))

    return torch.optim.lr_scheduler.LambdaLR(optimizer, factor)


def _autocast(settings: TrainSettings):
    import contextlib

    import torch

    if settings.amp and settings.device.startswith("cuda"):
        return torch.amp.autocast("cuda", dtype=torch.bfloat16)
    return contextlib.nullcontext()


def _evaluate(model, loader, criterion, settings: TrainSettings) -> Dict[str, float]:
    import torch

    from frontiernet.loss import df_to_metres

    model.eval()
    totals = {"loss": 0.0, "df": 0.0, "seg": 0.0, "df_mae_m": 0.0, "label_acc": 0.0}
    intersection = 0.0
    union = 0.0
    seen = 0
    with torch.no_grad():
        for batch in loader:
            inputs = batch["input"].to(settings.device, non_blocking=True)
            df_target = batch["df"].to(settings.device, non_blocking=True)
            labels = batch["label"].to(settings.device, non_blocking=True)
            with _autocast(settings):
                df_pred, logits = model(inputs)
                output = criterion(df_pred, logits, df_target, labels)
            count = int(inputs.shape[0])
            seen += count
            for key in totals:
                totals[key] += float(output["metrics"][key]) * count
            pred_region = df_to_metres(df_pred.float(), settings.df_normalizer) < settings.region_threshold_m
            target_region = df_to_metres(df_target.float(), settings.df_normalizer) < settings.region_threshold_m
            intersection += float((pred_region & target_region).sum())
            union += float((pred_region | target_region).sum())
    metrics = {key: value / max(1, seen) for key, value in totals.items()}
    metrics["region_iou"] = 1.0 if union == 0.0 else intersection / union
    metrics["frames"] = float(seen)
    return metrics


def train(settings: TrainSettings, model=None) -> Dict[str, object]:
    """Run the retraining loop and return a summary dict.

    ``model`` is injectable so tests can pass a stand-in with a fake backbone
    instead of instantiating the real ViT.
    """
    import torch
    from torch.utils.data import DataLoader

    from frontiernet.checkpoint import resolve_weights
    from frontiernet.dataset import FrontierFrameDataset, split_indices
    from frontiernet.loss import FrontierLoss, LossWeights, class_weights_from_histogram

    if settings.device.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is not available")
    _seed_everything(settings.seed)
    os.makedirs(settings.out, exist_ok=True)

    full = FrontierFrameDataset(
        settings.frames, settings.labels, input_size=settings.input_size
    )
    train_index, val_index = split_indices(len(full), settings.val_fraction, settings.seed)
    train_set = FrontierFrameDataset(
        settings.frames,
        settings.labels,
        indices=train_index,
        input_size=settings.input_size,
        flip=settings.flip,
        seed=settings.seed,
    )
    val_set = (
        FrontierFrameDataset(
            settings.frames,
            settings.labels,
            indices=val_index,
            input_size=settings.input_size,
        )
        if val_index
        else None
    )

    histogram = full.label_histogram(settings.classes)
    class_weights = class_weights_from_histogram(histogram, settings.class_weight_power)

    if model is None:
        if not os.path.isfile(resolve_weights(settings.official_weights)):
            raise FileNotFoundError(
                f"official checkpoint not found: {settings.official_weights}"
            )
        model, report = _build_model(settings)
    else:
        report = None
    model.to(settings.device)
    criterion = FrontierLoss(
        LossWeights(
            df=settings.df_weight,
            seg=settings.seg_weight,
            frontier_boost=settings.frontier_boost,
            df_normalizer=settings.df_normalizer,
            region_threshold_m=settings.region_threshold_m,
        ),
        class_weights=class_weights,
    ).to(settings.device)
    optimizer = _build_optimizer(model, settings)

    steps_per_epoch = max(1, math.ceil(len(train_set) / settings.batch_size))
    planned = steps_per_epoch * max(1, settings.epochs)
    if settings.max_steps > 0:
        planned = min(planned, settings.max_steps)
    scheduler = _build_scheduler(optimizer, planned, settings.warmup_ratio)

    generator = torch.Generator()
    generator.manual_seed(settings.seed)
    train_loader = DataLoader(
        train_set,
        batch_size=settings.batch_size,
        shuffle=True,
        num_workers=settings.workers,
        pin_memory=settings.device.startswith("cuda"),
        drop_last=False,
        generator=generator,
    )
    val_loader = (
        DataLoader(
            val_set,
            batch_size=settings.batch_size,
            shuffle=False,
            num_workers=settings.workers,
            pin_memory=settings.device.startswith("cuda"),
        )
        if val_set is not None
        else None
    )

    start_step = 0
    if settings.resume:
        payload = torch.load(settings.resume, map_location="cpu")
        model.load_state_dict(payload["state_dict"], strict=True)
        if "optimizer" in payload:
            optimizer.load_state_dict(payload["optimizer"])
        if "scheduler" in payload:
            scheduler.load_state_dict(payload["scheduler"])
        start_step = int(payload.get("step", 0))

    log_path = os.path.join(settings.out, "train_log.jsonl")
    log_handle = open(log_path, "a", encoding="utf-8")
    best = {"region_iou": -1.0, "loss": float("inf"), "epoch": -1}
    history: List[Dict[str, float]] = []
    step = start_step
    started = time.time()
    stopped_early = False

    for epoch in range(settings.epochs):
        model.train()
        if settings.freeze_backbone:
            model.encoder.backbone.eval()
        running = {"loss": 0.0, "df": 0.0, "seg": 0.0}
        batches = 0
        for batch in train_loader:
            if settings.max_steps > 0 and step >= settings.max_steps:
                stopped_early = True
                break
            inputs = batch["input"].to(settings.device, non_blocking=True)
            df_target = batch["df"].to(settings.device, non_blocking=True)
            labels = batch["label"].to(settings.device, non_blocking=True)
            with _autocast(settings):
                df_pred, logits = model(inputs)
                output = criterion(df_pred, logits, df_target, labels)
            loss = output["loss"]
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            if settings.grad_clip > 0:
                torch.nn.utils.clip_grad_norm_(model.trainable_parameters(), settings.grad_clip)
            optimizer.step()
            scheduler.step()
            step += 1
            batches += 1
            for key in running:
                running[key] += float(output[key].detach())
            if step % max(1, settings.log_every) == 0:
                record = {
                    "event": "step",
                    "step": step,
                    "epoch": epoch,
                    "loss": running["loss"] / batches,
                    "df": running["df"] / batches,
                    "seg": running["seg"] / batches,
                    "df_mae_m": float(output["metrics"]["df_mae_m"]),
                    "region_iou": float(output["metrics"]["region_iou"]),
                    "label_acc": float(output["metrics"]["label_acc"]),
                    "lr": optimizer.param_groups[-1]["lr"],
                    "elapsed_s": round(time.time() - started, 1),
                }
                log_handle.write(json.dumps(record) + "\n")
                log_handle.flush()
                print(
                    f"step {step:5d} loss {record['loss']:.4f} df {record['df']:.4f} "
                    f"seg {record['seg']:.4f} iou {record['region_iou']:.3f} "
                    f"acc {record['label_acc']:.3f}",
                    flush=True,
                )
                running = {key: 0.0 for key in running}
                batches = 0
        if stopped_early:
            break

        metrics = (
            _evaluate(model, val_loader, criterion, settings)
            if val_loader is not None
            else {}
        )
        record = {"event": "epoch", "epoch": epoch, "step": step, **metrics}
        history.append(record)
        log_handle.write(json.dumps(record) + "\n")
        log_handle.flush()
        print(
            f"epoch {epoch} val loss {metrics.get('loss', float('nan')):.4f} "
            f"iou {metrics.get('region_iou', float('nan')):.3f} "
            f"df_mae {metrics.get('df_mae_m', float('nan')):.3f}m",
            flush=True,
        )

        state = {
            "state_dict": model.state_dict(),
            "optimizer": optimizer.state_dict(),
            "scheduler": scheduler.state_dict(),
            "step": step,
            "epoch": epoch,
            "settings": settings.to_dict(),
            "version": __version__,
        }
        torch.save(state, os.path.join(settings.out, "last.pth"))
        score = metrics.get("region_iou", 0.0)
        if score > best["region_iou"] or (
            score == best["region_iou"] and metrics.get("loss", 1e9) < best["loss"]
        ):
            best = {
                "region_iou": score,
                "loss": metrics.get("loss", float("inf")),
                "epoch": epoch,
            }
            torch.save(state, os.path.join(settings.out, "best.pth"))

    log_handle.close()
    if not os.path.isfile(os.path.join(settings.out, "last.pth")):
        state = {
            "state_dict": model.state_dict(),
            "optimizer": optimizer.state_dict(),
            "scheduler": scheduler.state_dict(),
            "step": step,
            "epoch": -1,
            "settings": settings.to_dict(),
            "version": __version__,
        }
        torch.save(state, os.path.join(settings.out, "last.pth"))

    summary = {
        "version": __version__,
        "out": os.path.abspath(settings.out),
        "device": settings.device,
        "frames_total": len(full),
        "frames_train": len(train_set),
        "frames_val": len(val_set) if val_set is not None else 0,
        "steps": step,
        "epochs": settings.epochs,
        "batch_size": settings.batch_size,
        "elapsed_s": round(time.time() - started, 1),
        "head_transplant": report.summary() if report is not None else "injected",
        "class_histogram": histogram.tolist(),
        "class_weights": [round(float(value), 4) for value in class_weights],
        "parameters": model.describe(),
        "best": best,
        "history": history,
        "checkpoints": {
            "last": os.path.join(settings.out, "last.pth"),
            "best": os.path.join(settings.out, "best.pth")
            if os.path.isfile(os.path.join(settings.out, "best.pth"))
            else None,
        },
        "log": log_path,
    }
    with open(os.path.join(settings.out, "summary.json"), "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
    return summary


def load_settings(path: str) -> Dict[str, object]:
    """Read a YAML config into a plain dict (no extra dependency)."""
    import yaml

    with open(path, "r", encoding="utf-8") as handle:
        payload = yaml.safe_load(handle) or {}
    if not isinstance(payload, dict):
        raise TypeError(f"{path} must contain a mapping")
    return payload


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="frontiernet.train", description="retrain FrontierNet heads over DeFM"
    )
    parser.add_argument("--config", default=None, help="YAML defaults")
    parser.add_argument("--frames", default=None)
    parser.add_argument("--labels", default=None)
    parser.add_argument("--out", default=None)
    parser.add_argument("--official-weights", default=None)
    parser.add_argument("--defm-weights", default=None)
    parser.add_argument("--epochs", type=int, default=None)
    parser.add_argument("--batch-size", type=int, default=None)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--head-lr", type=float, default=None)
    parser.add_argument("--adapter-lr", type=float, default=None)
    parser.add_argument("--backbone-lr", type=float, default=None)
    parser.add_argument("--device", default=None)
    parser.add_argument("--workers", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--val-fraction", type=float, default=None)
    parser.add_argument("--log-every", type=int, default=None)
    parser.add_argument("--df-weight", type=float, default=None)
    parser.add_argument("--seg-weight", type=float, default=None)
    parser.add_argument("--frontier-boost", type=float, default=None)
    parser.add_argument("--class-weight-power", type=float, default=None)
    parser.add_argument("--no-amp", action="store_true")
    parser.add_argument("--no-flip", action="store_true")
    parser.add_argument("--resume", default=None)
    args = parser.parse_args(argv)

    payload: Dict[str, object] = load_settings(args.config) if args.config else {}
    overrides = {
        "frames": args.frames,
        "labels": args.labels,
        "out": args.out,
        "official_weights": args.official_weights,
        "defm_weights": args.defm_weights,
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "max_steps": args.max_steps,
        "head_lr": args.head_lr,
        "adapter_lr": args.adapter_lr,
        "backbone_lr": args.backbone_lr,
        "device": args.device,
        "workers": args.workers,
        "seed": args.seed,
        "val_fraction": args.val_fraction,
        "log_every": args.log_every,
        "df_weight": args.df_weight,
        "seg_weight": args.seg_weight,
        "frontier_boost": args.frontier_boost,
        "class_weight_power": args.class_weight_power,
        "resume": args.resume,
    }
    for key, value in overrides.items():
        if value is not None:
            payload[key] = value
    if args.no_amp:
        payload["amp"] = False
    if args.no_flip:
        payload["flip"] = False

    known = {field_name for field_name in TrainSettings.__dataclass_fields__}
    unknown = sorted(set(payload) - known)
    if unknown:
        raise SystemExit(f"unknown config keys: {unknown}")
    settings = TrainSettings(**payload)  # type: ignore[arg-type]
    summary = train(settings)
    print(json.dumps({key: summary[key] for key in ("steps", "best", "elapsed_s", "out")}, indent=2))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
