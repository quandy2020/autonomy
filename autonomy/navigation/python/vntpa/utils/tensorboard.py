# Copyright 2026 The Navigation Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""JSONL + TensorBoard event logger used by train and data analysis."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping, Optional

import numpy as np


def describe(values: np.ndarray) -> dict[str, float]:
    """Finite-sample mean/std/min/max/p50/p95; empty arrays become zeros."""
    flat = np.asarray(values, dtype=np.float64).reshape(-1)
    flat = flat[np.isfinite(flat)]
    if flat.size == 0:
        return {key: 0.0 for key in ("count", "mean", "std", "min", "max", "p50", "p95")}
    return {
        "count": float(flat.size),
        "mean": float(flat.mean()),
        "std": float(flat.std()),
        "min": float(flat.min()),
        "max": float(flat.max()),
        "p50": float(np.percentile(flat, 50)),
        "p95": float(np.percentile(flat, 95)),
    }


_SERIES_KINDS = frozenset({"train", "val"})


class EventLogger:
    """Write ``metrics.jsonl`` always; TensorBoard when the extra is installed."""

    def __init__(self, run_dir: str | Path, tensorboard_dir: Optional[str] = None) -> None:
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.jsonl = self.run_dir / "metrics.jsonl"
        self.writer = None
        destination = tensorboard_dir
        if destination is None:
            destination = str(self.run_dir / "tb")
        if destination:
            try:
                from torch.utils.tensorboard import SummaryWriter

                Path(destination).mkdir(parents=True, exist_ok=True)
                self.writer = SummaryWriter(destination)
            except Exception:
                self.writer = None
        self.tensorboard_dir = destination if self.writer is not None else ""

    def log(self, step: int, kind: str, **values: Any) -> None:
        """Print, append JSONL, and write TensorBoard.

        ``train`` / ``val`` become step series. Corpus snapshots stay text plus a
        handful of scalars (no ``a/b/c`` tag trees that fill the Time series home
        page with single-point empty charts).
        """
        record = {"kind": kind, "step": int(step), **values}
        print(json.dumps(record, default=_json_default), flush=True)
        with self.jsonl.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, default=_json_default) + "\n")
        if self.writer is None:
            return
        series = kind in _SERIES_KINDS
        if series:
            for tag, value in _flatten(kind, values):
                self._scalar(tag, value, step)
        else:
            for key, value in values.items():
                if not isinstance(value, (Mapping, list, tuple)):
                    self._scalar(f"{kind}/{key}", value, step)
            self.writer.add_text(
                f"{kind}/snapshot", json.dumps(values, indent=2, default=_json_default), step
            )
        self.writer.flush()

    def histogram(self, tag: str, values: np.ndarray, step: int) -> None:
        if self.writer is None:
            return
        flat = np.asarray(values, dtype=np.float32).reshape(-1)
        if flat.size == 0:
            return
        self.writer.add_histogram(tag, flat, step)
        self.writer.flush()

    def image(self, tag: str, image: np.ndarray, step: int) -> None:
        if self.writer is None:
            return
        self.writer.add_image(tag, np.asarray(image), step, dataformats="CHW")
        self.writer.flush()

    def text(self, tag: str, text: str, step: int = 0) -> None:
        if self.writer is None:
            return
        self.writer.add_text(tag, text, step)
        self.writer.flush()

    def hparams(self, params: Mapping[str, Any], metrics: Mapping[str, float]) -> None:
        """Log hyper-parameters as text. Avoid ``add_hparams`` (it creates an empty extra run)."""
        if self.writer is None:
            return
        payload = {**{key: _hparams_value(value) for key, value in params.items()}, **dict(metrics)}
        self.writer.add_text("hparams", json.dumps(payload, indent=2, default=_json_default), 0)
        for key, value in metrics.items():
            self._scalar(f"val/{key.split('/')[-1]}" if "/" in key else f"hparams/{key}", value, 0)
        self.writer.flush()

    def close(self) -> None:
        if self.writer is not None:
            self.writer.close()
            self.writer = None

    def _scalar(self, tag: str, value: Any, step: int) -> None:
        try:
            self.writer.add_scalar(tag, float(value), step)
        except (TypeError, ValueError):
            pass


def _flatten(kind: str, values: Mapping[str, Any]) -> list[tuple[str, Any]]:
    """``train/loss`` or ``data/frames_mean`` — never ``data/frames/mean``."""
    tags: list[tuple[str, Any]] = []
    for key, value in values.items():
        if isinstance(value, (list, tuple)):
            for index, item in enumerate(value):
                tags.append((f"{kind}/{key}_{index}", item))
        elif isinstance(value, Mapping):
            for nested, item in value.items():
                tags.append((f"{kind}/{key}_{nested}", item))
        else:
            tags.append((f"{kind}/{key}", value))
    return tags


def write_curves_from_jsonl(jsonl_path: str | Path, out_dir: str | Path) -> int:
    """Rebuild a TensorBoard run that only contains train/val step series."""
    from torch.utils.tensorboard import SummaryWriter

    path = Path(jsonl_path)
    destination = Path(out_dir)
    destination.mkdir(parents=True, exist_ok=True)
    written = 0
    writer = SummaryWriter(str(destination))
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            kind = str(record.get("kind", ""))
            step = int(record.get("step", 0))
            if kind not in _SERIES_KINDS:
                continue
            payload = {key: value for key, value in record.items() if key not in ("kind", "step")}
            for tag, value in _flatten(kind, payload):
                try:
                    writer.add_scalar(tag, float(value), step)
                    written += 1
                except (TypeError, ValueError):
                    pass
    writer.flush()
    writer.close()
    return written


def _json_default(value: Any) -> Any:
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return str(value)


def _hparams_value(value: Any) -> Any:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float, str)):
        return value
    return str(value)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Rebuild TensorBoard curves from metrics.jsonl")
    parser.add_argument("--jsonl", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    count = write_curves_from_jsonl(args.jsonl, args.out)
    print(json.dumps({"scalars": count, "out": args.out}))
