"""JSONL manifest dataset for RGB-D fine-tuning."""

import json
from pathlib import Path
from typing import Any

import numpy as np
import torch
from PIL import Image
from torch.utils.data import Dataset


class RgbdManifestDataset(Dataset[dict[str, torch.Tensor]]):
    """Load RGB-D samples whose paths are listed relative to a JSONL manifest."""

    def __init__(
        self,
        manifest_path: Path,
        dropout_probability: float,
        depth_scale: float,
        generator: torch.Generator | None = None,
    ) -> None:
        self.manifest_path = Path(manifest_path)
        self.dropout_probability = dropout_probability
        self.depth_scale = depth_scale
        self.generator = generator
        self.records = self._read_manifest()

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, index: int) -> dict[str, torch.Tensor]:
        record = self.records[index]
        image = self._load_rgb(record["rgb"])
        raw_depth = self._load_depth(record["raw_depth"])
        target_depth = self._load_depth(record["target_depth"])

        valid_raw = torch.isfinite(raw_depth) & (raw_depth > 0)
        raw_depth = torch.where(valid_raw, raw_depth, torch.zeros_like(raw_depth))
        if self.dropout_probability:
            keep = torch.rand(raw_depth.shape, generator=self.generator) >= self.dropout_probability
            raw_depth = torch.where(valid_raw & keep, raw_depth, torch.zeros_like(raw_depth))

        sample = {
            "image": image,
            "raw_depth": raw_depth.contiguous(),
            "target_depth": target_depth.contiguous(),
            "valid_mask": (torch.isfinite(target_depth) & (target_depth > 0)).contiguous(),
        }
        if "intrinsics" in record:
            sample["intrinsics"] = torch.tensor(record["intrinsics"], dtype=torch.float32).contiguous()
        return sample

    def _read_manifest(self) -> list[dict[str, Any]]:
        with self.manifest_path.open() as manifest:
            return [json.loads(line) for line in manifest if line.strip()]

    def _load_rgb(self, relative_path: str) -> torch.Tensor:
        with Image.open(self.manifest_path.parent / relative_path) as image:
            rgb = np.array(image.convert("RGB"), copy=True, order="C")
        return torch.from_numpy(rgb).permute(2, 0, 1).contiguous().to(torch.float32).div_(255.0)

    def _load_depth(self, relative_path: str) -> torch.Tensor:
        with Image.open(self.manifest_path.parent / relative_path) as image:
            depth = np.array(image, copy=True, order="C")
        return torch.from_numpy(depth).to(torch.float32).mul_(self.depth_scale)
