"""Home ontology dataset helpers for closed-set detector fine-tuning."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence


@dataclass(frozen=True)
class HomeSample:
    """One RGB sample with YOLO-style class boxes in normalized xywh."""

    image: Path
    # Each box: class_index, x_center, y_center, width, height in [0, 1].
    boxes: List[List[float]]


class HomeManifestDataset:
    """
    Load a JSONL home-detection manifest.

    Each line is a JSON object:
      {"image":"rel/path.jpg","boxes":[[class_id,xc,yc,w,h], ...]}
    Paths are resolved relative to the manifest directory.
    """

    def __init__(self, manifest: Path, labels: Sequence[str]) -> None:
        self.manifest = Path(manifest)
        self.root = self.manifest.parent
        self.labels = list(labels)
        if not self.labels:
            raise ValueError("labels must not be empty")
        if not self.manifest.is_file():
            raise FileNotFoundError(f"manifest not found: {self.manifest}")
        self.samples = self._load()

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int) -> HomeSample:
        return self.samples[index]

    def _load(self) -> List[HomeSample]:
        samples: List[HomeSample] = []
        with self.manifest.open("r", encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, start=1):
                line = line.strip()
                if not line:
                    continue
                try:
                    record = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise ValueError(
                        f"invalid JSON on line {line_number} of {self.manifest}"
                    ) from exc
                samples.append(self._parse_record(record, line_number))
        if not samples:
            raise ValueError(f"manifest has no samples: {self.manifest}")
        return samples

    def _parse_record(self, record: Dict[str, object], line_number: int) -> HomeSample:
        if not isinstance(record, dict):
            raise ValueError(f"line {line_number}: record must be an object")
        image = record.get("image")
        boxes = record.get("boxes")
        if not isinstance(image, str) or not image:
            raise ValueError(f"line {line_number}: image must be a non-empty string")
        if not isinstance(boxes, list):
            raise ValueError(f"line {line_number}: boxes must be a list")
        parsed_boxes: List[List[float]] = []
        for box in boxes:
            if not isinstance(box, list) or len(box) != 5:
                raise ValueError(
                    f"line {line_number}: each box must be [class, xc, yc, w, h]"
                )
            class_id = int(box[0])
            if class_id < 0 or class_id >= len(self.labels):
                raise ValueError(
                    f"line {line_number}: class_id {class_id} out of range for "
                    f"{len(self.labels)} labels"
                )
            values = [float(class_id)] + [float(v) for v in box[1:]]
            if any(not (0.0 <= v <= 1.0) for v in values[1:]):
                raise ValueError(
                    f"line {line_number}: box coordinates must be normalized to [0, 1]"
                )
            parsed_boxes.append(values)
        return HomeSample(image=self.root / image, boxes=parsed_boxes)


def write_yolo_dataset(
    dataset: HomeManifestDataset,
    output_dir: Path,
    split_name: str = "train",
) -> Path:
    """
    Materialize a Ultralytics-compatible dataset directory and return data.yaml.

    Layout:
      output_dir/
        images/<split_name>/...
        labels/<split_name>/*.txt
        data.yaml
    """
    output_dir = Path(output_dir)
    image_dir = output_dir / "images" / split_name
    label_dir = output_dir / "labels" / split_name
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)

    for index, sample in enumerate(dataset.samples):
        if not sample.image.is_file():
            raise FileNotFoundError(f"image not found: {sample.image}")
        suffix = sample.image.suffix or ".jpg"
        stem = f"{index:06d}"
        target_image = image_dir / f"{stem}{suffix}"
        target_image.write_bytes(sample.image.read_bytes())
        label_path = label_dir / f"{stem}.txt"
        lines = [
            f"{int(box[0])} {box[1]:.6f} {box[2]:.6f} {box[3]:.6f} {box[4]:.6f}"
            for box in sample.boxes
        ]
        label_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")

    yaml_path = output_dir / "data.yaml"
    names_block = "\n".join(f"  {i}: {name}" for i, name in enumerate(dataset.labels))
    yaml_path.write_text(
        "path: .\n"
        f"train: images/{split_name}\n"
        f"val: images/{split_name}\n"
        f"nc: {len(dataset.labels)}\n"
        "names:\n"
        f"{names_block}\n",
        encoding="utf-8",
    )
    return yaml_path
