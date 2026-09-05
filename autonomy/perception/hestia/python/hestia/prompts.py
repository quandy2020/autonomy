"""Prompt / home-label helpers aligned with C++ HestiaOptions lists."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable, List, Sequence


def load_label_list(path: Path) -> List[str]:
    """Load an ordered label list from JSON (list) or one-label-per-line text."""
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        raise ValueError(f"label file is empty: {path}")
    if path.suffix.lower() == ".json":
        data = json.loads(text)
        if not isinstance(data, list) or not all(isinstance(x, str) for x in data):
            raise ValueError(f"JSON label file must be a list of strings: {path}")
        labels = [item.strip() for item in data]
    else:
        labels = [line.strip() for line in text.splitlines() if line.strip()]
    if not labels:
        raise ValueError(f"label file has no entries: {path}")
    if any(not label for label in labels):
        raise ValueError(f"label file contains an empty entry: {path}")
    return labels


def write_label_sidecar(path: Path, labels: Sequence[str]) -> Path:
    """Write ordered labels next to an ONNX artifact for deployment checks."""
    if not labels:
        raise ValueError("labels must not be empty")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(list(labels), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return path


def assert_same_order(expected: Sequence[str], actual: Iterable[str], name: str) -> None:
    """Require export-time class order to match configured prompts/labels."""
    actual_list = list(actual)
    if list(expected) != actual_list:
        raise ValueError(
            f"{name} order mismatch: expected {list(expected)!r}, got {actual_list!r}"
        )
