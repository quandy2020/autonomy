"""Tests for fixed-profile Hestia ONNX export facade."""

from pathlib import Path
from typing import Any, Dict, Optional

import pytest

from hestia.export import INPUT_NAME, OUTPUT_NAME, export_detector, export_open_detector


class FakeExportYolo:
    def __init__(self, write_to: Optional[Path] = None) -> None:
        self.export_kwargs: Dict[str, Any] = {}
        self.write_to = write_to

    def export(self, **kwargs: Any) -> str:
        self.export_kwargs = kwargs
        assert self.write_to is not None
        self.write_to.write_bytes(b"onnx-bytes")
        return str(self.write_to)


def test_export_detector_writes_onnx_and_sidecar(tmp_path: Path) -> None:
    onnx_path = tmp_path / "hestia_home.onnx"
    fake = FakeExportYolo(write_to=tmp_path / "raw.onnx")
    result = export_detector(
        "best.pt",
        onnx_path,
        image_size=640,
        max_detections=100,
        labels=["chair", "cup"],
        factory=lambda _: fake,
    )
    assert result == onnx_path
    assert onnx_path.is_file()
    assert fake.export_kwargs["format"] == "onnx"
    assert fake.export_kwargs["imgsz"] == 640
    assert fake.export_kwargs["batch"] == 1
    assert fake.export_kwargs["max_det"] == 100
    assert fake.export_kwargs["dynamic"] is False
    sidecar = tmp_path / "hestia_home_home_labels.json"
    assert sidecar.is_file()
    assert "chair" in sidecar.read_text(encoding="utf-8")
    meta = (tmp_path / "hestia_home.meta.json").read_text(encoding="utf-8")
    assert INPUT_NAME in meta
    assert OUTPUT_NAME in meta
    assert "max_detections" in meta


def test_export_open_detector_writes_prompt_sidecar(tmp_path: Path) -> None:
    onnx_path = tmp_path / "hestia_open.onnx"
    fake = FakeExportYolo(write_to=tmp_path / "raw.onnx")
    export_open_detector(
        "yoloworld.pt",
        onnx_path,
        image_size=480,
        max_detections=50,
        prompts=["sofa", "remote"],
        factory=lambda _: fake,
    )
    assert (tmp_path / "hestia_open_open_prompts.json").is_file()
    assert fake.export_kwargs["imgsz"] == 480
    assert fake.export_kwargs["max_det"] == 50


def test_export_rejects_empty_labels(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="must not be empty"):
        export_detector(
            "best.pt",
            tmp_path / "x.onnx",
            640,
            100,
            labels=[],
            factory=lambda _: FakeExportYolo(tmp_path / "raw.onnx"),
        )
