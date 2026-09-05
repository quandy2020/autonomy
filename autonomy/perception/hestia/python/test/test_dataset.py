"""Tests for Hestia home manifest dataset materialization."""

from pathlib import Path

import pytest

from hestia.dataset import HomeManifestDataset, write_yolo_dataset


def _write_manifest(tmp_path: Path) -> Path:
    image = tmp_path / "img.jpg"
    image.write_bytes(b"fake-jpeg")
    manifest = tmp_path / "train.jsonl"
    manifest.write_text(
        '{"image":"img.jpg","boxes":[[0,0.5,0.5,0.2,0.3]]}\n',
        encoding="utf-8",
    )
    return manifest


def test_home_manifest_loads_and_resolves_paths(tmp_path: Path) -> None:
    manifest = _write_manifest(tmp_path)
    dataset = HomeManifestDataset(manifest, ["chair", "cup"])
    assert len(dataset) == 1
    sample = dataset[0]
    assert sample.image == tmp_path / "img.jpg"
    assert sample.boxes[0][0] == 0.0
    assert sample.boxes[0][1:] == [0.5, 0.5, 0.2, 0.3]


def test_home_manifest_rejects_bad_class(tmp_path: Path) -> None:
    image = tmp_path / "img.jpg"
    image.write_bytes(b"x")
    manifest = tmp_path / "train.jsonl"
    manifest.write_text(
        '{"image":"img.jpg","boxes":[[9,0.5,0.5,0.1,0.1]]}\n',
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="out of range"):
        HomeManifestDataset(manifest, ["chair"])


def test_write_yolo_dataset_layout(tmp_path: Path) -> None:
    manifest = _write_manifest(tmp_path)
    dataset = HomeManifestDataset(manifest, ["chair", "cup"])
    out = tmp_path / "yolo"
    yaml_path = write_yolo_dataset(dataset, out)
    assert yaml_path.is_file()
    assert (out / "images" / "train" / "000000.jpg").is_file()
    labels = (out / "labels" / "train" / "000000.txt").read_text(encoding="utf-8")
    assert labels.startswith("0 0.500000 0.500000 0.200000 0.300000")
    text = yaml_path.read_text(encoding="utf-8")
    assert "nc: 2" in text
    assert "0: chair" in text
