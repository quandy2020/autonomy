"""Tests for Hestia label/prompt helpers."""

from pathlib import Path

import pytest

from hestia.prompts import assert_same_order, load_label_list, write_label_sidecar


def test_load_label_list_from_json(tmp_path: Path) -> None:
    path = tmp_path / "labels.json"
    path.write_text('["chair", "cup"]\n', encoding="utf-8")
    assert load_label_list(path) == ["chair", "cup"]


def test_load_label_list_from_text(tmp_path: Path) -> None:
    path = tmp_path / "labels.txt"
    path.write_text("chair\ncup\n", encoding="utf-8")
    assert load_label_list(path) == ["chair", "cup"]


def test_load_label_list_rejects_empty(tmp_path: Path) -> None:
    path = tmp_path / "empty.txt"
    path.write_text("\n", encoding="utf-8")
    with pytest.raises(ValueError, match="empty"):
        load_label_list(path)


def test_write_label_sidecar_and_order_check(tmp_path: Path) -> None:
    path = write_label_sidecar(tmp_path / "home_labels.json", ["a", "b"])
    assert load_label_list(path) == ["a", "b"]
    assert_same_order(["a", "b"], ["a", "b"], "home")
    with pytest.raises(ValueError, match="order mismatch"):
        assert_same_order(["a", "b"], ["b", "a"], "home")
