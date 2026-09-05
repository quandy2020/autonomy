"""Tests for Hestia home detector training facade."""

from pathlib import Path
from typing import Any, Dict

import pytest

from hestia.train_home import ULTRALYTICS_LICENSE_HINT, train_home_detector


class FakeYolo:
    def __init__(self) -> None:
        self.train_kwargs: Dict[str, Any] = {}

    def train(self, **kwargs: Any) -> None:
        self.train_kwargs = kwargs
        weights = Path(kwargs["project"]) / kwargs["name"] / "weights"
        weights.mkdir(parents=True, exist_ok=True)
        (weights / "best.pt").write_bytes(b"fake")


def test_train_home_detector_forwards_arguments(tmp_path: Path) -> None:
    fake = FakeYolo()
    best = train_home_detector(
        "yolo11n.pt",
        "data.yaml",
        tmp_path,
        epochs=3,
        image_size=640,
        batch_size=4,
        device="cpu",
        workers=0,
        seed=7,
        class_indices=[0, 1],
        factory=lambda _: fake,
    )
    assert best.is_file()
    assert fake.train_kwargs["data"] == "data.yaml"
    assert fake.train_kwargs["epochs"] == 3
    assert fake.train_kwargs["imgsz"] == 640
    assert fake.train_kwargs["batch"] == 4
    assert fake.train_kwargs["device"] == "cpu"
    assert fake.train_kwargs["seed"] == 7
    assert fake.train_kwargs["classes"] == [0, 1]
    assert fake.train_kwargs["project"] == str(tmp_path)
    assert fake.train_kwargs["name"] == "home"


def test_train_home_detector_requires_checkpoint_artifact(tmp_path: Path) -> None:
    class EmptyYolo:
        def train(self, **kwargs: Any) -> None:
            Path(kwargs["project"]).mkdir(parents=True, exist_ok=True)

    with pytest.raises(RuntimeError, match="no checkpoint"):
        train_home_detector(
            "yolo11n.pt",
            "data.yaml",
            tmp_path,
            epochs=1,
            image_size=320,
            batch_size=1,
            factory=lambda _: EmptyYolo(),
        )


def test_default_factory_message_mentions_license(monkeypatch: pytest.MonkeyPatch) -> None:
    import hestia.train_home as train_home

    def boom(_: str) -> Any:
        raise ImportError(ULTRALYTICS_LICENSE_HINT)

    monkeypatch.setattr(train_home, "_default_factory", boom)
    with pytest.raises(ImportError, match="AGPL-3.0"):
        train_home_detector(
            "yolo11n.pt",
            "data.yaml",
            Path("/tmp"),
            epochs=1,
            image_size=320,
            batch_size=1,
            factory=None,
        )
