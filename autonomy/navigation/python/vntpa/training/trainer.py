"""Training/validation subprocess orchestration and lifecycle callbacks."""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional


@dataclass
class TrainConfig:
    """Arguments forwarded to ``thirdparty.pan.planner.train``."""

    args: List[str] = field(default_factory=list)
    nproc: int = 1

    def command(self) -> List[str]:
        return [
            sys.executable, "-m", "torchrun",
            f"--nproc_per_node={self.nproc}",
            "-m", "thirdparty.pan.planner.train", *self.args,
        ]


@dataclass
class ValConfig:
    """Arguments forwarded to ``thirdparty.pan.planner.eval``."""

    args: List[str] = field(default_factory=list)

    def command(self) -> List[str]:
        return [sys.executable, "-m", "thirdparty.pan.planner.eval", *self.args]


class Trainer:
    def __init__(self, config: TrainConfig, callbacks: Optional["CallbackList"] = None) -> None:
        self.config = config
        self.callbacks = callbacks or CallbackList()

    def train(self) -> int:
        self.callbacks.on_train_start(self)
        code = subprocess.run(self.config.command(), check=False).returncode
        self.callbacks.on_train_end(self, code)
        return code


class Validator:
    def __init__(self, config: ValConfig) -> None:
        self.config = config

    def validate(self) -> int:
        return subprocess.run(self.config.command(), check=False).returncode


class Callback:
    """Minimal lifecycle hook; implement only the methods you need."""

    def on_train_start(self, trainer: Trainer) -> None:
        pass

    def on_train_end(self, trainer: Trainer, returncode: int) -> None:
        pass


class CallbackList:
    def __init__(self, callbacks: Optional[List[Callback]] = None) -> None:
        self._callbacks = list(callbacks or [])

    def add(self, callback: Callback) -> None:
        self._callbacks.append(callback)

    def on_train_start(self, trainer: Trainer) -> None:
        for callback in self._callbacks:
            callback.on_train_start(trainer)

    def on_train_end(self, trainer: Trainer, returncode: int) -> None:
        for callback in self._callbacks:
            callback.on_train_end(trainer, returncode)


class JsonlCallback(Callback):
    """Append start/end events as JSONL."""

    def __init__(self, path: str) -> None:
        self.path = path
        self._file = None

    def on_train_start(self, trainer: Trainer) -> None:
        Path(self.path).parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.path, "a", encoding="utf-8")
        self._file.write(json.dumps({"event": "start", "command": trainer.config.command()}) + "\n")
        self._file.flush()

    def on_train_end(self, trainer: Trainer, returncode: int) -> None:
        if self._file is not None:
            self._file.write(json.dumps({"event": "end", "returncode": returncode}) + "\n")
            self._file.close()
            self._file = None


class CheckpointCallback(Callback):
    """Promote a successful run's checkpoint to a stable path."""

    def __init__(self, source: str, destination: str, manifest: Optional[str] = None) -> None:
        self.source = source
        self.destination = destination
        self.manifest = manifest

    def on_train_end(self, trainer: Trainer, returncode: int) -> None:
        if returncode != 0:
            return
        source = Path(self.source)
        if not source.is_file():
            return
        destination = Path(self.destination)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        if self.manifest:
            manifest = Path(self.manifest)
            manifest.parent.mkdir(parents=True, exist_ok=True)
            manifest.write_text(json.dumps({
                "checkpoint": str(destination),
                "command": trainer.config.command(),
            }, indent=2), encoding="utf-8")


def train_main() -> None:
    """Entry point for ``vntpa-train``: forward argv to VNT-PA."""
    from thirdparty.pan.planner import train

    train.main()


def eval_main() -> None:
    """Entry point for ``vntpa-eval``: forward argv to VNT-PA eval."""
    from thirdparty.pan.planner import eval as pan_eval

    pan_eval.main()
