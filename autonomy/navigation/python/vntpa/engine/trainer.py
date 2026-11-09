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

"""Functional training/validation orchestration and lifecycle hooks."""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Optional, Sequence

StartHook = Callable[["TrainConfig"], None]
EndHook = Callable[["TrainConfig", int], None]


@dataclass
class TrainConfig:
    """Arguments forwarded to ``thirdparty.pan.planner.train``."""

    args: List[str] = field(default_factory=list)
    nproc: int = 1

    def build_command(self) -> List[str]:
        """Return the single-node torch.distributed command for the paper trainer."""
        return [
            sys.executable,
            "-m",
            "torch.distributed.run",
            "--standalone",
            f"--nproc_per_node={self.nproc}",
            "-m",
            "thirdparty.pan.planner.train",
            *self.args,
        ]


@dataclass
class ValConfig:
    """Arguments forwarded to ``thirdparty.pan.planner.eval``."""

    args: List[str] = field(default_factory=list)

    def build_command(self) -> List[str]:
        """Return the evaluator command."""
        return [sys.executable, "-m", "thirdparty.pan.planner.eval", *self.args]


def train(
    config: TrainConfig,
    *,
    start_hooks: Sequence[StartHook] = (),
    end_hooks: Sequence[EndHook] = (),
) -> int:
    """Run one training job and invoke lifecycle hooks."""
    for hook in start_hooks:
        hook(config)
    code = subprocess.run(config.build_command(), check=False).returncode
    for hook in end_hooks:
        hook(config, code)
    return code


def validate(config: ValConfig) -> int:
    """Run one evaluation job."""
    return subprocess.run(config.build_command(), check=False).returncode


def build_jsonl_start_hook(path: str) -> StartHook:
    """Create a start hook that appends a JSONL event."""

    def write_start(config: TrainConfig) -> None:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps({"event": "start", "command": config.build_command()}) + "\n")

    return write_start


def build_jsonl_end_hook(path: str) -> EndHook:
    """Create an end hook that appends a JSONL event."""

    def write_end(config: TrainConfig, returncode: int) -> None:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps({"event": "end", "returncode": returncode}) + "\n")

    return write_end


def build_checkpoint_hook(
    source: str,
    destination: str,
    manifest: Optional[str] = None,
) -> EndHook:
    """Create an end hook that promotes a successful checkpoint."""

    def promote_checkpoint(config: TrainConfig, returncode: int) -> None:
        if returncode != 0:
            return
        source_path = Path(source)
        if not source_path.is_file():
            return
        destination_path = Path(destination)
        destination_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_path, destination_path)
        if manifest:
            manifest_path = Path(manifest)
            manifest_path.parent.mkdir(parents=True, exist_ok=True)
            manifest_path.write_text(
                json.dumps(
                    {
                        "checkpoint": str(destination_path),
                        "command": config.build_command(),
                    },
                    indent=2,
                ),
                encoding="utf-8",
            )

    return promote_checkpoint
