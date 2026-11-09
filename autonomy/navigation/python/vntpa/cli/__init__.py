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

"""Command line entry points for the vntpa product.

The root parser exposes the full task surface:

    predict   one depth/pose/goal inference
    train     delegate to the vendored VNT-PA trainer
    evaluate  delegate to the vendored VNT-PA evaluator
    export    checkpoint -> ONNX
    autosim   run the autolink bridge
    benchmark latency measurement
    models    list/inspect local ONNX artifacts
    dataset   validate HDF5 token stores
    info      package and environment diagnostics

Task-specific modules under ``vntpa.cli`` are thin entry points for
``python -m vntpa.cli.<task>`` and shell wrappers.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from vntpa.data.analyze import analyze
from vntpa.data.dataset import validate_root
from vntpa.engine.deployment.validator import inspect
from vntpa.engine.model import VNT
from vntpa.engine.trainer import (
    build_checkpoint_hook,
    build_jsonl_end_hook,
    build_jsonl_start_hook,
)
from vntpa.utils.config import (
    lookup,
    names,
    DeployConfig,
    PlannerConfig,
    locate,
    configure,
)


def deploy(args: argparse.Namespace) -> DeployConfig:
    """Build deployment settings from ``--config`` or explicit model paths.

    ``--config`` owns all deployment fields; otherwise the CLI model paths and
    provider selection are used.
    """
    if args.config:
        return configure(args.config).deploy
    if not args.model or not args.defm:
        raise SystemExit("either --config or both --model and --defm are required")
    providers = tuple(args.provider or ("CPUExecutionProvider",))
    return DeployConfig(
        planner_onnx=args.model,
        defm_onnx=args.defm,
        planner=PlannerConfig(context_size=args.context_size, providers=providers),
        providers=providers,
    )


def inputs(args: argparse.Namespace) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Load the ``.npy`` depth map and normalize pose/goal arrays."""
    depth = np.load(args.depth)
    pose = np.asarray(args.pose, dtype=np.float32)
    goal = np.asarray(args.goal, dtype=np.float32)
    return depth, pose, goal


def emit(payload: object, out: str | None = None) -> None:
    """Print JSON and optionally write it to ``out``."""
    text = json.dumps(payload, indent=2, default=str)
    if out:
        Path(out).write_text(text, encoding="utf-8")
    print(text)


def infer(args: argparse.Namespace) -> int:
    """Run one DeFM + VNT-PA inference and print the decoded actions."""
    depth, pose, goal = inputs(args)
    api = VNT(deploy(args))
    result = api.predict(depth, pose, goal)
    emit(
        {
            "actions": [int(action) for action in result.actions],
            "primitives": [
                {"action": int(item.action), "distance": item.distance, "angle": item.angle}
                for item in result.primitives
            ],
            "memory_size": result.memory_size,
        },
        args.out,
    )
    return 0


def export(args: argparse.Namespace) -> int:
    """Export a VNT-PA checkpoint to ONNX and write its metadata sidecar."""
    return VNT.export(args.checkpoint, args.out, context_size=args.context_size, format=args.format)


def train(args: argparse.Namespace) -> int:
    """Dispatch to ``vntpa.cli.train`` (frozen-bank loop or paper DDP)."""
    from vntpa.cli.train import main as train_main

    forwarded: list[str] = []
    for name in (
        "nproc",
        "log",
        "checkpoint_in",
        "checkpoint_out",
        "manifest",
        "tensorboard_dir",
    ):
        value = getattr(args, name, None)
        if value is None:
            continue
        forwarded.extend([f"--{name.replace('_', '-')}", str(value)])
    forwarded.extend(getattr(args, "trailing", []) or [])
    return train_main(forwarded)


def validate(args: argparse.Namespace) -> int:
    """Forward evaluation flags to the vendored VNT-PA evaluator."""
    return VNT.validate(args.trailing)


def simulate(args: argparse.Namespace) -> int:
    """Launch the autolink bridge as an isolated process."""
    command = [sys.executable, "-m", "vntpa.simulator.client", *args.trailing]
    return subprocess.run(command, check=False).returncode


def benchmark(args: argparse.Namespace) -> int:
    """Measure predictor latency percentiles."""
    depth, pose, goal = inputs(args)
    api = VNT(deploy(args))
    report = api.benchmark(depth, pose, goal, warmup=args.warmup, runs=args.runs)
    emit(report.dump())
    return 0


def models(args: argparse.Namespace) -> int:
    """List registry entries or inspect one ONNX artifact."""
    if args.inspect:
        info = inspect(args.inspect)
        payload = {
            "path": info.path,
            "inputs": {
                name: {"shape": list(spec.shape), "dtype": spec.dtype}
                for name, spec in info.inputs.items()
            },
            "outputs": {
                name: {"shape": list(spec.shape), "dtype": spec.dtype}
                for name, spec in info.outputs.items()
            },
        }
    else:
        payload = {name: asdict(lookup(name)) for name in names() if lookup(name) is not None}
    emit(payload)
    return 0


def dataset(args: argparse.Namespace) -> int:
    """Validate HDF5 token stores, or analyze corpus statistics."""
    if getattr(args, "analyze", False):
        report = analyze(
            args.root,
            getattr(args, "bank", "") or "",
            tensorboard_dir=getattr(args, "tensorboard_dir", None) or None,
            out=getattr(args, "out", None) or None,
        )
        emit(asdict(report))
        return 1 if report.errors and report.n_tours == 0 else 0
    report = validate_root(args.root)
    emit({"files": report.files, "errors": report.errors})
    return 0 if report.ok else 1


def diagnose() -> dict[str, str]:
    """Collect versions useful for bug reports."""
    import platform
    from importlib.metadata import PackageNotFoundError, version

    def installed(name: str) -> str:
        try:
            return version(name)
        except PackageNotFoundError:
            return "not-installed"

    return {
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "numpy": version("numpy"),
        "onnxruntime": version("onnxruntime"),
        "protobuf": version("protobuf"),
        "vntpa": version("vntpa"),
    }


def info(args: argparse.Namespace) -> int:
    """Print package, default config, task list and runtime versions."""
    del args
    from importlib.metadata import PackageNotFoundError, version

    from vntpa.version import __version__

    try:
        package_version = version("vntpa")
    except PackageNotFoundError:
        package_version = __version__
    emit(
        {
            "name": "vntpa",
            "version": package_version,
            "default_config": str(locate()),
            "tasks": [
                "predict",
                "train",
                "val",
                "export",
                "autosim",
                "benchmark",
                "dataset",
                "analyze",
                "models",
                "info",
            ],
            "environment": diagnose(),
        }
    )
    return 0


def add_predict_parser(sub: Any) -> None:
    """Add the one-shot inference command."""
    parser = sub.add_parser("predict", help="run one depth/pose/goal prediction")
    parser.add_argument("--config", default=None, help="product YAML config")
    parser.add_argument("--model", default=None, help="planner ONNX")
    parser.add_argument("--defm", default=None, help="DeFM ONNX")
    parser.add_argument("--depth", required=True, help=".npy metric depth (H, W)")
    parser.add_argument("--pose", nargs=3, type=float, required=True, metavar=("X", "Y", "YAW"))
    parser.add_argument("--goal", nargs=2, type=float, required=True, metavar=("X", "Y"))
    parser.add_argument("--context-size", type=int, default=64)
    parser.add_argument("--provider", action="append", default=[])
    parser.add_argument("--out", default=None, help="optional JSON output path")
    parser.set_defaults(func=infer)


def add_export_parser(sub: Any) -> None:
    """Add the ONNX export command."""
    parser = sub.add_parser("export", help="export a checkpoint to ONNX")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--context-size", type=int, default=64)
    parser.add_argument("--format", default="onnx", choices=["onnx"])
    parser.set_defaults(func=export)


def add_train_parser(sub: Any) -> None:
    """Add the training command; unknown flags are forwarded to VNT-PA."""
    parser = sub.add_parser("train", help="train via vendored VNT-PA")
    parser.add_argument("--nproc", type=int, default=1)
    parser.add_argument("--log", default=None, help="JSONL lifecycle log")
    parser.add_argument("--checkpoint-in", default="logs/pan/checkpoints/final.pt")
    parser.add_argument("--checkpoint-out", default=None)
    parser.add_argument("--manifest", default=None)
    parser.add_argument("--tensorboard-dir", default=None)
    parser.add_argument("trailing", nargs=argparse.REMAINDER)
    parser.set_defaults(func=train)


def add_evaluate_parser(sub: Any) -> None:
    """Add the evaluation command."""
    parser = sub.add_parser("evaluate", help="evaluate via vendored VNT-PA")
    parser.add_argument("trailing", nargs=argparse.REMAINDER)
    parser.set_defaults(func=validate)


def add_autosim_parser(sub: Any) -> None:
    """Add the autolink bridge command."""
    parser = sub.add_parser("autosim", help="run the autolink bridge")
    parser.add_argument("trailing", nargs=argparse.REMAINDER)
    parser.set_defaults(func=simulate)


def add_benchmark_parser(sub: Any) -> None:
    """Add the latency benchmark command."""
    parser = sub.add_parser("benchmark", help="measure predictor latency")
    parser.add_argument("--config", default=str(locate()))
    parser.add_argument("--depth", required=True, help=".npy metric depth")
    parser.add_argument("--pose", nargs=3, type=float, required=True)
    parser.add_argument("--goal", nargs=2, type=float, required=True)
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--runs", type=int, default=20)
    parser.set_defaults(func=benchmark)


def add_models_parser(sub: Any) -> None:
    """Add the model registry inspection command."""
    parser = sub.add_parser("models", help="list or inspect local models")
    parser.add_argument("--inspect", default=None, help="ONNX path to inspect")
    parser.set_defaults(func=models)


def add_dataset_parser(sub: Any) -> None:
    """Add the token-store validation command."""
    parser = sub.add_parser("dataset", help="validate or analyze offline token stores")
    parser.add_argument("--root", required=True)
    parser.add_argument("--analyze", action="store_true", help="write coverage/path/action stats")
    parser.add_argument("--bank", default="", help="optional frozen bank npz")
    parser.add_argument("--tensorboard-dir", default="")
    parser.add_argument("--out", default="", help="JSON report path")
    parser.set_defaults(func=dataset)


def add_analyze_parser(sub: Any) -> None:
    """Add the corpus statistics command."""
    parser = sub.add_parser("analyze", help="summarize tokens/banks to TensorBoard")
    parser.add_argument("--tokens", default="")
    parser.add_argument("--bank", default="")
    parser.add_argument("--tensorboard-dir", default="")
    parser.add_argument("--out", default="")
    parser.set_defaults(func=_analyze)


def _analyze(args: argparse.Namespace) -> int:
    from vntpa.cli.analyze import main as analyze_main

    forwarded: list[str] = []
    if args.tokens:
        forwarded.extend(["--tokens", args.tokens])
    if args.bank:
        forwarded.extend(["--bank", args.bank])
    if args.tensorboard_dir:
        forwarded.extend(["--tensorboard-dir", args.tensorboard_dir])
    if args.out:
        forwarded.extend(["--out", args.out])
    return analyze_main(forwarded)


def add_info_parser(sub: Any) -> None:
    """Add the environment diagnostics command."""
    parser = sub.add_parser("info", help="print package/config information")
    parser.set_defaults(func=info)


def build_parser() -> argparse.ArgumentParser:
    """Build the complete ``vntpa`` command surface."""
    parser = argparse.ArgumentParser(
        prog="vntpa",
        description="VNT-PA training, inference and deployment CLI",
    )
    sub = parser.add_subparsers(dest="task", required=True)
    add_predict_parser(sub)
    add_export_parser(sub)
    add_train_parser(sub)
    add_evaluate_parser(sub)
    add_autosim_parser(sub)
    add_benchmark_parser(sub)
    add_models_parser(sub)
    add_dataset_parser(sub)
    add_analyze_parser(sub)
    add_info_parser(sub)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Parse arguments and dispatch to one task handler."""
    parser = build_parser()
    args, unknown = parser.parse_known_args(argv)
    if unknown and hasattr(args, "trailing"):
        args.trailing.extend(unknown)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
