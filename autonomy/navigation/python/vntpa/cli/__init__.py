"""Unified `vntpa` command line, modeled after YOLO-style task commands."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from ..runtime.inference.predictor import VNT
from ..utils.config import (
    DEFAULT_REGISTRY,
    DeployConfig,
    PlannerConfig,
    default_config_path,
    load_product_config,
)
from ..data.dataset import validate_root
from ..runtime.deployment.onnx_validator import inspect_onnx
from ..training.trainer import CallbackList, CheckpointCallback, JsonlCallback


def _predict(args: argparse.Namespace) -> int:
    depth = np.load(args.depth)
    pose = np.asarray(args.pose, dtype=np.float32)
    goal = np.asarray(args.goal, dtype=np.float32)
    if args.config:
        deploy = load_product_config(args.config).deploy
    else:
        deploy = DeployConfig(
            planner_onnx=args.model,
            defm_onnx=args.defm,
            planner=PlannerConfig(context_size=args.context_size,
                                  providers=tuple(args.provider or ("CPUExecutionProvider",))),
            providers=tuple(args.provider or ("CPUExecutionProvider",)))
    api = VNT(deploy)
    result = api.predict(depth, pose, goal)
    payload = {
        "actions": [int(action) for action in result.actions],
        "primitives": [
            {"action": int(item.action), "distance": item.distance, "angle": item.angle}
            for item in result.primitives
        ],
        "memory_size": result.memory_size,
    }
    if args.out:
        Path(args.out).write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0


def _export(args: argparse.Namespace) -> int:
    return VNT.export(args.checkpoint, args.out, context_size=args.context_size,
                      format=args.format)


def _train(args: argparse.Namespace) -> int:
    if args.tensorboard_dir:
        args.trailing.extend(["--tensorboard-dir", args.tensorboard_dir])
    callbacks = []
    if args.log:
        callbacks.append(JsonlCallback(args.log))
    if args.checkpoint_out:
        callbacks.append(CheckpointCallback(
            args.checkpoint_in, args.checkpoint_out, args.manifest))
    return VNT.train(args.trailing, nproc=args.nproc,
                     callbacks=CallbackList(callbacks) if callbacks else None)


def _val(args: argparse.Namespace) -> int:
    return VNT.val(args.trailing)


def _autosim(args: argparse.Namespace) -> int:
    command = [sys.executable, "-m", "vntpa.runtime.sim.autosim_client", *args.trailing]
    import subprocess

    return subprocess.run(command, check=False).returncode


def _benchmark(args: argparse.Namespace) -> int:
    depth = np.load(args.depth)
    pose = np.asarray(args.pose, dtype=np.float32)
    goal = np.asarray(args.goal, dtype=np.float32)
    deploy = load_product_config(args.config).deploy
    api = VNT(deploy)
    report = api.benchmark(depth, pose, goal, warmup=args.warmup, runs=args.runs)
    print(json.dumps(report.to_dict(), indent=2))
    return 0


def _models(args: argparse.Namespace) -> int:
    if args.inspect:
        info = inspect_onnx(args.inspect)
        payload = {
            "path": info.path,
            "inputs": {name: {"shape": list(spec.shape), "dtype": spec.dtype}
                       for name, spec in info.inputs.items()},
            "outputs": {name: {"shape": list(spec.shape), "dtype": spec.dtype}
                        for name, spec in info.outputs.items()},
        }
    else:
        payload = {name: DEFAULT_REGISTRY.get(name).__dict__
                   for name in DEFAULT_REGISTRY.names()}
    print(json.dumps(payload, indent=2, default=str))
    return 0


def _dataset(args: argparse.Namespace) -> int:
    report = validate_root(args.root)
    print(json.dumps({"files": report.files, "errors": report.errors}, indent=2))
    return 0 if report.ok else 1


def _environment_info() -> dict:
    import platform
    import sys
    from importlib.metadata import PackageNotFoundError, version

    def installed(name: str) -> str:
        try:
            return version(name)
        except PackageNotFoundError:
            return "not-installed"

    return {
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "numpy": installed("numpy"),
        "onnxruntime": installed("onnxruntime"),
        "protobuf": installed("protobuf"),
        "vntpa": installed("vntpa"),
    }


def _info(_args: argparse.Namespace) -> int:
    from importlib.metadata import PackageNotFoundError, version

    from ..version import __version__

    try:
        package_version = version("vntpa")
    except PackageNotFoundError:
        package_version = __version__
    print(json.dumps({
        "name": "vntpa",
        "version": package_version,
        "default_config": str(default_config_path()),
        "tasks": ["predict", "train", "val", "export", "autosim",
                  "benchmark", "dataset", "models"],
        "environment": _environment_info(),
    }, indent=2))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="vntpa")
    sub = parser.add_subparsers(dest="task", required=True)

    predict = sub.add_parser("predict", help="run one depth/pose/goal prediction")
    predict.add_argument("--model")
    predict.add_argument("--defm")
    predict.add_argument("--config")
    predict.add_argument("--depth", required=True, help=".npy metric depth (H, W)")
    predict.add_argument("--pose", nargs=3, type=float, required=True, metavar=("X", "Y", "YAW"))
    predict.add_argument("--goal", nargs=2, type=float, required=True, metavar=("X", "Y"))
    predict.add_argument("--context-size", type=int, default=64)
    predict.add_argument("--provider", action="append", default=[])
    predict.add_argument("--out", default=None)
    predict.set_defaults(func=_predict)

    export = sub.add_parser("export", help="export a checkpoint to ONNX")
    export.add_argument("--checkpoint", required=True)
    export.add_argument("--out", required=True)
    export.add_argument("--context-size", type=int, default=64)
    export.add_argument("--format", default="onnx", choices=["onnx"])
    export.set_defaults(func=_export)

    train = sub.add_parser("train", help="train via vendored VNT-PA")
    train.add_argument("--nproc", type=int, default=1)
    train.add_argument("--log", default=None, help="JSONL lifecycle log")
    train.add_argument("--checkpoint-in", default="logs/pan/checkpoints/final.pt")
    train.add_argument("--checkpoint-out", default=None)
    train.add_argument("--manifest", default=None)
    train.add_argument("--tensorboard-dir", default=None)
    train.add_argument("trailing", nargs=argparse.REMAINDER)
    train.set_defaults(func=_train)

    val = sub.add_parser("val", help="evaluate via vendored VNT-PA")
    val.add_argument("trailing", nargs=argparse.REMAINDER)
    val.set_defaults(func=_val)

    autosim = sub.add_parser("autosim", help="run the autolink bridge")
    autosim.add_argument("trailing", nargs=argparse.REMAINDER)
    autosim.set_defaults(func=_autosim)

    benchmark = sub.add_parser("benchmark", help="measure predictor latency")
    benchmark.add_argument("--config", default=str(default_config_path()))
    benchmark.add_argument("--depth", required=True, help=".npy metric depth")
    benchmark.add_argument("--pose", nargs=3, type=float, required=True)
    benchmark.add_argument("--goal", nargs=2, type=float, required=True)
    benchmark.add_argument("--warmup", type=int, default=3)
    benchmark.add_argument("--runs", type=int, default=20)
    benchmark.set_defaults(func=_benchmark)

    models = sub.add_parser("models", help="list or inspect local models")
    models.add_argument("--inspect", default=None)
    models.set_defaults(func=_models)

    dataset = sub.add_parser("dataset", help="validate offline token stores")
    dataset.add_argument("--root", required=True)
    dataset.set_defaults(func=_dataset)

    info = sub.add_parser("info", help="print package/config information")
    info.set_defaults(func=_info)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args, unknown = parser.parse_known_args(argv)
    if unknown and hasattr(args, "trailing"):
        args.trailing.extend(unknown)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
