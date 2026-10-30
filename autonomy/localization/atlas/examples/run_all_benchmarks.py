#!/usr/bin/env python3
"""Run the supported Atlas accuracy suites sequentially and reproducibly.

The runner deliberately never executes more than one SLAM process at a time.
Every job gets its own trajectory, logs, evaluation artifacts and status JSON.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import os
import re
import statistics
import struct
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Iterable

from associate_tum_rgbd import associate, read_file_list


EUROC_SEQUENCES = (
    "MH_01_easy",
    "MH_02_easy",
    "MH_03_medium",
    "MH_04_difficult",
    "MH_05_difficult",
    "V1_01_easy",
    "V1_02_medium",
    "V1_03_difficult",
    "V2_01_easy",
    "V2_02_medium",
    "V2_03_difficult",
)
KITTI_SEQUENCES = tuple(f"{index:02d}" for index in range(11))
TARTANAIR_SEQUENCES = tuple(
    f"ArchVizTinyHouseDay/Data_easy/P{index:03d}" for index in range(7)
)
DEFAULT_SUITES = ("euroc", "kitti", "tum_rgbd", "tum_vi", "tartanair")
TUM_RGBD_CALIBRATIONS = {
    "freiburg1": {
        "Camera1.fx": 517.306408,
        "Camera1.fy": 516.469215,
        "Camera1.cx": 318.643040,
        "Camera1.cy": 255.313989,
        "Camera1.k1": 0.262383,
        "Camera1.k2": -0.953104,
        "Camera1.p1": -0.005358,
        "Camera1.p2": 0.002628,
        "Camera1.k3": 1.163314,
        "Stereo.b": 0.07732,
        "RGBD.DepthMapFactor": 5000.0,
    },
    "freiburg2": {
        "Camera1.fx": 520.908620,
        "Camera1.fy": 521.007327,
        "Camera1.cx": 325.141442,
        "Camera1.cy": 249.701764,
        "Camera1.k1": 0.231222,
        "Camera1.k2": -0.784899,
        "Camera1.p1": -0.003257,
        "Camera1.p2": -0.000105,
        "Camera1.k3": 0.917205,
        "Stereo.b": 0.0767,
        "RGBD.DepthMapFactor": 5208.0,
    },
    "freiburg3": {
        "Camera1.fx": 535.4,
        "Camera1.fy": 539.2,
        "Camera1.cx": 320.1,
        "Camera1.cy": 247.6,
        "Camera1.k1": 0.0,
        "Camera1.k2": 0.0,
        "Camera1.p1": 0.0,
        "Camera1.p2": 0.0,
        "Camera1.k3": 0.0,
        "Stereo.b": 0.0747,
        "RGBD.DepthMapFactor": 5000.0,
    },
}


@dataclasses.dataclass(frozen=True)
class Thresholds:
    maximum_ate_rmse_m: float
    minimum_coverage_percent: float
    maximum_translation_drift_percent: float | None = None
    maximum_rotation_drift_deg_per_100m: float | None = None


@dataclasses.dataclass(frozen=True)
class Job:
    suite: str
    sequence: str
    command: tuple[str, ...]
    evaluation_command: tuple[str, ...]
    trajectory: Path
    evaluation_dir: Path
    required_paths: tuple[Path, ...]
    thresholds: Thresholds

    @property
    def output_dir(self) -> Path:
        return self.trajectory.parent


@dataclasses.dataclass
class Result:
    suite: str
    sequence: str
    status: str
    resumed: bool = False
    elapsed_seconds: float = 0.0
    run_exit_code: int | None = None
    evaluation_exit_code: int | None = None
    ate_rmse_m: float | None = None
    coverage_percent: float | None = None
    translation_drift_percent: float | None = None
    rotation_drift_deg_per_100m: float | None = None
    violations: list[str] = dataclasses.field(default_factory=list)
    message: str = ""


def finite_positive(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("expected a finite positive value")
    return parsed


def finite_percentage(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0 or parsed > 100.0:
        raise argparse.ArgumentTypeError("expected a percentage in [0, 100]")
    return parsed


def parse_suites(value: str) -> tuple[str, ...]:
    suites = tuple(dict.fromkeys(part.strip() for part in value.split(",")))
    allowed = {*DEFAULT_SUITES, "mono"}
    invalid = sorted(set(suites) - allowed)
    if not suites or invalid:
        raise argparse.ArgumentTypeError(
            "suites must be comma-separated values from " + ",".join(sorted(allowed))
        )
    return suites


def parse_projection(line: str, label: str, path: Path) -> tuple[float, ...]:
    values = tuple(float(value) for value in line.split())
    if len(values) != 12 or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{path}: {label} must contain 12 finite values")
    return values


def read_kitti_calibration(path: Path) -> tuple[float, float, float, float, float]:
    projections: dict[str, tuple[float, ...]] = {}
    with path.open("r", encoding="utf-8") as stream:
        for raw_line in stream:
            if ":" not in raw_line:
                continue
            label, values = raw_line.split(":", 1)
            label = label.strip()
            if label in ("P0", "P1"):
                projections[label] = parse_projection(values, label, path)
    if set(projections) != {"P0", "P1"}:
        raise ValueError(f"{path}: rectified P0/P1 projections are required")
    primary = projections["P0"]
    secondary = projections["P1"]
    fx = primary[0]
    fy = primary[5]
    if fx <= 0.0 or fy <= 0.0 or secondary[0] <= 0.0:
        raise ValueError(f"{path}: focal lengths must be positive")
    primary_center_x = -primary[3] / primary[0]
    secondary_center_x = -secondary[3] / secondary[0]
    baseline = abs(secondary_center_x - primary_center_x)
    if not math.isfinite(baseline) or baseline <= 0.0:
        raise ValueError(f"{path}: P0/P1 baseline must be positive")
    return fx, fy, primary[2], primary[6], baseline


def read_png_size(path: Path) -> tuple[int, int]:
    with path.open("rb") as stream:
        header = stream.read(24)
    if (
        len(header) != 24
        or header[:8] != b"\x89PNG\r\n\x1a\n"
        or header[12:16] != b"IHDR"
    ):
        raise ValueError(f"{path}: expected a PNG IHDR header")
    width, height = struct.unpack(">II", header[16:24])
    if width <= 0 or height <= 0:
        raise ValueError(f"{path}: PNG dimensions must be positive")
    return width, height


def read_kitti_fps(path: Path) -> float:
    timestamps = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.strip()
            if not line:
                continue
            value = float(line)
            if not math.isfinite(value):
                raise ValueError(f"{path}:{line_number}: timestamp is not finite")
            timestamps.append(value)
    intervals = [
        second - first for first, second in zip(timestamps, timestamps[1:])
        if second > first
    ]
    if not intervals:
        raise ValueError(f"{path}: at least two increasing timestamps are required")
    fps = 1.0 / statistics.median(intervals)
    if not math.isfinite(fps) or fps <= 0.0:
        raise ValueError(f"{path}: invalid timestamp cadence")
    return fps


def replace_yaml_scalar(text: str, key: str, value: str) -> str:
    pattern = re.compile(rf"^{re.escape(key)}:\s*.*$", flags=re.MULTILINE)
    if len(pattern.findall(text)) != 1:
        raise ValueError(f"base config must contain exactly one {key}")
    return pattern.sub(f"{key}: {value}", text)


def write_kitti_config(base_config: Path, sequence_root: Path, output: Path) -> None:
    fx, fy, cx, cy, baseline = read_kitti_calibration(
        sequence_root / "calib.txt"
    )
    images = sorted((sequence_root / "image_0").glob("*.png"))
    if not images:
        raise ValueError(f"{sequence_root}: image_0 contains no PNG images")
    width, height = read_png_size(images[0])
    fps = read_kitti_fps(sequence_root / "times.txt")
    integer_fps = max(1, int(round(fps)))
    text = base_config.read_text(encoding="utf-8")
    replacements = {
        "Camera1.fx": f"{fx:.12g}",
        "Camera1.fy": f"{fy:.12g}",
        "Camera1.cx": f"{cx:.12g}",
        "Camera1.cy": f"{cy:.12g}",
        "Camera.width": str(width),
        "Camera.height": str(height),
        "Camera.fps": str(integer_fps),
        "Stereo.b": f"{baseline:.12g}",
    }
    for key, value in replacements.items():
        text = replace_yaml_scalar(text, key, value)
    output.parent.mkdir(parents=True, exist_ok=True)
    generated_comment = (
        "# Generated from this sequence's calib.txt, times.txt and image_0.\n"
    )
    if text.startswith("%YAML:") and "\n" in text:
        header, remainder = text.split("\n", 1)
        text = header + "\n" + generated_comment + remainder
    else:
        text = generated_comment + text
    output.write_text(text, encoding="utf-8")


def tum_rgbd_family(sequence: str) -> str:
    match = re.fullmatch(r"rgbd_dataset_(freiburg[123])_.+", sequence)
    if match is None:
        raise ValueError(f"unsupported TUM RGB-D sequence name: {sequence}")
    return match.group(1)


def discover_tum_rgbd_sequences(data_root: Path) -> tuple[str, ...]:
    """Discover the union of downloaded archives and extracted sequences."""
    root = data_root / "TUM"
    if not root.is_dir():
        return ()
    sequences = {
        path.name
        for path in root.glob("rgbd_dataset_*")
        if path.is_dir()
    }
    sequences.update(
        path.stem for path in root.glob("rgbd_dataset_*.tgz") if path.is_file()
    )
    return tuple(sorted(sequences))


def write_tum_rgbd_config(
    base_config: Path, sequence: str, output: Path
) -> None:
    family = tum_rgbd_family(sequence)
    text = base_config.read_text(encoding="utf-8")
    for key, value in TUM_RGBD_CALIBRATIONS[family].items():
        text = replace_yaml_scalar(text, key, f"{value:.12g}")
    generated_comment = (
        f"# Generated for {sequence} using the official {family} calibration.\n"
    )
    if text.startswith("%YAML:") and "\n" in text:
        header, remainder = text.split("\n", 1)
        text = header + "\n" + generated_comment + remainder
    else:
        text = generated_comment + text
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8")


def write_tum_rgbd_associations(
    dataset: Path, output: Path, maximum_difference: float = 0.02
) -> int:
    rgb_entries = read_file_list(dataset / "rgb.txt")
    depth_entries = read_file_list(dataset / "depth.txt")
    matches = associate(rgb_entries, depth_entries, maximum_difference)
    if not matches:
        raise ValueError(f"{dataset}: no RGB/depth timestamp pairs")
    lines = [
        f"{rgb.timestamp_text} {rgb.path} "
        f"{depth.timestamp_text} {depth.path}\n"
        for rgb, depth in matches
    ]
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text("".join(lines), encoding="utf-8")
    temporary.replace(output)
    return len(matches)


def metrics_coverage(metrics: dict[str, Any]) -> float | None:
    for key in (
        "ate_ground_truth_coverage_percent",
        "ground_truth_coverage_percent",
        "coverage_percent",
        "estimate_coverage_percent",
    ):
        value = metrics.get(key)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            return float(value)
    if isinstance(metrics.get("associated_poses"), int):
        return 100.0
    return None


def assess_metrics(
    metrics: dict[str, Any], thresholds: Thresholds
) -> tuple[list[str], float, float | None, float | None, float | None]:
    ate = float(metrics["ate_rmse_m"])
    if not math.isfinite(ate):
        raise ValueError("ate_rmse_m is missing or non-finite")
    coverage = metrics_coverage(metrics)
    translation_drift = metrics.get("kitti_translation_drift_percent")
    rotation_drift = metrics.get("kitti_rotation_drift_deg_per_100m")
    translation_drift = (
        float(translation_drift) if translation_drift is not None else None
    )
    rotation_drift = float(rotation_drift) if rotation_drift is not None else None
    violations = []
    if ate > thresholds.maximum_ate_rmse_m:
        violations.append(
            f"ATE RMSE {ate:.6g} m > {thresholds.maximum_ate_rmse_m:.6g} m"
        )
    if coverage is None:
        violations.append("coverage metric is missing")
    elif coverage < thresholds.minimum_coverage_percent:
        violations.append(
            f"coverage {coverage:.3f}% < "
            f"{thresholds.minimum_coverage_percent:.3f}%"
        )
    if (
        thresholds.maximum_translation_drift_percent is not None
        and translation_drift is not None
        and translation_drift > thresholds.maximum_translation_drift_percent
    ):
        violations.append(
            f"translation drift {translation_drift:.6g}% > "
            f"{thresholds.maximum_translation_drift_percent:.6g}%"
        )
    if (
        thresholds.maximum_rotation_drift_deg_per_100m is not None
        and rotation_drift is not None
        and rotation_drift
        > thresholds.maximum_rotation_drift_deg_per_100m
    ):
        violations.append(
            f"rotation drift {rotation_drift:.6g} deg/100m > "
            f"{thresholds.maximum_rotation_drift_deg_per_100m:.6g} deg/100m"
        )
    return violations, ate, coverage, translation_drift, rotation_drift


def load_metrics(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path}: metrics must be a JSON object")
    return value


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def run_command(
    command: tuple[str, ...], log_path: Path, environment: dict[str, str]
) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as stream:
        stream.write("command: " + json.dumps(command) + "\n")
        stream.flush()
        completed = subprocess.run(
            command,
            stdout=stream,
            stderr=subprocess.STDOUT,
            env=environment,
            check=False,
        )
    return completed.returncode


def make_thresholds(args: argparse.Namespace, suite: str) -> Thresholds:
    if suite in ("euroc", "mono"):
        return Thresholds(
            args.euroc_max_ate_m,
            args.euroc_min_coverage_percent,
        )
    if suite == "kitti":
        return Thresholds(
            args.kitti_max_ate_m,
            100.0,
            args.kitti_max_translation_drift_percent,
            args.kitti_max_rotation_drift_deg_per_100m,
        )
    if suite == "tum_rgbd":
        return Thresholds(
            args.tum_rgbd_max_ate_m,
            args.tum_rgbd_min_coverage_percent,
        )
    if suite == "tum_vi":
        return Thresholds(
            args.tum_vi_max_ate_m,
            args.tum_vi_min_coverage_percent,
        )
    if suite == "tartanair":
        return Thresholds(
            args.tartanair_max_ate_m,
            args.tartanair_min_coverage_percent,
        )
    raise ValueError(f"unknown suite {suite}")


def make_jobs(args: argparse.Namespace) -> list[Job]:
    workspace = args.workspace.resolve()
    atlas = workspace / "src/autonomy/autonomy/localization/atlas"
    binaries = workspace / "build/autonomy/bin"
    vocabulary = atlas / "config/orb_vocab.fbow"
    data_root = args.data_root.resolve()
    output_root = args.output_dir.resolve()
    python = sys.executable
    jobs = []

    if "euroc" in args.suites:
        for sequence in EUROC_SEQUENCES:
            dataset = data_root / "EuRoC/data" / sequence
            output = output_root / "euroc" / sequence
            trajectory = output / "trajectory_euroc.txt"
            jobs.append(
                Job(
                    "euroc",
                    sequence,
                    (
                        str(binaries / "atlas_stereo_inertial_euroc"),
                        str(vocabulary),
                        str(atlas / "config/euroc_stereo_inertial.yaml"),
                        str(dataset),
                        str(dataset / "mav0/cam0/data.csv"),
                        str(dataset / "mav0/imu0/data.csv"),
                        str(trajectory),
                    ),
                    (
                        python,
                        str(atlas / "examples/evaluate_euroc.py"),
                        str(dataset / "mav0/state_groundtruth_estimate0/data.csv"),
                        str(trajectory),
                        str(output / "evaluation"),
                        "--max-interval-ms",
                        "20",
                    ),
                    trajectory,
                    output / "evaluation",
                    (
                        binaries / "atlas_stereo_inertial_euroc",
                        vocabulary,
                        atlas / "config/euroc_stereo_inertial.yaml",
                        dataset / "mav0/cam0/data.csv",
                        dataset / "mav0/cam1/data.csv",
                        dataset / "mav0/imu0/data.csv",
                        dataset / "mav0/state_groundtruth_estimate0/data.csv",
                    ),
                    make_thresholds(args, "euroc"),
                )
            )

    if "mono" in args.suites:
        mono_sequences = EUROC_SEQUENCES if args.mono_all else (args.mono_sequence,)
        for sequence in mono_sequences:
            dataset = data_root / "EuRoC/data" / sequence
            output = output_root / "mono" / sequence
            trajectory = output / "trajectory_euroc.txt"
            jobs.append(
                Job(
                    "mono",
                    sequence,
                    (
                        str(binaries / "atlas_mono_euroc"),
                        str(vocabulary),
                        str(atlas / "config/euroc_mono.yaml"),
                        str(dataset),
                        str(dataset / "mav0/cam0/data.csv"),
                        str(trajectory),
                    ),
                    (
                        python,
                        str(atlas / "examples/evaluate_euroc.py"),
                        str(dataset / "mav0/state_groundtruth_estimate0/data.csv"),
                        str(trajectory),
                        str(output / "evaluation"),
                        "--estimate-frame",
                        "camera",
                        "--calibration-yaml",
                        str(dataset / "mav0/cam0/sensor.yaml"),
                        "--alignment",
                        "sim3",
                        "--report-fixed-scale",
                    ),
                    trajectory,
                    output / "evaluation",
                    (
                        binaries / "atlas_mono_euroc",
                        vocabulary,
                        atlas / "config/euroc_mono.yaml",
                        dataset / "mav0/cam0/data.csv",
                        dataset / "mav0/cam0/sensor.yaml",
                        dataset / "mav0/state_groundtruth_estimate0/data.csv",
                    ),
                    make_thresholds(args, "mono"),
                )
            )

    if "kitti" in args.suites:
        base_config = atlas / "config/kitti_00_02.yaml"
        for sequence in KITTI_SEQUENCES:
            dataset = data_root / "kitti/odometry/sequences" / sequence
            output = output_root / "kitti" / sequence
            config = output_root / "generated_configs" / f"kitti_{sequence}.yaml"
            if base_config.is_file() and (dataset / "calib.txt").is_file():
                write_kitti_config(base_config, dataset, config)
            trajectory = output / "trajectory_kitti.txt"
            ground_truth = data_root / "kitti/odometry/poses" / f"{sequence}.txt"
            jobs.append(
                Job(
                    "kitti",
                    sequence,
                    (
                        str(binaries / "atlas_stereo_kitti"),
                        str(vocabulary),
                        str(config),
                        str(dataset),
                        str(trajectory),
                    ),
                    (
                        python,
                        str(atlas / "examples/evaluate_kitti.py"),
                        str(ground_truth),
                        str(trajectory),
                        str(output / "evaluation"),
                    ),
                    trajectory,
                    output / "evaluation",
                    (
                        binaries / "atlas_stereo_kitti",
                        vocabulary,
                        base_config,
                        config,
                        dataset / "calib.txt",
                        dataset / "times.txt",
                        dataset / "image_0",
                        dataset / "image_1",
                        ground_truth,
                    ),
                    make_thresholds(args, "kitti"),
                )
            )

    if "tum_rgbd" in args.suites:
        sequences = discover_tum_rgbd_sequences(data_root)
        if not sequences:
            raise ValueError(
                f"{data_root / 'TUM'}: no TUM RGB-D archives or sequences found"
            )
        base_config = atlas / "config/tum_rgbd.yaml"
        for sequence in sequences:
            dataset = data_root / "TUM" / sequence
            output = output_root / "tum_rgbd" / sequence
            config = output_root / "generated_configs" / f"{sequence}.yaml"
            associations = (
                output_root / "generated_associations" / f"{sequence}.txt"
            )
            if base_config.is_file():
                write_tum_rgbd_config(base_config, sequence, config)
            if (dataset / "rgb.txt").is_file() and (
                dataset / "depth.txt"
            ).is_file():
                write_tum_rgbd_associations(dataset, associations)
            trajectory = output / "trajectory.tum"
            ground_truth = dataset / "groundtruth.txt"
            jobs.append(
                Job(
                    "tum_rgbd",
                    sequence,
                    (
                        str(binaries / "atlas_rgbd_tum"),
                        str(vocabulary),
                        str(config),
                        str(dataset),
                        str(associations),
                        str(trajectory),
                    ),
                    (
                        python,
                        str(atlas / "examples/evaluate_tum_rgbd.py"),
                        str(ground_truth),
                        str(trajectory),
                        str(output / "evaluation"),
                        "--expected-timestamps",
                        str(associations),
                    ),
                    trajectory,
                    output / "evaluation",
                    (
                        binaries / "atlas_rgbd_tum",
                        vocabulary,
                        base_config,
                        config,
                        dataset,
                        dataset / "rgb.txt",
                        dataset / "depth.txt",
                        dataset / "rgb",
                        dataset / "depth",
                        associations,
                        ground_truth,
                        atlas / "examples/evaluate_tum_rgbd.py",
                    ),
                    make_thresholds(args, "tum_rgbd"),
                )
            )

    if "tum_vi" in args.suites:
        dataset = data_root / "TUM-VI/dataset-room1_512_16"
        output = output_root / "tum_vi" / "room1_512_16"
        trajectory = output / "trajectory_euroc.txt"
        jobs.append(
            Job(
                "tum_vi",
                "room1_512_16",
                (
                    str(binaries / "atlas_stereo_inertial_tum_vi"),
                    str(vocabulary),
                    str(atlas / "config/tum_vi_stereo_inertial.yaml"),
                    str(dataset),
                    str(dataset / "mav0/cam0/data.csv"),
                    str(dataset / "mav0/imu0/data.csv"),
                    str(trajectory),
                ),
                (
                    python,
                    str(atlas / "examples/evaluate_euroc.py"),
                    str(dataset / "dso/gt_imu.csv"),
                    str(trajectory),
                    str(output / "evaluation"),
                    "--max-interval-ms",
                    "20",
                ),
                trajectory,
                output / "evaluation",
                (
                    binaries / "atlas_stereo_inertial_tum_vi",
                    vocabulary,
                    atlas / "config/tum_vi_stereo_inertial.yaml",
                    dataset / "mav0/cam0/data.csv",
                    dataset / "mav0/cam1/data.csv",
                    dataset / "mav0/imu0/data.csv",
                    dataset / "dso/gt_imu.csv",
                ),
                make_thresholds(args, "tum_vi"),
            )
        )

    if "tartanair" in args.suites:
        for relative in TARTANAIR_SEQUENCES:
            dataset = data_root / "tartanair_v2" / relative
            sequence = relative.replace("/", "_")
            output = output_root / "tartanair" / sequence
            trajectory = output / "trajectory.tum"
            jobs.append(
                Job(
                    "tartanair",
                    sequence,
                    (
                        str(binaries / "atlas_rgbd_inertial_tartanair"),
                        str(vocabulary),
                        str(atlas / "config/tartanair_rgbd_inertial.yaml"),
                        str(dataset),
                        str(trajectory),
                    ),
                    (
                        python,
                        str(atlas / "examples/evaluate_tartanair.py"),
                        str(dataset / "pose_lcam_front.txt"),
                        str(dataset / "imu/cam_time.txt"),
                        str(trajectory),
                        str(output / "evaluation"),
                    ),
                    trajectory,
                    output / "evaluation",
                    (
                        binaries / "atlas_rgbd_inertial_tartanair",
                        vocabulary,
                        atlas / "config/tartanair_rgbd_inertial.yaml",
                        dataset / "image_lcam_front",
                        dataset / "depth_lcam_front",
                        dataset / "imu/cam_time.txt",
                        dataset / "pose_lcam_front.txt",
                    ),
                    make_thresholds(args, "tartanair"),
                )
            )
    return jobs


def result_from_metrics(job: Job, metrics: dict[str, Any], resumed: bool) -> Result:
    violations, ate, coverage, translation, rotation = assess_metrics(
        metrics, job.thresholds
    )
    return Result(
        job.suite,
        job.sequence,
        "threshold_failed" if violations else "passed",
        resumed=resumed,
        ate_rmse_m=ate,
        coverage_percent=coverage,
        translation_drift_percent=translation,
        rotation_drift_deg_per_100m=rotation,
        violations=violations,
    )


def execute_job(
    job: Job, resume: bool, environment: dict[str, str]
) -> Result:
    job.output_dir.mkdir(parents=True, exist_ok=True)
    missing = [str(path) for path in job.required_paths if not path.exists()]
    if missing:
        return Result(
            job.suite,
            job.sequence,
            "missing_data",
            message="missing: " + ", ".join(missing),
        )
    metrics_path = job.evaluation_dir / "metrics.json"
    if resume and metrics_path.is_file():
        try:
            return result_from_metrics(job, load_metrics(metrics_path), True)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            pass

    start = time.monotonic()
    run_exit_code = None
    if not (resume and job.trajectory.is_file() and job.trajectory.stat().st_size > 0):
        run_exit_code = run_command(
            job.command, job.output_dir / "run.log", environment
        )
        if run_exit_code != 0:
            return Result(
                job.suite,
                job.sequence,
                "run_failed",
                elapsed_seconds=time.monotonic() - start,
                run_exit_code=run_exit_code,
            )

    evaluation_exit_code = run_command(
        job.evaluation_command,
        job.output_dir / "evaluate.log",
        environment,
    )
    elapsed = time.monotonic() - start
    if evaluation_exit_code != 0 or not metrics_path.is_file():
        return Result(
            job.suite,
            job.sequence,
            "evaluation_failed",
            elapsed_seconds=elapsed,
            run_exit_code=run_exit_code,
            evaluation_exit_code=evaluation_exit_code,
        )
    result = result_from_metrics(job, load_metrics(metrics_path), False)
    result.elapsed_seconds = elapsed
    result.run_exit_code = run_exit_code
    result.evaluation_exit_code = evaluation_exit_code
    return result


def inventory_tum_rgbd(data_root: Path, output_dir: Path) -> list[dict[str, Any]]:
    root = data_root / "TUM"
    records = []
    for sequence in discover_tum_rgbd_sequences(data_root):
        archive = root / f"{sequence}.tgz"
        dataset = root / sequence
        required = (
            dataset / "rgb.txt",
            dataset / "depth.txt",
            dataset / "groundtruth.txt",
            dataset / "rgb",
            dataset / "depth",
        )
        missing = [
            str(path.relative_to(root)) for path in required if not path.exists()
        ]
        records.append(
            {
                "sequence": sequence,
                "archive": str(archive) if archive.is_file() else None,
                "archive_bytes": archive.stat().st_size if archive.is_file() else None,
                "extracted_directory": str(dataset),
                "extracted_directory_present": dataset.is_dir(),
                "ready": dataset.is_dir() and not missing,
                "missing_components": missing,
            }
        )
    write_json(output_dir / "tum_rgbd_archive_inventory.json", records)
    return records


def write_summary(output_dir: Path, results: Iterable[Result]) -> None:
    rows = [dataclasses.asdict(result) for result in results]
    write_json(output_dir / "summary.json", rows)
    fields = [field.name for field in dataclasses.fields(Result)]
    with (output_dir / "summary.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            row["violations"] = "; ".join(row["violations"])
            writer.writerow(row)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path("/workspace/autonomy"))
    parser.add_argument("--data-root", type=Path, default=Path("/data/slam"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--suites", type=parse_suites, default=DEFAULT_SUITES)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--stop-on-failure", action="store_true")
    parser.add_argument("--mono-all", action="store_true")
    parser.add_argument(
        "--mono-sequence", choices=EUROC_SEQUENCES, default="MH_01_easy"
    )
    parser.add_argument("--omp-threads", type=int, default=2)
    parser.add_argument("--euroc-max-ate-m", type=finite_positive, default=0.15)
    parser.add_argument(
        "--euroc-min-coverage-percent", type=finite_percentage, default=90.0
    )
    parser.add_argument("--kitti-max-ate-m", type=finite_positive, default=5.0)
    parser.add_argument(
        "--kitti-max-translation-drift-percent",
        type=finite_positive,
        default=5.0,
    )
    parser.add_argument(
        "--kitti-max-rotation-drift-deg-per-100m",
        type=finite_positive,
        default=2.0,
    )
    parser.add_argument("--tum-rgbd-max-ate-m", type=finite_positive, default=0.15)
    parser.add_argument(
        "--tum-rgbd-min-coverage-percent",
        type=finite_percentage,
        default=90.0,
    )
    parser.add_argument("--tum-vi-max-ate-m", type=finite_positive, default=0.15)
    parser.add_argument(
        "--tum-vi-min-coverage-percent", type=finite_percentage, default=90.0
    )
    parser.add_argument(
        "--tartanair-max-ate-m", type=finite_positive, default=0.25
    )
    parser.add_argument(
        "--tartanair-min-coverage-percent",
        type=finite_percentage,
        default=90.0,
    )
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()
    if args.omp_threads <= 0:
        raise SystemExit("--omp-threads must be positive")
    args.output_dir = args.output_dir.resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    tum_rgbd_inventory = inventory_tum_rgbd(
        args.data_root.resolve(), args.output_dir
    )
    ready_tum_rgbd = sum(record["ready"] for record in tum_rgbd_inventory)
    print(
        "TUM RGB-D inventory: "
        f"{ready_tum_rgbd}/{len(tum_rgbd_inventory)} sequences ready; "
        "archives are never extracted implicitly"
    )

    try:
        jobs = make_jobs(args)
    except (OSError, ValueError) as error:
        print(f"benchmark setup failed: {error}", file=sys.stderr)
        return 2
    environment = os.environ.copy()
    environment["OMP_NUM_THREADS"] = str(args.omp_threads)
    environment["OPENBLAS_NUM_THREADS"] = "1"
    environment["MKL_NUM_THREADS"] = "1"
    results = []
    for index, job in enumerate(jobs, 1):
        print(f"[{index}/{len(jobs)}] START {job.suite}/{job.sequence}", flush=True)
        try:
            result = execute_job(job, args.resume, environment)
        except (
            OSError,
            KeyError,
            TypeError,
            ValueError,
            json.JSONDecodeError,
        ) as error:
            result = Result(
                job.suite,
                job.sequence,
                "internal_error",
                message=str(error),
            )
        results.append(result)
        write_json(job.output_dir / "status.json", dataclasses.asdict(result))
        write_summary(args.output_dir, results)
        print(
            f"[{index}/{len(jobs)}] END {job.suite}/{job.sequence}: "
            f"{result.status} ATE={result.ate_rmse_m} "
            f"coverage={result.coverage_percent}",
            flush=True,
        )
        if args.stop_on_failure and result.status != "passed":
            break
    write_summary(args.output_dir, results)
    return 0 if results and all(result.status == "passed" for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
