#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors
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

"""Download SLAM datasets used by Atlas / Argus into the /data layout.

Default destination is $AUTONOMY_DATASETS, then /data:

  <dest>/slam/EuRoC/data/<seq>/mav0/...
  <dest>/slam/kitti/odometry/sequences/<id>/
  <dest>/slam/TUM/rgbd_dataset_freiburg1_desk/
  <dest>/slam/TUM-VI/dataset-room1_512_16/
  <dest>/Oxford/sequences/<seq>/raw/{imu.csv,lidar-clouds/}

Spires archives are large. The default Spires pack is IMU + Velodyne zips +
ground truth (no camera images). Pass --spires-images to add images.zip.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tarfile
import urllib.error
import urllib.request
import zipfile
from pathlib import Path

EUROC_BASE = (
    "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
)
EUROC_ARCHIVES = {
    "MH_01_easy": f"{EUROC_BASE}/machine_hall/MH_01_easy/MH_01_easy.zip",
    "MH_02_easy": f"{EUROC_BASE}/machine_hall/MH_02_easy/MH_02_easy.zip",
    "MH_03_medium": f"{EUROC_BASE}/machine_hall/MH_03_medium/MH_03_medium.zip",
    "MH_04_difficult": (
        f"{EUROC_BASE}/machine_hall/MH_04_difficult/MH_04_difficult.zip"
    ),
    "MH_05_difficult": (
        f"{EUROC_BASE}/machine_hall/MH_05_difficult/MH_05_difficult.zip"
    ),
    "V1_01_easy": f"{EUROC_BASE}/vicon_room1/V1_01_easy/V1_01_easy.zip",
    "V1_02_medium": f"{EUROC_BASE}/vicon_room1/V1_02_medium/V1_02_medium.zip",
    "V1_03_difficult": (
        f"{EUROC_BASE}/vicon_room1/V1_03_difficult/V1_03_difficult.zip"
    ),
    "V2_01_easy": f"{EUROC_BASE}/vicon_room2/V2_01_easy/V2_01_easy.zip",
    "V2_02_medium": f"{EUROC_BASE}/vicon_room2/V2_02_medium/V2_02_medium.zip",
    "V2_03_difficult": (
        f"{EUROC_BASE}/vicon_room2/V2_03_difficult/V2_03_difficult.zip"
    ),
}

KITTI_FILES = {
    "data_odometry_gray.zip": (
        "https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_gray.zip"
    ),
    "data_odometry_poses.zip": (
        "https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_poses.zip"
    ),
    "data_odometry_calib.zip": (
        "https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_calib.zip"
    ),
}
KITTI_VELODYNE = (
    "https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_velodyne.zip"
)

TUM_RGBD_BASE = "https://vision.in.tum.de/rgbd/dataset"
TUM_RGBD_SEQUENCES = (
    "freiburg1/rgbd_dataset_freiburg1_desk",
    "freiburg1/rgbd_dataset_freiburg1_desk2",
    "freiburg1/rgbd_dataset_freiburg1_room",
    "freiburg1/rgbd_dataset_freiburg1_xyz",
    "freiburg1/rgbd_dataset_freiburg1_rpy",
    "freiburg1/rgbd_dataset_freiburg1_360",
    "freiburg1/rgbd_dataset_freiburg1_floor",
    "freiburg1/rgbd_dataset_freiburg1_plant",
    "freiburg1/rgbd_dataset_freiburg1_teddy",
    "freiburg1/rgbd_dataset_freiburg1_person",
    "freiburg2/rgbd_dataset_freiburg2_desk",
    "freiburg3/rgbd_dataset_freiburg3_long_office_household",
)

TUM_VI_512_BASE = (
    "https://vision.in.tum.de/tumvi/exported/euroc/512_16"
)
TUM_VI_ROOM = tuple(f"dataset-room{i}_512_16" for i in range(1, 7))
TUM_VI_EXTRA = (
    tuple(f"dataset-corridor{i}_512_16" for i in range(1, 6))
    + tuple(f"dataset-magistrale{i}_512_16" for i in range(1, 7))
    + tuple(f"dataset-outdoors{i}_512_16" for i in range(1, 9))
    + tuple(f"dataset-slides{i}_512_16" for i in range(1, 4))
)

SPIRES_SEQUENCES = (
    "2024-03-12-keble-college-01",
    "2024-03-12-keble-college-02",
    "2024-03-12-keble-college-03",
    "2024-03-12-keble-college-04",
    "2024-03-12-keble-college-05",
    "2024-03-13-observatory-quarter-01",
    "2024-03-13-observatory-quarter-02",
    "2024-03-14-blenheim-palace-01",
    "2024-03-14-blenheim-palace-02",
    "2024-03-14-blenheim-palace-03",
    "2024-03-14-blenheim-palace-04",
    "2024-03-14-blenheim-palace-05",
    "2024-03-15-bodleian-library-01",
    "2024-03-18-christ-church-01",
    "2024-03-18-christ-church-02",
    "2024-03-18-christ-church-03",
    "2024-03-18-christ-church-04",
    "2024-03-20-christ-church-05",
    "2024-03-20-christ-church-06",
    "2024-05-20-bodleian-library-02",
    "2024-07-09-new-college-01",
    "2024-07-09-new-college-02",
    "2024-07-09-new-college-03",
    "2024-07-09-new-college-04",
)
SPIRES_REPO = "ori-drs/oxford_spires_dataset"

ALL_GROUPS = ("euroc", "kitti", "tum-rgbd", "tum-vi", "spires")
USER_AGENT = "autonomy-localization-datasets/1.0"


def default_dest() -> Path:
    env = os.environ.get("AUTONOMY_DATASETS", "").strip()
    if env:
        return Path(env).expanduser()
    data = Path("/data")
    if data.is_dir():
        return data
    return Path("/data")


def log(message: str) -> None:
    print(message, flush=True)


def download_file(url: str, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.is_file() and dest.stat().st_size > 0:
        log(f"  skip existing {dest}")
        return
    part = dest.with_name(dest.name + ".part")
    offset = part.stat().st_size if part.is_file() else 0
    request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    if offset > 0:
        request.add_header("Range", f"bytes={offset}-")
        log(f"  resume {dest.name} @ {offset} bytes")
    else:
        log(f"  get {url}")
    try:
        with urllib.request.urlopen(request, timeout=120) as response:
            mode = "ab" if offset > 0 and response.status == 206 else "wb"
            if mode == "wb":
                offset = 0
            with part.open(mode) as handle:
                copied = offset
                while True:
                    chunk = response.read(1024 * 1024)
                    if not chunk:
                        break
                    handle.write(chunk)
                    copied += len(chunk)
                    if copied == offset or copied % (32 * 1024 * 1024) < len(
                        chunk
                    ):
                        log(f"    {dest.name}: {copied / (1024 ** 3):.2f} GiB")
    except urllib.error.HTTPError as error:
        raise RuntimeError(f"download failed {url}: HTTP {error.code}") from error
    part.replace(dest)


def extract_zip(archive: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    log(f"  unzip {archive.name} -> {destination}")
    with zipfile.ZipFile(archive) as zipped:
        zipped.extractall(destination)


def extract_tar(archive: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    log(f"  untar {archive.name} -> {destination}")
    with tarfile.open(archive) as tar:
        tar.extractall(destination)


def ensure_euroc_layout(sequence_dir: Path) -> None:
    mav0 = sequence_dir / "mav0"
    if mav0.is_dir():
        return
    nested = sequence_dir / sequence_dir.name
    if (nested / "mav0").is_dir():
        for child in nested.iterdir():
            target = sequence_dir / child.name
            if not target.exists():
                child.rename(target)
        shutil.rmtree(nested, ignore_errors=True)


def merge_kitti_tree(extracted: Path, odometry: Path) -> None:
    sequences = extracted / "dataset" / "sequences"
    poses = extracted / "dataset" / "poses"
    if sequences.is_dir():
        dest_seq = odometry / "sequences"
        dest_seq.mkdir(parents=True, exist_ok=True)
        for item in sequences.iterdir():
            target = dest_seq / item.name
            if item.is_dir() and not target.exists():
                shutil.move(str(item), str(target))
            elif item.is_dir():
                for inner in item.iterdir():
                    inner_target = target / inner.name
                    if not inner_target.exists():
                        shutil.move(str(inner), str(inner_target))
    if poses.is_dir():
        dest_poses = odometry / "poses"
        dest_poses.mkdir(parents=True, exist_ok=True)
        for item in poses.iterdir():
            target = dest_poses / item.name
            if not target.exists():
                shutil.move(str(item), str(target))


def split_csv(value: str | None) -> tuple[str, ...]:
    if not value:
        return ()
    return tuple(item.strip() for item in value.split(",") if item.strip())


def _fail_unknown(kind: str, token: str, choices: tuple[str, ...]) -> None:
    sample = ", ".join(choices[:8])
    extra = " ..." if len(choices) > 8 else ""
    raise RuntimeError(
        f"unknown {kind} sequence {token!r}. Known: {sample}{extra}"
    )


def resolve_euroc(selected: tuple[str, ...]) -> dict[str, str]:
    if not selected:
        return dict(EUROC_ARCHIVES)
    resolved: dict[str, str] = {}
    for token in selected:
        if token in EUROC_ARCHIVES:
            resolved[token] = EUROC_ARCHIVES[token]
            continue
        matches = [
            name
            for name in EUROC_ARCHIVES
            if name == token
            or name.startswith(token + "_")
            or name.startswith(token)
        ]
        if len(matches) != 1:
            _fail_unknown("euroc", token, tuple(EUROC_ARCHIVES))
        resolved[matches[0]] = EUROC_ARCHIVES[matches[0]]
    return resolved


def resolve_tum_rgbd(selected: tuple[str, ...]) -> tuple[str, ...]:
    if not selected:
        return TUM_RGBD_SEQUENCES
    resolved: list[str] = []
    for token in selected:
        if token in TUM_RGBD_SEQUENCES:
            resolved.append(token)
            continue
        matches = [
            relative
            for relative in TUM_RGBD_SEQUENCES
            if relative == token
            or relative.endswith("/" + token)
            or relative.split("/", 1)[-1] == token
            or relative.split("/", 1)[-1].replace("rgbd_dataset_", "") == token
        ]
        unique = list(dict.fromkeys(matches))
        if len(unique) != 1:
            names = tuple(item.split("/", 1)[-1] for item in TUM_RGBD_SEQUENCES)
            _fail_unknown("tum-rgbd", token, names)
        resolved.append(unique[0])
    return tuple(resolved)


def tum_vi_catalog() -> tuple[str, ...]:
    return TUM_VI_ROOM + TUM_VI_EXTRA


def _tum_vi_short(name: str) -> str:
    return name.replace("dataset-", "").replace("_512_16", "")


def resolve_tum_vi(selected: tuple[str, ...], extra: bool) -> tuple[str, ...]:
    default = tum_vi_catalog() if extra else TUM_VI_ROOM
    if not selected:
        return default
    catalog = tum_vi_catalog()
    resolved: list[str] = []
    for token in selected:
        if token in catalog:
            resolved.append(token)
            continue
        matches = [
            item
            for item in catalog
            if item == token or _tum_vi_short(item) == token
        ]
        unique = list(dict.fromkeys(matches))
        if len(unique) != 1:
            _fail_unknown("tum-vi", token, catalog)
        resolved.append(unique[0])
    return tuple(dict.fromkeys(resolved))


def resolve_spires(selected: tuple[str, ...]) -> tuple[str, ...]:
    if not selected:
        return SPIRES_SEQUENCES
    resolved: list[str] = []
    for token in selected:
        if token in SPIRES_SEQUENCES:
            resolved.append(token)
            continue
        matches = [
            name
            for name in SPIRES_SEQUENCES
            if name == token or name.endswith("-" + token)
        ]
        unique = list(dict.fromkeys(matches))
        if len(unique) != 1:
            _fail_unknown("spires", token, SPIRES_SEQUENCES)
        resolved.append(unique[0])
    return tuple(resolved)


def download_euroc(root: Path, archives_map: dict[str, str]) -> None:
    dest = root / "slam" / "EuRoC" / "data"
    archives = dest / "_archives"
    for name, url in archives_map.items():
        sequence = dest / name
        if (sequence / "mav0" / "cam0" / "data.csv").is_file():
            log(f"[euroc] {name} already present")
            continue
        archive = archives / f"{name}.zip"
        download_file(url, archive)
        extract_zip(archive, sequence)
        ensure_euroc_layout(sequence)
        log(f"[euroc] ready {sequence}")


def download_kitti(root: Path, velodyne: bool) -> None:
    odometry = root / "slam" / "kitti" / "odometry"
    archives = odometry / "_archives"
    files = dict(KITTI_FILES)
    if velodyne:
        files["data_odometry_velodyne.zip"] = KITTI_VELODYNE
    for name, url in files.items():
        archive = archives / name
        download_file(url, archive)
        staging = archives / (name + ".extract")
        if staging.exists():
            shutil.rmtree(staging)
        extract_zip(archive, staging)
        merge_kitti_tree(staging, odometry)
        shutil.rmtree(staging, ignore_errors=True)
    seq00 = odometry / "sequences" / "00" / "image_0"
    if not seq00.is_dir():
        raise RuntimeError("KITTI extract did not produce sequences/00/image_0")
    log(f"[kitti] ready {odometry}")


def download_tum_rgbd(root: Path, sequences: tuple[str, ...]) -> None:
    dest = root / "slam" / "TUM"
    archives = dest / "_archives"
    for relative in sequences:
        family, sequence = relative.split("/", 1)
        extracted = dest / sequence
        if (extracted / "rgb.txt").is_file() and (extracted / "depth.txt").is_file():
            log(f"[tum-rgbd] {sequence} already present")
            continue
        archive = archives / f"{sequence}.tgz"
        url = f"{TUM_RGBD_BASE}/{family}/{sequence}.tgz"
        download_file(url, archive)
        extract_tar(archive, dest)
        log(f"[tum-rgbd] ready {extracted}")


def download_tum_vi(root: Path, names: tuple[str, ...]) -> None:
    dest = root / "slam" / "TUM-VI"
    archives = dest / "_archives"
    for name in names:
        sequence = dest / name
        if (sequence / "mav0" / "cam0" / "data.csv").is_file():
            log(f"[tum-vi] {name} already present")
            continue
        archive = archives / f"{name}.tar"
        download_file(f"{TUM_VI_512_BASE}/{name}.tar", archive)
        extract_tar(archive, dest)
        log(f"[tum-vi] ready {sequence}")


def ensure_huggingface_hub() -> None:
    try:
        import huggingface_hub  # noqa: F401
    except ImportError:
        log("[spires] installing huggingface_hub")
        subprocess.check_call(
            [sys.executable, "-m", "pip", "install", "--user", "huggingface_hub"]
        )


def unzip_if_needed(archive: Path, destination: Path) -> None:
    if destination.is_dir() and any(destination.iterdir()):
        return
    if not archive.is_file():
        return
    extract_zip(archive, destination)


def download_spires(
    root: Path, with_images: bool, sequences: tuple[str, ...]
) -> None:
    ensure_huggingface_hub()
    from huggingface_hub import snapshot_download

    dest = root / "Oxford"
    dest.mkdir(parents=True, exist_ok=True)
    patterns = ["calibration/*"]
    for sequence in sequences:
        raw = dest / "sequences" / sequence / "raw"
        if (raw / "imu.csv").is_file() and (
            (raw / "lidar-clouds").is_dir() or (raw / "lidar-clouds.zip").is_file()
        ):
            log(f"[spires] {sequence} already present")
            continue
        patterns.extend(
            [
                f"sequences/{sequence}/raw/imu.csv",
                f"sequences/{sequence}/raw/lidar-clouds.zip",
                f"sequences/{sequence}/raw/lidar-clouds/*",
                f"sequences/{sequence}/processed/vilens-slam/gt.tum",
                f"sequences/{sequence}/processed/trajectory/gt-tum.txt",
            ]
        )
        if with_images:
            patterns.extend(
                [
                    f"sequences/{sequence}/raw/images.zip",
                    f"sequences/{sequence}/raw/cam0/*",
                    f"sequences/{sequence}/raw/cam1/*",
                    f"sequences/{sequence}/raw/cam2/*",
                ]
            )
    if len(patterns) == 1:
        log("[spires] all sequences already present")
        return
    log("[spires] Hugging Face snapshot (LIO pack; images optional)")
    snapshot_download(
        repo_id=SPIRES_REPO,
        repo_type="dataset",
        local_dir=str(dest),
        allow_patterns=patterns,
    )
    for sequence in sequences:
        raw = dest / "sequences" / sequence / "raw"
        unzip_if_needed(raw / "lidar-clouds.zip", raw / "lidar-clouds")
        if with_images:
            unzip_if_needed(raw / "images.zip", raw)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dest",
        type=Path,
        default=default_dest(),
        help="Dataset root (default: $AUTONOMY_DATASETS or /data)",
    )
    parser.add_argument(
        "--datasets",
        default=None,
        metavar="GROUPS",
        help="Comma groups: euroc,kitti,tum-rgbd,tum-vi,spires "
        "(default: all, or only groups named by sequence flags)",
    )
    parser.add_argument(
        "--euroc",
        default=None,
        metavar="SEQ",
        help="EuRoC sequences, e.g. MH_01_easy,V1_01_easy (also MH_01)",
    )
    parser.add_argument(
        "--tum-rgbd",
        default=None,
        metavar="SEQ",
        help="TUM RGB-D sequences, e.g. freiburg1_desk,freiburg2_desk",
    )
    parser.add_argument(
        "--tum-vi",
        default=None,
        metavar="SEQ",
        help="TUM-VI sequences, e.g. room1,corridor1",
    )
    parser.add_argument(
        "--spires",
        default=None,
        metavar="SEQ",
        help="Spires sequences, e.g. keble-college-02,christ-church-01",
    )
    parser.add_argument(
        "--kitti-velodyne",
        action="store_true",
        help="Also download KITTI Velodyne bins (~80 GiB extra)",
    )
    parser.add_argument(
        "--tum-vi-full",
        action="store_true",
        help="TUM-VI 512: rooms plus corridor/magistrale/outdoors/slides",
    )
    parser.add_argument(
        "--spires-images",
        action="store_true",
        help="Also download Oxford Spires camera zips",
    )
    parser.add_argument(
        "--lite",
        action="store_true",
        help="Smoke set: MH_01, KITTI gray/poses/calib, f1_desk, TUM-VI room1, "
        "Spires keble-college-02",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="Print available groups and sequence names, then exit",
    )
    return parser.parse_args()


def list_catalog() -> None:
    log("groups: " + ", ".join(ALL_GROUPS))
    log("euroc:")
    for name in EUROC_ARCHIVES:
        log(f"  {name}")
    log("kitti: data_odometry_{gray,poses,calib}.zip  (+ --kitti-velodyne)")
    log("tum-rgbd:")
    for relative in TUM_RGBD_SEQUENCES:
        log(f"  {relative.split('/', 1)[-1]}")
    log("tum-vi:")
    for name in tum_vi_catalog():
        log(f"  {name}  ({_tum_vi_short(name)})")
    log("spires:")
    for name in SPIRES_SEQUENCES:
        log(f"  {name}")


def selected_groups(args: argparse.Namespace) -> tuple[str, ...]:
    sequence_flags = {
        "euroc": args.euroc,
        "tum-rgbd": args.tum_rgbd,
        "tum-vi": args.tum_vi,
        "spires": args.spires,
    }
    if args.datasets is not None:
        selected = split_csv(args.datasets)
    elif any(sequence_flags.values()) and not args.lite:
        selected = tuple(
            group for group, value in sequence_flags.items() if value
        )
    else:
        selected = ALL_GROUPS
    unknown = [item for item in selected if item not in ALL_GROUPS]
    if unknown:
        raise RuntimeError(f"unknown dataset groups: {unknown}")
    return selected


def main() -> int:
    args = parse_args()
    if args.list:
        list_catalog()
        return 0
    euroc_filter = split_csv(args.euroc)
    tum_rgbd_filter = split_csv(args.tum_rgbd)
    tum_vi_filter = split_csv(args.tum_vi)
    spires_filter = split_csv(args.spires)
    if args.lite:
        if not euroc_filter:
            euroc_filter = ("MH_01_easy",)
        if not tum_rgbd_filter:
            tum_rgbd_filter = ("freiburg1_desk",)
        if not tum_vi_filter:
            tum_vi_filter = ("room1",)
        if not spires_filter:
            spires_filter = ("keble-college-02",)
    dest = args.dest.expanduser().resolve()
    dest.mkdir(parents=True, exist_ok=True)
    if not os.access(dest, os.W_OK):
        log(f"destination is not writable: {dest}")
        log("Set AUTONOMY_DATASETS to a writable host path and recreate SpaceHero.")
        return 2
    try:
        selected = selected_groups(args)
        euroc_map = resolve_euroc(euroc_filter)
        tum_rgbd_seq = resolve_tum_rgbd(tum_rgbd_filter)
        tum_vi_seq = resolve_tum_vi(tum_vi_filter, extra=args.tum_vi_full)
        spires_seq = resolve_spires(spires_filter)
    except RuntimeError as error:
        log(str(error))
        return 2
    log(f"destination {dest}")
    log(f"groups {', '.join(selected)}")
    log("This can be tens to hundreds of GiB. Existing files are skipped.")
    try:
        if "euroc" in selected:
            download_euroc(dest, euroc_map)
        if "kitti" in selected:
            download_kitti(dest, velodyne=args.kitti_velodyne)
        if "tum-rgbd" in selected:
            download_tum_rgbd(dest, tum_rgbd_seq)
        if "tum-vi" in selected:
            download_tum_vi(dest, tum_vi_seq)
        if "spires" in selected:
            download_spires(dest, args.spires_images, spires_seq)
    except (RuntimeError, OSError, urllib.error.URLError) as error:
        log(f"failed: {error}")
        return 1
    log("done")
    log(f"  EuRoC     {dest}/slam/EuRoC/data")
    log(f"  KITTI     {dest}/slam/kitti/odometry")
    log(f"  TUM RGB-D {dest}/slam/TUM")
    log(f"  TUM-VI    {dest}/slam/TUM-VI")
    log(f"  Spires    {dest}/Oxford/sequences")
    return 0


if __name__ == "__main__":
    sys.exit(main())
