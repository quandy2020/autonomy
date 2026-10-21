# Copyright 2026 The Openbot Authors (duyongquan)
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

"""Oxford Spires raw sequence reader (imu.csv + lidar-clouds + optional cam)."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import numpy as np

from autonomy.tools.datasets.pcd_io import load_pcd_xyzi


@dataclass
class ImuSample:
    t: float
    acc: tuple[float, float, float]
    gyr: tuple[float, float, float]


@dataclass
class SpiresFrame:
    index: int
    timestamp: float
    xyz: np.ndarray
    intensity: np.ndarray
    # Sweep-relative time in milliseconds (Hesai timestamp field), if known.
    time_ms: np.ndarray | None = None
    image: np.ndarray | None = None
    image_timestamp: float = 0.0
    cloud_path: str = ""
    image_path: str = ""


def stem_to_seconds(stem: str) -> float:
    """Parse a filename stem as seconds (ns / us / sec heuristics)."""
    if not stem:
        return 0.0
    try:
        value = float(stem)
    except ValueError:
        return 0.0
    if "." not in stem:
        if value >= 1e15:
            return value * 1e-9
        if value >= 1e12:
            return value * 1e-6
    return value


def list_stems(directory: Path, *exts: str) -> list[str]:
    if not directory.is_dir():
        return []
    stems: list[str] = []
    wanted = {e.lower() for e in exts}
    for path in directory.iterdir():
        if path.is_file() and path.suffix.lower() in wanted:
            stems.append(path.stem)
    stems.sort()
    return stems


def nearest_index(times: list[float], t: float, max_dt: float) -> int:
    if not times:
        return -1
    lo, hi = 0, len(times)
    while lo < hi:
        mid = (lo + hi) // 2
        if times[mid] < t:
            lo = mid + 1
        else:
            hi = mid
    best = -1
    # Float slack so boundary matches (e.g. |Δt| == max_dt) still associate.
    best_dt = float(max_dt) + 1e-9
    for i in (lo - 1, lo):
        if 0 <= i < len(times):
            dt = abs(times[i] - t)
            if dt <= best_dt:
                best_dt = dt
                best = i
    return best


def load_imu_csv(path: Path) -> list[ImuSample]:
    """Load ``secs,nsecs,acc_*,ang_vel_*`` IMU CSV (Oxford Spires layout)."""
    samples: list[ImuSample] = []
    with path.open("r", newline="") as fh:
        reader = csv.reader(fh)
        header = next(reader, None)
        if header is None:
            return samples
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            if len(row) < 8:
                continue
            try:
                sec = float(row[0])
                nsec = float(row[1])
                ax, ay, az = float(row[2]), float(row[3]), float(row[4])
                wx, wy, wz = float(row[5]), float(row[6]), float(row[7])
            except ValueError:
                continue
            samples.append(
                ImuSample(
                    t=sec + nsec * 1e-9,
                    acc=(ax, ay, az),
                    gyr=(wx, wy, wz),
                )
            )
    return samples


def _load_image_rgb(path: Path) -> np.ndarray | None:
    try:
        from PIL import Image as PilImage

        return np.asarray(PilImage.open(path).convert("RGB"))
    except Exception:
        pass
    try:
        import cv2

        bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    except Exception:
        return None


class SpiresReader:
    """Sequential reader for Oxford Spires ``raw/`` sequences.

    Accepts sequence root, ``.../raw``, or a folder that already contains
    ``lidar-clouds/``. Camera is optional (LIO-only publish).
    """

    def __init__(
        self,
        root: str | Path,
        *,
        camera: str = "cam0",
        max_image_dt_sec: float = 0.05,
        prefer_undistorted: bool = False,
        load_images: bool = True,
    ) -> None:
        self.root = Path(root).resolve()
        self.camera = camera
        self.max_image_dt_sec = float(max_image_dt_sec)
        self.prefer_undistorted = prefer_undistorted
        self.load_images = load_images

        self.raw_dir: Path
        self.seq_root: Path
        self.lidar_dir: Path
        self.image_dir: Path | None = None
        self.imu_path: Path
        self.lidar_stems: list[str] = []
        self.lidar_times: list[float] = []
        self.image_stems: list[str] = []
        self.image_times: list[float] = []
        self.imu: list[ImuSample] = []

        self._resolve()
        self._index()

    def _resolve(self) -> None:
        r = self.root
        if not r.exists():
            raise FileNotFoundError(
                f"dataset path does not exist: {r}\n"
                "  Inside Docker, set AUTONOMY_DATASETS on the host and recreate:\n"
                "    export AUTONOMY_DATASETS=/path/to/datasets\n"
                "    AUTONOMY_FORCE_RECREATE=1 python3 docker/run_autonomy.py\n"
                "  then pass --path /data/Oxford/sequences/SEQ_NAME"
            )
        if (r / "lidar-clouds").is_dir():
            self.raw_dir = r
            self.seq_root = r.parent if r.name == "raw" else r
        elif (r / "raw" / "lidar-clouds").is_dir():
            self.raw_dir = r / "raw"
            self.seq_root = r
        else:
            raise FileNotFoundError(
                f"unsupported Spires layout (need lidar-clouds or raw/lidar-clouds): {r}\n"
                f"  exists={r.exists()} contents={list(r.iterdir())[:8] if r.is_dir() else []}"
            )

        lidar = self.raw_dir / "lidar-clouds"
        if self.prefer_undistorted:
            undist = self.seq_root / "processed" / "vilens-slam" / "undist-clouds"
            if undist.is_dir() and any(undist.glob("*.pcd")):
                lidar = undist
        self.lidar_dir = lidar

        cam = self.raw_dir / self.camera
        nested = cam / "data"
        if nested.is_dir():
            self.image_dir = nested
        elif cam.is_dir():
            self.image_dir = cam
        else:
            self.image_dir = None

        self.imu_path = self.raw_dir / "imu.csv"
        if not self.imu_path.is_file():
            raise FileNotFoundError(f"missing imu.csv: {self.imu_path}")

    def _index(self) -> None:
        self.lidar_stems = list_stems(self.lidar_dir, ".pcd")
        if not self.lidar_stems:
            raise FileNotFoundError(f"no PCD under {self.lidar_dir}")
        self.lidar_times = [stem_to_seconds(s) for s in self.lidar_stems]

        self.image_stems = []
        self.image_times = []
        if self.load_images and self.image_dir is not None:
            for ext in (".jpg", ".jpeg", ".png"):
                stems = list_stems(self.image_dir, ext)
                if stems:
                    self.image_stems = stems
                    break
            self.image_times = [stem_to_seconds(s) for s in self.image_stems]

        self.imu = load_imu_csv(self.imu_path)
        if not self.imu:
            raise FileNotFoundError(f"empty imu.csv: {self.imu_path}")

    def __len__(self) -> int:
        return len(self.lidar_stems)

    def info(self) -> dict:
        t0 = self.lidar_times[0] if self.lidar_times else 0.0
        t1 = self.lidar_times[-1] if self.lidar_times else 0.0
        return {
            "seq_root": str(self.seq_root),
            "raw_dir": str(self.raw_dir),
            "lidar_dir": str(self.lidar_dir),
            "image_dir": str(self.image_dir) if self.image_dir else "",
            "imu_path": str(self.imu_path),
            "lidar_frames": len(self.lidar_stems),
            "image_frames": len(self.image_stems),
            "imu_samples": len(self.imu),
            "duration_sec": float(t1 - t0),
            "camera": self.camera,
        }

    def _cloud_path(self, stem: str) -> Path:
        path = self.lidar_dir / f"{stem}.pcd"
        if path.is_file():
            return path
        matches = list(self.lidar_dir.glob(f"{stem}.*"))
        if matches:
            return matches[0]
        raise FileNotFoundError(f"missing cloud for stem {stem}")

    def _image_path(self, stem: str) -> Path | None:
        if self.image_dir is None:
            return None
        for ext in (".jpg", ".jpeg", ".png"):
            path = self.image_dir / f"{stem}{ext}"
            if path.is_file():
                return path
        return None

    def read(self, index: int, *, load_image: bool | None = None) -> SpiresFrame:
        if index < 0 or index >= len(self.lidar_stems):
            raise IndexError(index)
        want_image = self.load_images if load_image is None else load_image
        stem = self.lidar_stems[index]
        cloud_path = self._cloud_path(stem)
        header_t = self.lidar_times[index]
        xyz, intensity, time_ms = load_pcd_xyzi(
            cloud_path, header_time=header_t
        )
        frame = SpiresFrame(
            index=index,
            timestamp=header_t,
            xyz=xyz,
            intensity=intensity,
            time_ms=time_ms,
            cloud_path=str(cloud_path),
        )
        if want_image and self.image_times:
            j = nearest_index(
                self.image_times, frame.timestamp, self.max_image_dt_sec
            )
            if j >= 0:
                img_path = self._image_path(self.image_stems[j])
                if img_path is not None:
                    rgb = _load_image_rgb(img_path)
                    if rgb is not None:
                        frame.image = rgb
                        frame.image_timestamp = self.image_times[j]
                        frame.image_path = str(img_path)
        return frame

    def iter_frames(
        self,
        *,
        start: int = 0,
        max_frames: int | None = None,
        load_image: bool | None = None,
    ) -> Iterator[SpiresFrame]:
        end = len(self)
        if max_frames is not None:
            end = min(end, start + int(max_frames))
        for i in range(max(0, start), end):
            yield self.read(i, load_image=load_image)
