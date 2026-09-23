#!/usr/bin/env python3
"""Validate Atlas platform YAML covers ORB-SLAM3 Settings field groups."""

from __future__ import annotations

import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    print("SKIP: PyYAML not installed")
    sys.exit(0)

ROOT = Path(__file__).resolve().parents[2] / "config" / "platforms"

REQUIRED_CAM = {"fx", "fy", "cx", "cy"}
REQUIRED_ORB = {"n_features", "scale_factor", "n_levels", "ini_th_fast", "min_th_fast"}
REQUIRED_IMU = {"noise_gyro", "noise_acc", "walk_gyro", "walk_acc", "T_b_c"}


def check(path: Path, need_imu: bool) -> list[str]:
    errs: list[str] = []
    data = yaml.safe_load(path.read_text())
    cam = data.get("camera") or {}
    missing = REQUIRED_CAM - set(cam)
    if missing:
        errs.append(f"{path.name}: camera missing {sorted(missing)}")
    orb = data.get("orb") or {}
    missing = REQUIRED_ORB - set(orb)
    if missing:
        errs.append(f"{path.name}: orb missing {sorted(missing)}")
    if need_imu:
        imu = data.get("imu") or {}
        missing = REQUIRED_IMU - set(imu)
        if missing:
            errs.append(f"{path.name}: imu missing {sorted(missing)}")
        tbc = imu.get("T_b_c") if imu else None
        if tbc is not None and len(tbc) != 16:
            errs.append(f"{path.name}: imu.T_b_c len={len(tbc)} != 16")
    return errs


def main() -> None:
    files = [
        ("euroc_mono.yaml", False),
        ("euroc_mono_inertial.yaml", True),
        ("euroc_stereo.yaml", False),
        ("euroc_stereo_inertial.yaml", True),
    ]
    errs: list[str] = []
    for name, need_imu in files:
        path = ROOT / name
        if not path.is_file():
            errs.append(f"missing {path}")
            continue
        errs.extend(check(path, need_imu))
    if errs:
        print("FAIL")
        for e in errs:
            print(" ", e)
        sys.exit(1)
    print("PASS: platform YAML Settings field coverage")
    for name, _ in files:
        print(f"  ok {name}")


if __name__ == "__main__":
    main()
