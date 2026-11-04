#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Download TrackVLA-layout SMPL-X humanoids into autosim/data/humanoids/.

Does **not** require a TrackVLA checkout. Assets land at::

  data/humanoids/humanoid_data/<name>/<name>.urdf
  data/humanoids/humanoid_data/<name>/<name>_motion_data_smplx.pkl

Usage::

  python3 scripts/download_humanoids.py
  python3 scripts/download_humanoids.py --force
"""

from __future__ import annotations

import argparse
import shutil
import sys
import tempfile
import zipfile
from pathlib import Path
from urllib.request import Request, urlopen

# Same pack as TrackVLA README (Google Drive fallback when 0x0.st expires).
DRIVE_FILE_ID = "1aE_wyvPqvOuVmF8px2vTO3trr70DKf1l"
DRIVE_URL = (
    f"https://drive.usercontent.google.com/download"
    f"?id={DRIVE_FILE_ID}&export=download&confirm=t"
)


def project_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "config" / "default.yaml").is_file():
            return parent
    return here.parents[1]


def dest_root() -> Path:
    return project_root() / "data" / "humanoids"


def download(url: str, path: Path) -> None:
    print(f"Downloading {url}")
    req = Request(url, headers={"User-Agent": "autosim-download_humanoids/1.0"})
    with urlopen(req, timeout=600) as response, path.open("wb") as handle:
        total = int(response.headers.get("Content-Length") or 0)
        done = 0
        while True:
            chunk = response.read(1024 * 1024)
            if not chunk:
                break
            handle.write(chunk)
            done += len(chunk)
            if total:
                pct = 100.0 * done / total
                print(f"\r  {done / 1e6:.1f}/{total / 1e6:.1f} MB ({pct:.0f}%)", end="")
            else:
                print(f"\r  {done / 1e6:.1f} MB", end="")
        print()
    # Reject HTML virus-scan pages.
    head = path.read_bytes()[:200].lower()
    if b"<!doctype html" in head or b"<html" in head:
        path.unlink(missing_ok=True)
        raise RuntimeError(
            "Download returned HTML (Google Drive virus-scan page). "
            "Open the Drive link in a browser, save humanoids.zip, then:\n"
            f"  python3 scripts/download_humanoids.py --zip /path/to/humanoids.zip\n"
            f"Drive: https://drive.google.com/file/d/{DRIVE_FILE_ID}/view"
        )


def look_like_humanoid_data(path: Path) -> bool:
    if not path.is_dir():
        return False
    for child in path.iterdir():
        if not child.is_dir():
            continue
        if (child / f"{child.name}.urdf").is_file():
            return True
    return False


def find_humanoid_data(extracted: Path) -> Path | None:
    """Locate ``humanoid_data`` (or equivalent) inside an extracted zip."""
    candidates = [
        extracted / "humanoid_data",
        extracted / "humanoids" / "humanoid_data",
        extracted / "data" / "humanoids" / "humanoid_data",
        extracted / "humanoids",
    ]
    for path in candidates:
        if look_like_humanoid_data(path):
            return path
    # Nested single top-level folder from zip.
    kids = [p for p in extracted.iterdir() if p.is_dir() and not p.name.startswith("__")]
    if len(kids) == 1:
        return find_humanoid_data(kids[0])
    if look_like_humanoid_data(extracted):
        return extracted
    return None


def install_from_zip(zip_path: Path, force: bool) -> Path:
    target = dest_root() / "humanoid_data"
    if target.is_dir() and any(target.iterdir()) and not force:
        print(f"Already present: {target} (pass --force to replace)")
        return target
    with tempfile.TemporaryDirectory(prefix="autosim_humanoids_") as tmp:
        tmp_path = Path(tmp)
        print(f"Extracting {zip_path} …")
        with zipfile.ZipFile(zip_path, "r") as zf:
            zf.extractall(tmp_path)
        src = find_humanoid_data(tmp_path)
        if src is None:
            raise RuntimeError(
                f"Could not find humanoid_data layout inside {zip_path}"
            )
        dest_root().mkdir(parents=True, exist_ok=True)
        if target.exists():
            shutil.rmtree(target)
        shutil.copytree(src, target)
    n = sum(1 for p in target.iterdir() if p.is_dir())
    print(f"Installed {n} avatars → {target}")
    return target


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--zip",
        type=Path,
        help="Use a local humanoids.zip instead of downloading",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Replace existing data/humanoids/humanoid_data",
    )
    args = parser.parse_args()
    dest_root().mkdir(parents=True, exist_ok=True)
    try:
        if args.zip:
            zip_path = args.zip.expanduser().resolve()
            if not zip_path.is_file():
                print(f"zip not found: {zip_path}", file=sys.stderr)
                return 1
            install_from_zip(zip_path, force=args.force)
        else:
            zip_path = dest_root() / "humanoids.zip"
            download(DRIVE_URL, zip_path)
            install_from_zip(zip_path, force=args.force)
            zip_path.unlink(missing_ok=True)
    except Exception as exc:  # noqa: BLE001
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
