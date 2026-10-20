#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Download Isaac People Characters (+ optional Animations) from NVIDIA Omniverse S3
# into autosim/usd/people/ for habitat.isaac.traffic.character_root.
#
#   python3 scripts/download_people_assets.py
#   python3 scripts/download_people_assets.py --no-animations
#   python3 scripts/download_people_assets.py --jobs 8

from __future__ import annotations

import argparse
import concurrent.futures
import sys
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DEST = ROOT / "usd" / "people"
S3_HOST = "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
S3_CHAR_PREFIX = "Assets/Isaac/5.0/Isaac/People/Characters/"
S3_ANIM_PREFIX = "Assets/Isaac/5.0/Isaac/People/Animations/"

# Compact set used by Hunav_isaac_wrapper + a few extras (skip giant original_* packs).
DEFAULT_CHARACTERS = (
    "Biped_Setup.usd",
    # Required payload of Biped_Setup (skeleton for retarget → walk gait).
    "biped_demo",
    "F_Business_02",
    "F_Medical_01",
    "M_Medical_01",
    "female_adult_police_01_new",
    "female_adult_police_02",
    "female_adult_police_03_new",
    "male_adult_construction_01_new",
    "male_adult_construction_05_new",
    "male_adult_police_04",
)

# Clips we drive + extras referenced by Biped_Setup (avoids USD payload warnings).
DEFAULT_ANIMATIONS = (
    "stand_idle_loop.skelanim.usd",
    "stand_walk_loop_in_place.skelanim.usd",
    "stand_walk_1.skelanim.usd",
    "stand_idle_wave_loop.skelanim.usd",
    "stand_walk_2.skelanim.usd",
    "stand_walk_3.skelanim.usd",
    "stand_walk_4.skelanim.usd",
    "stand_walk_5.skelanim.usd",
    "stand_walk_7.skelanim.usd",
    "LookAround.skelanim.usd",
    "Sit.skelanim.usd",
)


def list_keys(prefix: str) -> list[str]:
    """List object keys under an S3 prefix (paginated)."""
    keys: list[str] = []
    token = None
    ns = {"s3": "http://s3.amazonaws.com/doc/2006-03-01/"}
    while True:
        params: dict[str, str] = {
            "list-type": "2",
            "prefix": prefix,
            "max-keys": "1000",
        }
        if token:
            params["continuation-token"] = token
        url = f"{S3_HOST}/?{urllib.parse.urlencode(params)}"
        with urllib.request.urlopen(url, timeout=60) as resp:
            data = resp.read()
        root = ET.fromstring(data)
        for node in root.findall("s3:Contents", ns):
            key = node.findtext("s3:Key", default="", namespaces=ns)
            if key and not key.endswith("/"):
                keys.append(key)
        truncated = root.findtext("s3:IsTruncated", default="false", namespaces=ns)
        if truncated.lower() != "true":
            break
        token = root.findtext("s3:NextContinuationToken", default="", namespaces=ns)
        if not token:
            break
    return keys


def keep_character(key: str, allow: set[str]) -> bool:
    relative = key[len(S3_CHAR_PREFIX) :] if key.startswith(S3_CHAR_PREFIX) else key
    if not relative or "/.thumbs/" in f"/{relative}" or relative.startswith(".thumbs/"):
        return False
    top = relative.split("/", 1)[0]
    return top in allow


def keep_animation(key: str, allow: set[str]) -> bool:
    relative = key[len(S3_ANIM_PREFIX) :] if key.startswith(S3_ANIM_PREFIX) else key
    if not relative or relative.endswith("/"):
        return False
    name = Path(relative).name
    return name in allow


def _expected_size(key: str) -> int | None:
    """HEAD Content-Length when available."""
    url = f"{S3_HOST}/{urllib.parse.quote(key, safe='/')}"
    try:
        req = urllib.request.Request(url, method="HEAD")
        with urllib.request.urlopen(req, timeout=30) as resp:
            length = resp.headers.get("Content-Length")
            return int(length) if length else None
    except Exception:
        return None


def download_one(key: str, dest_root: Path, prefix: str) -> tuple[str, bool, str]:
    relative = key[len(prefix) :] if key.startswith(prefix) else key
    target = dest_root / relative
    expected = _expected_size(key)
    if (
        target.is_file()
        and target.stat().st_size > 0
        and (expected is None or target.stat().st_size == expected)
    ):
        return relative, True, "skip"
    target.parent.mkdir(parents=True, exist_ok=True)
    url = f"{S3_HOST}/{urllib.parse.quote(key, safe='/')}"
    # Prefer curl resume — large PNG textures often stall urllib.
    import shutil
    import subprocess

    if shutil.which("curl"):
        cmd = [
            "curl",
            "-fsSL",
            "--retry",
            "5",
            "--retry-delay",
            "2",
            "-C",
            "-",
            "-o",
            str(target),
            url,
        ]
        try:
            subprocess.run(cmd, check=True, timeout=600)
            if expected is not None and target.stat().st_size != expected:
                target.unlink(missing_ok=True)
                subprocess.run(
                    [
                        "curl",
                        "-fsSL",
                        "--retry",
                        "5",
                        "-o",
                        str(target),
                        url,
                    ],
                    check=True,
                    timeout=600,
                )
            return relative, True, "ok"
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError) as exc:
            return relative, False, str(exc)
    try:
        urllib.request.urlretrieve(url, target)
        return relative, True, "ok"
    except (urllib.error.URLError, OSError) as exc:
        return relative, False, str(exc)


def download_prefix(
    prefix: str,
    dest: Path,
    *,
    keep,
    jobs: int,
) -> tuple[int, int]:
    keys = [k for k in list_keys(prefix) if keep(k)]
    print(f"download {len(keys)} files → {dest}")
    ok = fail = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, jobs)) as pool:
        futures = [pool.submit(download_one, key, dest, prefix) for key in keys]
        for fut in concurrent.futures.as_completed(futures):
            relative, success, status = fut.result()
            if success:
                ok += 1
                if status != "skip":
                    print(f"  + {relative}")
            else:
                fail += 1
                print(f"  ! {relative}: {status}", file=sys.stderr)
    return ok, fail


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dest", type=Path, default=DEFAULT_DEST)
    parser.add_argument("--jobs", type=int, default=6)
    parser.add_argument("--no-animations", action="store_true")
    parser.add_argument(
        "--all-characters",
        action="store_true",
        help="Download every Characters/* pack (large).",
    )
    args = parser.parse_args()

    char_dest = args.dest / "Characters"
    anim_dest = args.dest / "Animations"
    char_dest.mkdir(parents=True, exist_ok=True)

    if args.all_characters:
        allow_char = None

        def keep_char(key: str) -> bool:
            relative = key[len(S3_CHAR_PREFIX) :]
            return bool(relative) and "/.thumbs/" not in f"/{relative}"

    else:
        allow = set(DEFAULT_CHARACTERS)

        def keep_char(key: str) -> bool:
            return keep_character(key, allow)

    ok, fail = download_prefix(
        S3_CHAR_PREFIX, char_dest, keep=keep_char, jobs=args.jobs
    )
    print(f"Characters: ok={ok} fail={fail}")

    if not args.no_animations:
        anim_dest.mkdir(parents=True, exist_ok=True)
        allow_anim = set(DEFAULT_ANIMATIONS)
        # Also accept stand_walk_loop_in_place if listed under a different name.
        keys = list_keys(S3_ANIM_PREFIX)
        for key in keys:
            name = Path(key).name
            if "walk" in name and "loop" in name and "mirror" not in name:
                allow_anim.add(name)
            if name in ("stand_idle_loop.skelanim.usd",):
                allow_anim.add(name)

        def keep_anim(key: str) -> bool:
            return keep_animation(key, allow_anim)

        a_ok, a_fail = download_prefix(
            S3_ANIM_PREFIX, anim_dest, keep=keep_anim, jobs=args.jobs
        )
        print(f"Animations: ok={a_ok} fail={a_fail}")
        fail += a_fail

    # Smoke: how many character USDs discovery would see.
    packs = [
        p
        for p in sorted(char_dest.iterdir())
        if p.is_dir() and not p.name.startswith(".")
    ]
    print(f"character packs: {len(packs)} under {char_dest}")
    for pack in packs[:8]:
        usds = list(pack.glob("*.usd")) + list(pack.glob("*.usda"))
        print(f"  {pack.name}: {len(usds)} usd")
    return 1 if fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
