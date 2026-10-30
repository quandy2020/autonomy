#!/usr/bin/env python3
"""Safely extract downloaded TUM RGB-D archives without overwriting data."""

from __future__ import annotations

import argparse
import dataclasses
import json
import shutil
import tarfile
import tempfile
from pathlib import Path, PurePosixPath
from typing import Sequence


@dataclasses.dataclass(frozen=True)
class ExtractionResult:
    sequence: str
    archive: str
    destination: str
    status: str
    message: str = ""


def validated_members(
    archive: tarfile.TarFile, expected_root: str
) -> list[tarfile.TarInfo]:
    """Allow regular files/directories below exactly one expected root."""
    members = archive.getmembers()
    if not members:
        raise ValueError("archive is empty")
    for member in members:
        path = PurePosixPath(member.name)
        parts = tuple(part for part in path.parts if part not in ("", "."))
        if path.is_absolute() or ".." in parts:
            raise ValueError(f"unsafe archive path: {member.name!r}")
        if not parts or parts[0] != expected_root:
            raise ValueError(
                f"archive member is outside {expected_root!r}: {member.name!r}"
            )
        if not (member.isdir() or member.isfile()):
            raise ValueError(
                f"unsupported archive member type: {member.name!r}"
            )
    return members


def extract_archive(
    archive_path: Path, root: Path, dry_run: bool = False
) -> ExtractionResult:
    sequence = archive_path.stem
    destination = root / sequence
    common = {
        "sequence": sequence,
        "archive": str(archive_path),
        "destination": str(destination),
    }
    if destination.exists():
        return ExtractionResult(
            **common,
            status="already_present",
            message="destination exists and was not modified",
        )
    if dry_run:
        return ExtractionResult(**common, status="would_extract")

    destination_created = False
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{sequence}.extract-", dir=str(root))
    )
    try:
        with tarfile.open(archive_path, "r:gz") as archive:
            members = validated_members(archive, sequence)
            archive.extractall(temporary, members=members)
        extracted = temporary / sequence
        if not extracted.is_dir():
            raise ValueError(f"archive did not create {sequence!r}")
        try:
            destination.mkdir()
            destination_created = True
        except FileExistsError:
            return ExtractionResult(
                **common,
                status="already_present",
                message="destination appeared concurrently and was not modified",
            )
        for child in extracted.iterdir():
            child.replace(destination / child.name)
    except (OSError, tarfile.TarError, ValueError) as error:
        if destination_created:
            shutil.rmtree(destination, ignore_errors=True)
        return ExtractionResult(**common, status="failed", message=str(error))
    finally:
        shutil.rmtree(temporary, ignore_errors=True)
    return ExtractionResult(**common, status="extracted")


def prepare(
    root: Path, sequences: Sequence[str] = (), dry_run: bool = False
) -> list[ExtractionResult]:
    root = root.resolve()
    if not root.is_dir():
        raise ValueError(f"TUM root is not a directory: {root}")
    archives = sorted(root.glob("rgbd_dataset_*.tgz"))
    if sequences:
        requested = set(sequences)
        known = {archive.stem for archive in archives}
        unknown = sorted(requested - known)
        if unknown:
            raise ValueError("archives not found: " + ", ".join(unknown))
        archives = [archive for archive in archives if archive.stem in requested]
    if not archives:
        raise ValueError(f"no TUM RGB-D archives found in {root}")
    return [extract_archive(archive, root, dry_run) for archive in archives]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, help="directory containing TUM .tgz files")
    parser.add_argument(
        "--sequence",
        action="append",
        default=[],
        help="extract only this archive stem; repeat for multiple sequences",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="report actions without extracting"
    )
    args = parser.parse_args()
    try:
        results = prepare(args.root, args.sequence, args.dry_run)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(
        json.dumps(
            [dataclasses.asdict(result) for result in results],
            indent=2,
            sort_keys=True,
        )
    )
    return 1 if any(result.status == "failed" for result in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
