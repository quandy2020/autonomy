#!/usr/bin/env python3
"""Expand autonomy/common/version.cpp.in from version.json (Bazel genrule helper)."""

from __future__ import annotations

import json
import pathlib
import sys


def main() -> int:
    if len(sys.argv) != 4:
        print(
            f"usage: {sys.argv[0]} <version.cpp.in> <version.json> <out.cpp>",
            file=sys.stderr,
        )
        return 2

    src = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
    ver = json.loads(pathlib.Path(sys.argv[2]).read_text(encoding="utf-8"))
    major = ver.get("major", 0)
    minor = ver.get("minor", 0)
    patch = ver.get("patch", 0)
    repl = {
        "AUTONOMY_MAJOR_VERSION": str(major),
        "AUTONOMY_MINOR_VERSION": str(minor),
        "AUTONOMY_PATCH_VERSION": str(patch),
        "AUTONOMY_VERSION": f"{major}.{minor}.{patch}",
        "GIT_COMMIT_ID": "unknown",
        "GIT_VERSION": "unknown",
        "GIT_BRANCH": "unknown",
        "GIT_COMMIT_DATE": "Unknown",
        "GIT_COMMIT_AUTHOR": "unknown",
        "GIT_COMMIT_EMAIL": "unknown",
        "BUILD_TIMESTAMP": "unknown",
        "BUILD_HOST": "unknown",
        "BUILD_USER": "unknown",
        "SYSTEM_NAME": "Darwin",
        "SYSTEM_PROCESSOR": "unknown",
        "SYSTEM_VERSION": "unknown",
        "COMPILER_ID": "Clang",
        "COMPILER_VERSION": "unknown",
    }
    out = src
    for key, value in repl.items():
        out = out.replace("@" + key + "@", value)
    pathlib.Path(sys.argv[3]).write_text(out, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
