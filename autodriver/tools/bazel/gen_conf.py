#!/usr/bin/env python3
"""Generate autodriver/conf/conf.hpp from conf.hpp.in + version.json."""

from __future__ import annotations

import json
import pathlib
import sys
from datetime import datetime


def main() -> int:
    if len(sys.argv) != 4:
        print(
            f"usage: {sys.argv[0]} <conf.hpp.in> <version.json> <output.hpp>",
            file=sys.stderr,
        )
        return 2

    template_path = pathlib.Path(sys.argv[1])
    version_path = pathlib.Path(sys.argv[2])
    output_path = pathlib.Path(sys.argv[3])

    data = json.loads(version_path.read_text(encoding="utf-8"))
    now = datetime.now()
    config_dir = str(version_path.resolve().parent / "config")

    replacements = {
        "@AUTODRIVER_PACKAGE_NAME@": str(data.get("name", "Autodriver")),
        "@AUTODRIVER_VERSION@": str(data.get("version", "0.0.0")),
        "@AUTODRIVER_FULL_VERSION@": str(
            data.get("full_version", data.get("version", "0.0.0"))
        ),
        "@AUTODRIVER_PACKAGE_DESCRIPTION@": str(
            data.get("description", "Sensor + chassis HAL for vehicle robots")
        ),
        "@AUTODRIVER_GIT_DESCRIBE@": str(data.get("git_describe", "unknown")),
        "@AUTODRIVER_GIT_COMMIT@": str(data.get("git_commit", "unknown")),
        "@AUTODRIVER_GIT_BRANCH@": str(data.get("git_branch", "unknown")),
        "@AUTODRIVER_GIT_COMMIT_DATE@": str(
            data.get("git_commit_date", "unknown")
        ),
        "@AUTODRIVER_GIT_AUTHOR@": str(data.get("git_author", "unknown")),
        "@AUTODRIVER_GIT_EMAIL@": str(data.get("git_email", "unknown")),
        "@AUTODRIVER_GIT_DIRTY@": (
            "true" if data.get("git_dirty") else "false"
        ),
        "@AUTODRIVER_BUILD_DATE@": now.strftime("%Y-%m-%d"),
        "@AUTODRIVER_BUILD_TIME@": now.strftime("%Y-%m-%d %H:%M:%S.000"),
        "@AUTODRIVER_DEFAULT_CONFIG_DIR@": config_dir,
        "@CMAKE_INSTALL_PREFIX@": "/usr/local",
    }

    text = template_path.read_text(encoding="utf-8")
    for key, value in replacements.items():
        text = text.replace(key, value)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
