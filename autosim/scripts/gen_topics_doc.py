#!/usr/bin/env python3
"""Regenerate the per-config channel table in ``docs/06_topics.md``.

The table is derived from ``Config.channel_map()`` so a channel rename in a
YAML shows up in the docs. ``--check`` fails when the checked-in table is
stale (used by the test suite).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from autosim.runtime.settings import Config  # noqa: E402

DOC = ROOT / "docs" / "06_topics.md"
BEGIN = "<!-- BEGIN generated: config-channels -->"
END = "<!-- END generated: config-channels -->"
BASELINE = "default.yaml"


def config_paths() -> list[Path]:
    """Shipped configs that can be launched directly."""
    paths = sorted((ROOT / "config").glob("*.yaml"))
    paths += sorted((ROOT / "config" / "robots").glob("*.yaml"))
    return [path for path in paths if not path.name.startswith("_")]


def channel_map(path: Path) -> dict[str, str]:
    """Channels without backend validation (Isaac USD may be absent)."""
    data = Config._load_yaml(path)
    Config.fill_omitted_sensors(data)
    return Config(data).channel_map()


def _diff(path: Path, baseline: dict[str, str]) -> str:
    channels = channel_map(path)
    diffs: list[str] = []
    for key, channel in channels.items():
        if key not in baseline:
            diffs.append(f"`{key}={channel}`（新增）")
        elif baseline[key] != channel:
            diffs.append(f"`{key}={channel}`")
    if path.parent.name != "robots":
        for key in baseline:
            if key not in channels:
                diffs.append(f"`{key}`（缺）")
    return "、".join(diffs) if diffs else "同 `default.yaml`"


def render() -> str:
    baseline = channel_map(ROOT / "config" / BASELINE)
    top: list[tuple[str, str]] = []
    robots: list[tuple[str, str]] = []
    for path in config_paths():
        try:
            diff = "基线" if path.name == BASELINE else _diff(path, baseline)
        except Exception as exc:  # noqa: BLE001 - show the failure in the table
            diff = f"加载失败：`{exc}`"
        (robots if path.parent.name == "robots" else top).append((path.name, diff))
    lines = [
        BEGIN,
        "",
        "以 `config/default.yaml` 为基线，只列新增、改名或缺失的逻辑键；完整默认表见第 1 节。",
        "",
        "### 顶层配置",
        "",
        "| 配置 | 与 `default.yaml` 的差异 |",
        "|------|--------------------------|",
    ]
    lines += [f"| `{name}` | {diff} |" for name, diff in top]
    lines += [
        "",
        "### 机器人档案",
        "",
        "机器人档案只列新增或改名的键；它们不是完整的导航栈，缺省键由 `_base.yaml` 或运行时补齐。",
        "",
        "| 档案 | 与 `default.yaml` 的差异 |",
        "|------|--------------------------|",
    ]
    lines += [f"| `{name}` | {diff} |" for name, diff in robots]
    lines += ["", END]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail when the doc is stale")
    args = parser.parse_args()
    text = DOC.read_text()
    if BEGIN not in text or END not in text:
        raise SystemExit(f"missing generated block markers in {DOC}")
    start = text.index(BEGIN)
    end = text.index(END) + len(END)
    updated = text[:start] + render() + text[end:]
    if args.check:
        if updated != text:
            raise SystemExit(
                "docs/06_topics.md is stale; run scripts/gen_topics_doc.py"
            )
        return
    DOC.write_text(updated)
    print(f"updated {DOC.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
