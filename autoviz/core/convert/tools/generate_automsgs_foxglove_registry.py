#!/usr/bin/env python3
# Copyright 2025 The Openbot Authors
"""Scan automsgs/proto and emit registry/automsgs_foxglove_registry_data.hpp.

设计：字符串只出现一次（池化），每类型一行 Row；编译期 static_assert 有序；运行时由
automsgs_foxglove_registry.cpp 构建 vector 并二分查找。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


def strip_comment(line: str) -> str:
    if "//" in line:
        line = line[: line.index("//")]
    return line.strip()


def top_level_messages(text: str) -> list[str]:
    depth = 0
    names: list[str] = []
    for raw in text.splitlines():
        line = strip_comment(raw)
        if not line:
            continue
        open_b = line.count("{")
        close_b = line.count("}")
        if depth == 0:
            m = re.match(r"^message\s+(\w+)\s*\{", line)
            if m:
                names.append(m.group(1))
        depth += open_b - close_b
        if depth < 0:
            depth = 0
    return names


def parse_package(text: str) -> str | None:
    for raw in text.splitlines():
        line = strip_comment(raw)
        m = re.match(r"^package\s+([\w.]+)\s*;", line)
        if m:
            return m.group(1)
    return None


def cpp_escape(s: str) -> str:
    return (
        '"'
        + s.replace("\\", "\\\\")
        .replace('"', '\\"')
        .replace("\n", "\\n")
        + '"'
    )


SPECIAL: dict[tuple[str, str], tuple[str, str, str]] = {
    ("nav_msgs", "Path"): (
        "foxglove.SceneUpdate",
        "kConvertToSceneUpdate",
        "3D：SceneUpdate + LinePrimitive",
    ),
    ("sensor_msgs", "LaserScan"): (
        "foxglove.LaserScan",
        "kConvertToLaserScan",
        "LaserScan 面板；未实现字段映射时用 Log 回退",
    ),
    ("sensor_msgs", "PointCloud2"): (
        "foxglove.PointCloud",
        "kConvertToPointCloud",
        "PointCloud；未实现时用 Log 回退",
    ),
    ("sensor_msgs", "PointCloud"): (
        "foxglove.PointCloud",
        "kConvertToPointCloud",
        "旧版 PointCloud；未实现时用 Log 回退",
    ),
    ("sensor_msgs", "CompressedImage"): (
        "foxglove.CompressedImage",
        "kConvertToImage",
        "Image 面板；未实现时用 Log 回退",
    ),
    ("sensor_msgs", "Image"): (
        "foxglove.RawImage",
        "kConvertToImage",
        "RawImage；未实现时用 Log 回退",
    ),
    ("nav_msgs", "OccupancyGrid"): (
        "foxglove.Grid",
        "kConvertToGrid",
        "Grid/地图；未实现时用 Log 回退",
    ),
    ("sensor_msgs", "NavSatFix"): (
        "foxglove.LocationFix",
        "kConvertToLocationFix",
        "地图 LocationFix；未实现时用 Log 回退",
    ),
    ("sensor_msgs", "JointState"): (
        "foxglove.JointState",
        "kConvertToLog",
        "目标 JointState；字段对齐后改专用转换，暂 TextFormat/Log",
    ),
    ("geometry_msgs", "PoseStamped"): (
        "foxglove.PoseInFrame",
        "kConvertToPoseInFrame",
        "3D：PoseInFrame",
    ),
    ("geometry_msgs", "TransformStamped"): (
        "foxglove.FrameTransform",
        "kConvertToFrameTransform",
        "3D：FrameTransform（TF）",
    ),
    ("geometry_msgs", "PoseArray"): (
        "foxglove.PosesInFrame",
        "kConvertToPosesInFrame",
        "3D：PosesInFrame",
    ),
    ("nav_msgs", "Odometry"): (
        "foxglove.PoseInFrame",
        "kConvertToPoseInFrame",
        "3D：位姿为 PoseInFrame（header.frame_id）",
    ),
    ("visualization_msgs", "Marker"): (
        "foxglove.SceneUpdate",
        "kConvertToSceneUpdate",
        "3D：SceneUpdate / Marker",
    ),
    ("visualization_msgs", "MarkerArray"): (
        "foxglove.SceneUpdate",
        "kConvertToSceneUpdate",
        "3D：SceneUpdate / MarkerArray",
    ),
    ("sensor_msgs", "MultiEchoLaserScan"): (
        "foxglove.PointCloud",
        "kConvertToPointCloud",
        "3D：多回波展开为 PointCloud",
    ),
    ("nav_msgs", "GridCells"): (
        "foxglove.SceneUpdate",
        "kConvertToSceneUpdate",
        "3D：GridCells → 立方体单元",
    ),
    ("vision_msgs", "BoundingBox3D"): (
        "foxglove.SceneUpdate",
        "kConvertToSceneUpdate",
        "3D：单框 CubePrimitive",
    ),
    ("vision_msgs", "BoundingBox3DArray"): (
        "foxglove.SceneUpdate",
        "kConvertToSceneUpdate",
        "3D：多框 SceneUpdate",
    ),
    ("vision_msgs", "Detection3D"): (
        "foxglove.SceneUpdate",
        "kConvertToSceneUpdate",
        "3D：检测框 + 标签",
    ),
    ("vision_msgs", "Detection3DArray"): (
        "foxglove.SceneUpdate",
        "kConvertToSceneUpdate",
        "3D：Detection3D 数组",
    ),
    ("std_msgs", "String"): (
        "foxglove.Log",
        "kConvertToLog",
        "Log：正文为 String.data",
    ),
}


def classify_msgs(full_name: str, pkg: str, msg: str) -> tuple[str, str, str]:
    key = (pkg, msg)
    if key in SPECIAL:
        return SPECIAL[key]
    return (
        "foxglove.Log",
        "kConvertToLog",
        "Log：protobuf TextFormat（通用）",
    )


def classify_full_name(full_name: str) -> tuple[str, str, str]:
    parts = full_name.split(".")
    if len(parts) >= 4 and parts[0] == "automsgs" and parts[1] == "msgs":
        return classify_msgs(full_name, parts[2], parts[3])
    if len(parts) == 3 and parts[0] == "automsgs" and parts[1] == "srvs":
        return (
            "foxglove.Log",
            "kConvertToLog",
            "服务封装消息：Log / Raw",
        )
    if len(parts) >= 4 and parts[0] == "automsgs" and parts[1] == "rpcs":
        return (
            "foxglove.Log",
            "kConvertToLog",
            "RPC 消息：Log / Raw",
        )
    return ("foxglove.Log", "kConvertToLog", "Log：通用")


def intern(strings: list[str], table: dict[str, int], s: str) -> int:
    if s not in table:
        table[s] = len(strings)
        strings.append(s)
    return table[s]


def emit_data_hpp(
    entries: list[tuple[str, str, str, str]],
    out_path: Path,
) -> None:
    """entries: (full_name, fg_canonical, strategy_enum, hint) sorted by full_name."""
    pool: list[str] = []
    sid: dict[str, int] = {}
    rows: list[tuple[int, int, str, int]] = []
    for full, fg, strat, hint in entries:
        rows.append(
            (
                intern(pool, sid, full),
                intern(pool, sid, fg),
                strat,
                intern(pool, sid, hint),
            )
        )

    n = len(rows)
    lines: list[str] = [
        "// GENERATED BY tools/generate_automsgs_foxglove_registry.py — DO NOT EDIT.",
        "",
        "#pragma once",
        "",
        "#include <array>",
        "#include <cstddef>",
        "#include <string_view>",
        "",
        '#include "autonomy/autoviz/core/convert/registry/automsgs_foxglove_registry.hpp"',
        "",
        "namespace autoviz {",
        "namespace converter {",
        "namespace foxglove_registry_data {",
        "",
        "namespace str {",
    ]

    for i, text in enumerate(pool):
        lines.append(f"inline constexpr char s{i}[] = {cpp_escape(text)};")

    lines.extend(
        [
            "}  // namespace str",
            "",
            "struct Row {",
            "  std::string_view proto_full_name;",
            "  std::string_view foxglove_canonical_name;",
            "  AutomsgsFoxgloveStrategy strategy;",
            "  std::string_view studio_hint;",
            "};",
            "",
            f"inline constexpr std::array<Row, {n}> kRows{{{{",
        ]
    )

    for pi, ci, strat, hi in rows:
        lines.append(
            "    Row{str::"
            + f"s{pi}, str::s{ci}, AutomsgsFoxgloveStrategy::{strat}, str::s{hi}"
            + "},"
        )

    lines.extend(
        [
            "}};",
            "",
            "template <std::size_t N>",
            "constexpr bool RowsAreSorted(const std::array<Row, N>& a) {",
            "  for (std::size_t i = 1; i < N; ++i) {",
            "    if (a[i - 1].proto_full_name >= a[i].proto_full_name) {",
            "      return false;",
            "    }",
            "  }",
            "  return true;",
            "}",
            "",
            "static_assert(RowsAreSorted(kRows),",
            "              \"foxglove registry rows must stay sorted by proto_full_name\");",
            "",
            "}  // namespace foxglove_registry_data",
            "}  // namespace converter",
            "}  // namespace autoviz",
            "",
        ]
    )

    out_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    # tools/ → convert → core → autoviz → **src/autonomy**（与 automsgs 同级的 autonomy 包根）
    autonomy_pkg_root = Path(__file__).resolve().parents[4]
    proto_root = autonomy_pkg_root / "automsgs" / "proto"
    if not proto_root.is_dir():
        print(
            "proto root not found:",
            proto_root,
            "(expect src/autonomy/automsgs/proto)",
            file=sys.stderr,
        )
        return 1

    out_path = (
        Path(__file__).resolve().parent.parent
        / "registry"
        / "automsgs_foxglove_registry_data.hpp"
    )

    raw_entries: list[tuple[str, str, str, str]] = []

    for proto in sorted(proto_root.rglob("*.proto")):
        text = proto.read_text(encoding="utf-8")
        pkg = parse_package(text)
        if not pkg:
            continue
        for msg in top_level_messages(text):
            full = f"{pkg}.{msg}"
            fg, strat, hint = classify_full_name(full)
            raw_entries.append((full, fg, strat, hint))

    raw_entries.sort(key=lambda x: x[0])

    emit_data_hpp(raw_entries, out_path)
    print("Wrote", out_path, "rows", len(raw_entries), "unique strings", "see pool in file")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
