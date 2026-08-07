#!/usr/bin/env python3
"""Sync Autoviz icons directly from a local ROS 2 RViz checkout (no frames/wrappers)."""

from __future__ import annotations

import os
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "resources"
ICONS = ROOT / "icons"
QRC = ROOT / "autoviz.qrc"

RVIZ_ROOT = Path(
    os.environ.get(
        "RVIZ_SOURCE",
        "/home/quandy/workspace/github/ros2/rviz",
    )
)

RVIZ_ICON_ROOTS = [
    RVIZ_ROOT / "rviz_default_plugins" / "icons",
    RVIZ_ROOT / "rviz_common" / "icons",
]

# Autoviz-only resource path -> RViz source filename (under icons/ or icons/classes/)
FALLBACK_BY_DEST: dict[str, str] = {
    # panels (Foxglove) -> closest RViz display
    "icons/panels/panel_3d.png": "classes/RobotModel.png",
    "icons/panels/panel_image.png": "classes/Image.png",
    "icons/panels/panel_map.png": "classes/Map.png",
    "icons/panels/panel_plot.png": "classes/Path.png",
    "icons/panels/panel_raw_messages.png": "classes/FlatColor.svg",
    "icons/panels/panel_transform_tree.png": "classes/TF.png",
    "icons/panels/panel_data_source.png": "classes/Time.png",
    "icons/panels/panel_parameters.png": "options.png",
    "icons/panels/panel_publish.png": "classes/PublishPoint.svg",
    "icons/panels/panel_table.png": "classes/GridCells.png",
    "icons/panels/panel_teleop.png": "classes/Interact.png",
    "icons/panels/panel_gauge.png": "classes/Effort.png",
    "icons/panels/panel_indicator.png": "classes/Illuminance.png",
    "icons/panels/panel_state.png": "classes/PoseArray.png",
    "icons/panels/panel_service.png": "classes/Wrench.png",
    "icons/panels/panel_log.png": "classes/Help.svg",
    "icons/panels/panel_channel_graph.png": "classes/TF.png",
    "icons/panels/panel_markdown.png": "classes/Help.svg",
    "icons/panels/panel_stack.png": "classes/Group.png",
    "icons/panels/panel_tab.png": "classes/Displays.svg",
    "icons/panels/panel_audio.png": "default_class_icon.png",
    # plot toolbar — RViz2 semantics (not Foxglove plot chrome)
    "icons/plot/plot_reset_view.svg": "rotate.svg",
    "icons/plot/plot_legend.png": "classes/Displays.svg",
    # chrome / sidebars
    "icons/add_panel.png": "plus.png",
    "icons/sidebar_left.png": "left_dock.svg",
    "icons/sidebar_right.png": "right_dock.svg",
    "icons/global_status.png": "ok.png",
    "icons/aviz.png": "default_package_icon.png",
    # menus
    "icons/menu/open_config.png": "package.png",
    "icons/menu/save_config.png": "package.png",
    "icons/menu/save_as.png": "package.png",
    "icons/menu/recent.png": "rotate.svg",
    "icons/menu/file.png": "package.png",
    "icons/menu/config_file.png": "package.png",
    "icons/menu/layout.png": "left_dock.svg",
    "icons/menu/reset_layout.png": "rotate.svg",
    "icons/menu/add_panel.png": "plus.png",
    "icons/menu/camera.png": "classes/Camera.png",
    "icons/menu/backend.png": "default_package_icon.png",
    "icons/menu/orbit.png": "rotate_cam.svg",
    "icons/menu/xy_orbit.png": "rotate.svg",
    "icons/menu/top_down.png": "classes/Grid.png",
    "icons/menu/top_down_ortho.png": "classes/GridCells.png",
    "icons/menu/third_person.png": "classes/RobotModel.png",
    "icons/menu/fps.png": "move2d.svg",
    "icons/menu/opengl.png": "classes/DepthCloud.png",
    "icons/menu/ogre.png": "classes/RobotModel.png",
    "icons/menu/help.png": "classes/Help.svg",
    "icons/menu/about.png": "classes/Help.svg",
    "icons/menu/settings.png": "options.png",
    "icons/menu/view.png": "classes/Views.svg",
    "icons/menu/app.png": "default_package_icon.png",
    "icons/menu/screenshot.png": "classes/Camera.png",
    "icons/menu/quit.png": "close.png",
    # autoviz class aliases
    "icons/classes/CameraInfo.png": "classes/Camera.png",
    "icons/classes/Imu.png": "classes/Effort.png",
    "icons/classes/Wrench.svg": "classes/Wrench.png",
    "icons/classes/Selection.png": "classes/Selection.png",
    # strata -> default
    "icons/classes/StrataBuilding.png": "default_class_icon.png",
    "icons/classes/StrataCanvasLabel.png": "default_class_icon.png",
    "icons/classes/StrataFov.png": "classes/FocusCamera.svg",
    "icons/classes/StrataIotBubble.png": "default_class_icon.png",
    "icons/classes/StrataLabelBubble.png": "default_class_icon.png",
    "icons/classes/StrataPoi.png": "classes/Marker.png",
    "icons/classes/StrataRoadGraph.png": "classes/Path.png",
    "icons/classes/StrataRobot.png": "classes/RobotModel.png",
    "icons/classes/StrataRobot3D.png": "classes/RobotModel.png",
    "icons/classes/StrataSemanticZone.png": "classes/Polygon.png",
}


def build_rviz_index() -> dict[str, Path]:
    """filename (lower) -> absolute path; default_plugins overrides common."""
    index: dict[str, Path] = {}
    for base in RVIZ_ICON_ROOTS:
        if not base.is_dir():
            continue
        for path in base.rglob("*"):
            if not path.is_file():
                continue
            if path.suffix.lower() not in {".png", ".svg"}:
                continue
            if "/classes/src/" in path.as_posix():
                continue
            rel = path.relative_to(base).as_posix()
            index[path.name.lower()] = path
            index[rel.lower()] = path
            index[path.stem.lower()] = path
    return index


def resolve_source(index: dict[str, Path], spec: str) -> Path | None:
    spec = spec.replace("\\", "/")
    key = spec.lower()
    if key in index:
        return index[key]
    base = Path(spec).name.lower()
    return index.get(base)


def copy_icon(src: Path, dest: Path) -> Path:
    """Copy icon; keep source extension so SVG is never stored as .png."""
    dest.parent.mkdir(parents=True, exist_ok=True)
    if src.suffix.lower() != dest.suffix.lower():
        dest = dest.with_suffix(src.suffix)
    shutil.copy2(src, dest)
    return dest


def sync_all_rviz_classes(index: dict[str, Path], copied: set[str]) -> int:
    n = 0
    for rel_key, src in sorted(index.items()):
        if "/" not in rel_key:
            continue
        if not rel_key.startswith("classes/"):
            continue
        if rel_key.count("/") > 1:
            continue
        name = src.name
        dest = ICONS / "classes" / name
        rel_dest = f"icons/classes/{name}"
        if rel_dest in copied:
            continue
        copy_icon(src, dest)
        copied.add(rel_dest)
        n += 1
    return n


def sync_rviz_root_icons(index: dict[str, Path], copied: set[str]) -> int:
    n = 0
    skip = {"classes"}
    for base in RVIZ_ICON_ROOTS:
        if not base.is_dir():
            continue
        for path in base.iterdir():
            if not path.is_file():
                continue
            if path.suffix.lower() not in {".png", ".svg"}:
                continue
            dest = ICONS / path.name
            rel_dest = f"icons/{path.name}"
            if rel_dest in copied:
                continue
            copy_icon(path, dest)
            copied.add(rel_dest)
            n += 1
    return n


def sync_fallbacks(index: dict[str, Path], copied: set[str]) -> int:
    n = 0
    for dest_rel, src_spec in FALLBACK_BY_DEST.items():
        src = resolve_source(index, src_spec)
        if src is None:
            print(f"WARN fallback source missing: {src_spec} -> {dest_rel}")
            continue
        dest = ROOT / dest_rel
        actual = copy_icon(src, dest)
        copied.add(f"icons/{actual.relative_to(ICONS).as_posix()}")
        n += 1
    return n


def sync_images() -> None:
    src_dir = RVIZ_ROOT / "rviz_common" / "images"
    dst_dir = ROOT / "images"
    dst_dir.mkdir(parents=True, exist_ok=True)
    if not src_dir.is_dir():
        return
    for name in ("splash.png", "splash_overlay.png"):
        src = src_dir / name
        if src.is_file():
            shutil.copy2(src, dst_dir / name)


def write_qrc() -> None:
    files: list[str] = []
    for folder, prefix in ((ICONS, "icons"), (ROOT / "images", "images")):
        if not folder.is_dir():
            continue
        for path in sorted(folder.rglob("*")):
            if path.is_file() and path.suffix.lower() in {".png", ".svg"}:
                files.append(f"{prefix}/{path.relative_to(folder).as_posix()}")

    lines = [
        "<!DOCTYPE RCC>",
        '<RCC version="1.0">',
        '  <qresource prefix="/autoviz">',
    ]
    for rel in files:
        lines.append(f"    <file>{rel}</file>")
    lines.extend(["  </qresource>", "</RCC>", ""])
    QRC.write_text("\n".join(lines), encoding="utf-8")


def clean_icons_dir() -> None:
    if ICONS.exists():
        shutil.rmtree(ICONS)
    ICONS.mkdir(parents=True)


def main() -> None:
    if not RVIZ_ROOT.is_dir():
        raise SystemExit(f"RViz source not found: {RVIZ_ROOT}")

    clean_icons_dir()
    index = build_rviz_index()
    if not index:
        raise SystemExit(f"No icons found under {RVIZ_ICON_ROOTS}")

    copied: set[str] = set()
    n_class = sync_all_rviz_classes(index, copied)
    n_root = sync_rviz_root_icons(index, copied)
    n_fb = sync_fallbacks(index, copied)
    sync_images()
    write_qrc()

    print(f"RViz root: {RVIZ_ROOT}")
    print(f"Indexed {len(index)} RViz icon keys")
    print(f"Copied classes={n_class} root={n_root} fallbacks={n_fb} total={len(copied)}")
    print(f"Updated {QRC}")


if __name__ == "__main__":
    main()
