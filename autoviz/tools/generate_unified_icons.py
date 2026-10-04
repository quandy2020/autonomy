#!/usr/bin/env python3
"""Sync Autoviz icons from a local ROS 2 RViz checkout, storing SVG only."""

from __future__ import annotations

import base64
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
# Destinations are always .svg (PNG sources are converted on copy).
FALLBACK_BY_DEST: dict[str, str] = {
    "icons/panels/3d.svg": "classes/RobotModel.png",
    "icons/panels/image.svg": "classes/Image.png",
    "icons/panels/map.svg": "classes/Map.png",
    "icons/panels/plot.svg": "classes/Path.png",
    "icons/panels/raw_messages.svg": "classes/FlatColor.svg",
    "icons/panels/transform_tree.svg": "classes/TF.png",
    "icons/panels/data_source.svg": "classes/Time.png",
    "icons/panels/publish.svg": "classes/PublishPoint.svg",
    "icons/panels/teleop.svg": "classes/Interact.png",
    "icons/panels/service.svg": "classes/Wrench.png",
    "icons/panels/channel_graph.svg": "classes/TF.png",
    "icons/panels/stack.svg": "classes/Group.png",
    "icons/panels/tab.svg": "classes/Displays.svg",
    "icons/plot/reset_view.svg": "rotate.svg",
    "icons/plot/legend.svg": "classes/Displays.svg",
    "icons/tool/add_panel.svg": "plus.png",
    "icons/dock/sidebar_left.svg": "left_dock.svg",
    "icons/dock/sidebar_right.svg": "right_dock.svg",
    "icons/status/global.svg": "ok.png",
    "icons/aviz.svg": "default_package_icon.png",
    "icons/menu/open_config.svg": "package.png",
    "icons/menu/save_config.svg": "package.png",
    "icons/menu/save_as.svg": "package.png",
    "icons/menu/recent.svg": "rotate.svg",
    "icons/menu/file.svg": "package.png",
    "icons/menu/config_file.svg": "package.png",
    "icons/menu/layout.svg": "left_dock.svg",
    "icons/menu/reset_layout.svg": "rotate.svg",
    "icons/menu/add_panel.svg": "plus.png",
    "icons/menu/camera.svg": "classes/Camera.png",
    "icons/menu/backend.svg": "default_package_icon.png",
    "icons/menu/orbit.svg": "rotate_cam.svg",
    "icons/menu/xy_orbit.svg": "rotate.svg",
    "icons/menu/top_down.svg": "classes/Grid.png",
    "icons/menu/top_down_ortho.svg": "classes/GridCells.png",
    "icons/menu/third_person.svg": "classes/RobotModel.png",
    "icons/menu/fps.svg": "move2d.svg",
    "icons/menu/opengl.svg": "classes/DepthCloud.png",
    "icons/menu/ogre.svg": "classes/RobotModel.png",
    "icons/menu/help.svg": "classes/Help.svg",
    "icons/menu/about.svg": "classes/Help.svg",
    "icons/menu/settings.svg": "options.png",
    "icons/menu/view.svg": "classes/Views.svg",
    "icons/menu/app.svg": "default_package_icon.png",
    "icons/menu/screenshot.svg": "classes/Camera.png",
    "icons/menu/quit.svg": "close.png",
    "icons/classes/CameraInfo.svg": "classes/Camera.png",
    "icons/classes/Imu.svg": "classes/Effort.png",
    "icons/classes/Wrench.svg": "classes/Wrench.png",
    "icons/classes/Selection.svg": "classes/Selection.png",
}


def png_to_embedded_svg(png_path: Path, svg_path: Path) -> None:
    from PIL import Image

    with Image.open(png_path) as im:
        w, h = im.size
    data = base64.b64encode(png_path.read_bytes()).decode("ascii")
    svg_path.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'xmlns:xlink="http://www.w3.org/1999/xlink" '
        f'width="{w}" height="{h}" viewBox="0 0 {w} {h}">\n'
        f'  <image width="{w}" height="{h}" '
        f'xlink:href="data:image/png;base64,{data}"/>\n'
        f"</svg>\n",
        encoding="utf-8",
    )


def convert_png_file_to_svg(png_path: Path, svg_path: Path) -> None:
    """Wrap PNG as SVG with embedded raster — preserves icon fidelity."""
    png_to_embedded_svg(png_path, svg_path)


def ensure_svg(path: Path) -> Path:
    """If path is PNG, convert beside it to SVG and remove PNG."""
    if path.suffix.lower() != ".png":
        return path
    svg_path = path.with_suffix(".svg")
    convert_png_file_to_svg(path, svg_path)
    path.unlink(missing_ok=True)
    return svg_path


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
    """Copy icon keeping original format (PNG stays PNG, SVG stays SVG)."""
    dest = dest.with_suffix(src.suffix.lower())
    dest.parent.mkdir(parents=True, exist_ok=True)
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
        name = src.stem
        if name == "Tool Properties":
            name = "ToolProperties"
        dest = ICONS / "classes" / (name + src.suffix.lower())
        rel_dest = f"icons/classes/{dest.name}"
        if rel_dest in copied:
            continue
        copy_icon(src, dest)
        copied.add(rel_dest)
        n += 1
    return n


def sync_rviz_root_icons(index: dict[str, Path], copied: set[str]) -> int:
    n = 0
    for base in RVIZ_ICON_ROOTS:
        if not base.is_dir():
            continue
        for path in base.iterdir():
            if not path.is_file():
                continue
            if path.suffix.lower() not in {".png", ".svg"}:
                continue
            dest = ICONS / (path.stem + path.suffix.lower())
            rel_dest = f"icons/{dest.name}"
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
        # Keep source extension under the destination stem.
        dest = (ROOT / dest_rel).with_suffix(src.suffix.lower())
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
            if path.is_file() and path.suffix.lower() in {".png", ".svg", ".json"}:
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
