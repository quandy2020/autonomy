#!/usr/bin/env python3
"""Update Autoviz Qt translation catalogs (lupdate + optional QGC merge).

Run from anywhere in the repo:
    python3 tools/translations/autoviz_lupdate.py

Optionally merge matching strings from QGroundControl:
    python3 tools/translations/autoviz_lupdate.py --qgc-translations /path/to/qgc/translations
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

_TOOLS_DIR = Path(__file__).resolve().parents[1]
if str(_TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR))

from _bootstrap import ensure_tools_dir

ensure_tools_dir(__file__)

from common import default_qgc_translations_dir, find_autoviz_root, log_info, log_ok, translations_dir
from autoviz_core_zh_cn_fill import apply_autoviz_core_zh_cn

LOCALES = (
    "az_AZ",
    "bg_BG",
    "de_DE",
    "el_GR",
    "eo",
    "es_ES",
    "fi_FI",
    "fr_FR",
    "he_IL",
    "it_IT",
    "ja_JP",
    "ko_KR",
    "nb_NO",
    "nl_NL",
    "no_NO",
    "pl_PL",
    "pt_PT",
    "ru_RU",
    "sv_SE",
    "tr_TR",
    "uk_UA",
    "zh_CN",
    "zh_TW",
)

LUPDATE_SOURCES = ("autoviz", "qml")


def resolve_lupdate() -> Path:
    for name in ("lupdate-qt6", "lupdate"):
        if found := shutil.which(name):
            return Path(found)
    raise FileNotFoundError("lupdate not found; install qt6-tools-dev / qttools5-dev-tools")


def locale_ts_path(root: Path, locale: str) -> Path:
    return translations_dir(root) / f"autoviz_{locale}.ts"


def locale_to_ts_language(locale: str) -> str:
    return locale.replace("_", "-")


def ensure_locale_files(root: Path) -> list[Path]:
    base_ts = translations_dir(root) / "autoviz.ts"
    trans_dir = translations_dir(root)
    trans_dir.mkdir(parents=True, exist_ok=True)
    locale_files: list[Path] = []
    for locale in LOCALES:
        path = locale_ts_path(root, locale)
        if not path.exists() and base_ts.exists():
            text = base_ts.read_text(encoding="utf-8")
            text = text.replace(
                '<TS version="2.1">',
                f'<TS version="2.1" language="{locale_to_ts_language(locale)}" '
                f'sourcelanguage="en">',
                1,
            )
            path.write_text(text, encoding="utf-8")
        locale_files.append(path)
    return locale_files


def run_lupdate(lupdate: Path, root: Path, ts_files: list[Path]) -> None:
    base_ts = translations_dir(root) / "autoviz.ts"
    cmd = [str(lupdate), *LUPDATE_SOURCES, "-no-obsolete", "-ts", str(base_ts)]
    cmd.extend(str(path) for path in ts_files)
    log_info(f"Running: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=root, check=True)


def parse_qgc_translations(path: Path) -> dict[str, str]:
    if not path.is_file():
        return {}

    tree = ET.parse(path)
    out: dict[str, str] = {}
    for message in tree.getroot().iter("message"):
        source_el = message.find("source")
        translation_el = message.find("translation")
        if source_el is None or translation_el is None or source_el.text is None:
            continue
        translation = translation_el.text or ""
        if not translation or translation_el.attrib.get("type") == "unfinished":
            continue
        if translation != source_el.text:
            out[source_el.text] = translation
    return out


def merge_qgc_into_aviz(autoviz_path: Path, qgc_map: dict[str, str]) -> int:
    if not qgc_map:
        return 0

    tree = ET.parse(autoviz_path)
    merged = 0
    for message in tree.getroot().iter("message"):
        source_el = message.find("source")
        translation_el = message.find("translation")
        if source_el is None or source_el.text is None:
            continue
        if translation_el is None:
            translation_el = ET.SubElement(message, "translation")
        source = source_el.text
        current = translation_el.text or ""
        if source not in qgc_map:
            continue
        if current and translation_el.attrib.get("type") != "unfinished" and current != source:
            continue
        translation_el.text = qgc_map[source]
        translation_el.attrib.pop("type", None)
        merged += 1

    tree.write(autoviz_path, encoding="utf-8", xml_declaration=True)
    return merged


def apply_autoviz_zh_cn_overrides(path: Path) -> None:
    overrides = {
        "Vehicle 3D": "三维载具",
        "Ground (diff-drive)": "地面（差速驱动）",
        "Ground (Ackermann)": "地面（阿克曼）",
        "Drone (simple)": "无人机（简易）",
        "Drone (F450 mesh)": "无人机（F450 模型）",
        "Model:": "模型：",
        "TF frame:": "TF 坐标系：",
        "Follow": "跟随",
        "Origin": "重置原点",
        "No TF": "无 TF",
        "Autolink Channels": "Autolink 通道",
        "Tool Properties": "工具属性",
        "TF Tree": "TF 树",
        "Transformation": "变换",
        "Playback": "回放",
        "Show unvisualizable topics": "显示不可视化话题",
        "Filter topics by name:": "按名称过滤话题：",
        "By display type": "按 Display 类型",
        "By channel": "按通道",
        "Display Name": "Display 名称",
        "Select a Display type.": "请选择 Display 类型。",
        "Enter a name for the display.": "请输入 Display 名称。",
        "Name in use. Display names must be unique.": "名称已占用，Display 名称必须唯一。",
        "Add Panel": "添加面板",
        "All panels are already visible.": "所有面板均已显示。",
        "Open Config": "打开配置",
        "Save Config": "保存配置",
        "Failed to load config:\n%1": "加载配置失败：\n%1",
        "Failed to save config:\n%1": "保存配置失败：\n%1",
        "Save Image": "保存图像",
        "About": "关于",
        "This is Autoviz.\n\nCompiled against Qt version %1.": "这是 Aviz。\n\n基于 Qt %1 编译。",
        "No hardware GPU detected; using OpenGL backend.": "未检测到硬件 GPU，使用 OpenGL 后端。",
        "Fixed Frame": "固定坐标系",
        "Background Color": "背景颜色",
        "Show Grid": "显示网格",
        "Frame Rate": "帧率",
        "Global Options": "全局选项",
    }
    tree = ET.parse(path)
    for message in tree.getroot().iter("message"):
        source_el = message.find("source")
        translation_el = message.find("translation")
        if source_el is None or source_el.text not in overrides:
            continue
        if translation_el is None:
            translation_el = ET.SubElement(message, "translation")
        translation_el.text = overrides[source_el.text]
        translation_el.attrib.pop("type", None)
    tree.write(path, encoding="utf-8", xml_declaration=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--qgc-translations",
        type=Path,
        default=None,
        help="Path to QGroundControl translations/ directory",
    )
    args = parser.parse_args()

    try:
        lupdate = resolve_lupdate()
    except FileNotFoundError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    autoviz_root = find_autoviz_root()
    locale_files = ensure_locale_files(autoviz_root)
    run_lupdate(lupdate, autoviz_root, locale_files)

    qgc_dir = args.qgc_translations or default_qgc_translations_dir()
    if qgc_dir.is_dir():
        log_info(f"Merging matching strings from {qgc_dir}")
        for locale in LOCALES:
            qgc_path = qgc_dir / f"qgc_source_{locale}.ts"
            autoviz_path = locale_ts_path(autoviz_root, locale)
            merged = merge_qgc_into_aviz(autoviz_path, parse_qgc_translations(qgc_path))
            log_info(f"  {locale}: {merged} strings from QGC")
        apply_autoviz_zh_cn_overrides(locale_ts_path(autoviz_root, "zh_CN"))
        core_filled, core_partial = apply_autoviz_core_zh_cn(
            locale_ts_path(autoviz_root, "zh_CN"))
        log_info(
            f"Autoviz core zh_CN: {core_filled} translated, {core_partial} partial kept"
        )
    else:
        log_info(f"QGC translations not found at {qgc_dir}; skipping merge")
        core_filled, core_partial = apply_autoviz_core_zh_cn(
            locale_ts_path(autoviz_root, "zh_CN"))
        log_info(
            f"Autoviz core zh_CN: {core_filled} translated, {core_partial} partial kept"
        )

    log_ok("Autoviz translation files updated.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
