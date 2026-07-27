#!/usr/bin/env python3
"""Fill Autoviz core zh_CN translations in autoviz_zh_CN.ts."""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

_TOOLS_DIR = Path(__file__).resolve().parents[1]
if str(_TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(_TOOLS_DIR))

from _bootstrap import ensure_tools_dir

ensure_tools_dir(__file__)

from common import find_autoviz_root, log_info, log_ok, translations_dir

AUTOVIZ_CORE_ZH_CN: dict[str, str] = {
    "...": "...",
    "+0.1s": "+0.1 秒",
    "−0.1s": "−0.1 秒",
    "%1 (autoviz)": "%1（autoviz）",
    "%1 Error(s)": "%1 个错误",
    "%1 m/s · %2°/s": "%1 米/秒 · %2°/秒",
    "%1 Warn(s)": "%1 个警告",
    "&About": "关于(&A)",
    "Active Tool: %1": "当前工具：%1",
    "Add": "添加",
    "Add a tool to the toolbar": "向工具栏添加工具",
    "Add &New Panel": "添加新面板(&N)",
    "Apply the selected transformation plugin (RViz-style).": (
        "应用所选变换插件（RViz 风格）。"
    ),
    "Approximate": "近似",
    "Autolink OK | channels: %1 | Fixed Frame: %2 | View: %3 | Tool: %4": (
        "Autolink 正常 | 通道：%1 | 固定坐标系：%2 | 视图：%3 | 工具：%4"
    ),
    "Autolink Record (*.record);;All Files (*)": (
        "Autolink 记录 (*.record);;所有文件 (*)"
    ),
    "Autoviz Config (*.autoviz)": "Autoviz 配置 (*.autoviz)",
    "Autoviz Config (*.autoviz *.yaml);;RViz Config (*.rviz);;All Files (*)": (
        "Autoviz 配置 (*.autoviz *.yaml);;RViz 配置 (*.rviz);;所有文件 (*)"
    ),
    "Autoviz replays Autolink .record files.\n\nConvert legacy bag offline:\n  bag_to_record input.bag output.record\n\nThen open output.record in Playback.": (
        "Autoviz 可回放 Autolink .record 文件。\n\n"
        "离线转换 legacy bag：\n  bag_to_record input.bag output.record\n\n"
        "然后在回放中打开 output.record。"
    ),
    "Background color for the 3D view.": "3D 视图背景颜色。",
    "bag_to_record failed (code %1).\n%2": "bag_to_record 失败（代码 %1）。\n%2",
    "bag_to_record timed out.": "bag_to_record 超时。",
    "Browse...": "浏览...",
    "Channels: —": "通道：—",
    "Channels: %1": "通道：%1",
    "Configure the display frame, background, and other global options.": (
        "配置显示坐标系、背景色及其他全局选项。"
    ),
    "Convert a legacy .bag file to Autolink .record using bag_to_record (Autolink developer tools), then open the result.": (
        "使用 bag_to_record（Autolink 开发者工具）将 legacy .bag 转为 Autolink .record，然后打开结果。"
    ),
    "Convert an .mcap recording to Autolink .record, then open the result.": (
        "将 .mcap 录制转为 Autolink .record，然后打开结果。"
    ),
    "Converted record could not be opened:\n%1": "无法打开转换后的记录：\n%1",
    "Convert Required": "需要转换",
    "Could not find `bag_to_record` in PATH.\n\nInstall Autolink developer tools, then run:\n  bag_to_record %1 %2": (
        "PATH 中未找到 `bag_to_record`。\n\n"
        "请安装 Autolink 开发者工具后运行：\n  bag_to_record %1 %2"
    ),
    "Could not open record file:\n%1": "无法打开记录文件：\n%1",
    "Could not open record file:\n%1\n\nEnsure the file is a valid Autolink .record.": (
        "无法打开记录文件：\n%1\n\n请确认文件为有效的 Autolink .record。"
    ),
    "Create visualization": "创建可视化",
    "Current View": "当前视图",
    "&Delete Panel": "删除面板(&D)",
    "Description:": "描述：",
    "Display name:": "Display 名称：",
    "Displays": "Display",
    "Duplicate": "复制",
    "Elapsed:": "已播放：",
    "Enable or disable this display.": "启用或禁用此 Display。",
    "Enable or disable this nested display.": "启用或禁用此嵌套 Display。",
    "Exact": "精确",
    "Experimental": "实验性",
    "Failed to start bag_to_record.": "启动 bag_to_record 失败。",
    "Failed to start MCAP converter.": "启动 MCAP 转换器失败。",
    "&File": "文件(&F)",
    "<Fixed Frame>": "<固定坐标系>",
    "Focal Point": "焦点",
    "Focal Shape Fixed Size": "焦点形状固定大小",
    "Focal Shape Size": "焦点形状大小",
    "Frame into which all data is transformed before being displayed.": (
        "显示前将所有数据变换到的坐标系。"
    ),
    "Freeze sim time.": "冻结仿真时间。",
    "&Fullscreen": "全屏(&F)",
    "Global Status": "全局状态",
    "&Help": "帮助(&H)",
    "Help": "帮助",
    "Import...": "导入...",
    "Import Failed": "导入失败",
    "Import Legacy Bag (.bag)": "导入 Legacy Bag (.bag)",
    "Import MCAP (.mcap)": "导入 MCAP (.mcap)",
    "Import / Open Record": "导入 / 打开记录",
    "Invert Z Axis": "反转 Z 轴",
    "Jump to 0,0,0 with the current view controller. Shortcut: Z": (
        "使用当前视图控制器跳转到 0,0,0。快捷键：Z"
    ),
    "Legacy Bag (*.bag);;All Files (*)": "Legacy Bag (*.bag);;所有文件 (*)",
    "Loop": "循环",
    "MCAP conversion failed.\n%1": "MCAP 转换失败。\n%1",
    "MCAP converter script or python3 not found.\n\nConvert offline with Autolink tools, then open the .record file:\n  mcap_to_record.py %1 %2": (
        "未找到 MCAP 转换脚本或 python3。\n\n"
        "请用 Autolink 工具离线转换后打开 .record：\n  mcap_to_record.py %1 %2"
    ),
    "MCAP converter timed out.": "MCAP 转换器超时。",
    "MCAP (*.mcap);;All Files (*)": "MCAP (*.mcap);;所有文件 (*)",
    "Near Clip Distance": "近裁剪距离",
    "New Name?": "新名称？",
    "No configurable properties": "无可配置属性",
    "\nOgre rendering backend enabled.": "\n已启用 Ogre 渲染后端。",
    "No image": "无图像",
    "(none)": "（无）",
    "No record loaded": "未加载记录",
    "No stderr output.": "无 stderr 输出。",
    "Ok": "确定",
    "OK %1": "确定 %1",
    "Open": "打开",
    "Open...": "打开...",
    "Open an existing Autolink .record file for playback.": (
        "打开已有 Autolink .record 文件进行回放。"
    ),
    "Open Autolink Record": "打开 Autolink 记录",
    "Open Autolink Record (.record)": "打开 Autolink 记录 (.record)",
    "&Open Config": "打开配置(&O)",
    "Open Failed": "打开失败",
    "Output .record (import modes):": "输出 .record（导入模式）：",
    "&Panels": "面板(&P)",
    "Parent": "父级",
    "Paused": "已暂停",
    "Pick Color": "选择颜色",
    "Playing": "播放中",
    "PNG Image (*.png)": "PNG 图像 (*.png)",
    "&Quit": "退出(&Q)",
    "Rate": "倍率",
    "&Recent Configs": "最近配置(&R)",
    "Remove a tool from the toolbar": "从工具栏移除工具",
    "Rename": "重命名",
    "Rename Display": "重命名 Display",
    "Rename View": "重命名视图",
    "Resume": "继续",
    "Save Autolink Record": "保存 Autolink 记录",
    "Save Autoviz Config": "保存 Autoviz 配置",
    "&Save Config": "保存配置(&S)",
    "Save Config &As": "配置另存为(&A)",
    "Save &Image": "保存图像(&I)",
    "See share/autonomy/autoviz/scripts/mcap_to_record.py.": (
        "参见 share/autonomy/autoviz/scripts/mcap_to_record.py。"
    ),
    "Select an output .record path.": "请选择输出 .record 路径。",
    "Select a source file.": "请选择源文件。",
    "Selected Points": "选中点",
    "Select File": "选择文件",
    "Selection": "选择",
    "Select Source File": "选择源文件",
    "Show &Help panel": "显示帮助面板(&H)",
    "Show or hide the reference grid in the 3D view.": "显示或隐藏 3D 视图参考网格。",
    "Shows aggregated errors and warnings from displays.": "显示各 Display 聚合的错误与警告。",
    "Sim Time: 0.000 / 0.000 s": "仿真时间：0.000 / 0.000 秒",
    "Sim Time: %1 / %2 s": "仿真时间：%1 / %2 秒",
    "Source:": "来源：",
    "Source file:": "源文件：",
    "Status": "状态",
    "Stopped": "已停止",
    "Synchronization:": "同步：",
    "Synchronize sim time and TF transforms to a given source.": (
        "将仿真时间与 TF 变换同步到指定来源。"
    ),
    "Target Frame": "目标坐标系",
    "Target number of frames to try to render each second.": (
        "每秒尝试渲染的目标帧数。"
    ),
    "Time:": "时间：",
    "Time Source: Sim Time": "时间源：仿真时间",
    "Time source to use for synchronization.": "用于同步的时间源。",
    "Topic or channel name for incoming data.": "输入数据的 topic 或通道名。",
    "Type:": "类型：",
    "Use Import mode for .bag or .mcap files.": "对 .bag 或 .mcap 文件请使用导入模式。",
    "View %1": "视图 %1",
    "Views": "视图",
    "Wall Elapsed:": "墙钟已播放：",
    "Wall Time:": "墙钟时间：",
    "X": "X",
    "Y": "Y",
    "Z": "Z",
    "Zero": "归零",
    "Mode": "模式",
}


def apply_autoviz_core_zh_cn(path: Path, *, force: bool = True) -> tuple[int, int]:
    tree = ET.parse(path)
    filled = 0
    partial_kept = 0

    for ctx in tree.getroot().findall("context"):
        name_el = ctx.find("name")
        if name_el is None or name_el.text is None:
            continue

        for msg in ctx.findall("message"):
            source_el = msg.find("source")
            if source_el is None or source_el.text is None:
                continue
            translation_el = msg.find("translation")
            if translation_el is None:
                translation_el = ET.SubElement(msg, "translation")

            source = source_el.text
            current = (translation_el.text or "").strip()
            unfinished = translation_el.attrib.get("type") == "unfinished"

            if source in AUTOVIZ_CORE_ZH_CN:
                if not force and not unfinished and current and current != source:
                    continue
                translation_el.text = AUTOVIZ_CORE_ZH_CN[source]
                translation_el.attrib.pop("type", None)
                filled += 1
            elif unfinished and current:
                translation_el.text = current
                translation_el.attrib.pop("type", None)
                partial_kept += 1

    tree.write(path, encoding="utf-8", xml_declaration=True)
    return filled, partial_kept


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ts", type=Path, default=None)
    args = parser.parse_args()

    autoviz_root = find_autoviz_root()
    ts_path = args.ts or translations_dir(autoviz_root) / "autoviz_zh_CN.ts"
    if not ts_path.is_file():
        print(f"Error: {ts_path} not found", file=sys.stderr)
        return 1

    filled, partial = apply_autoviz_core_zh_cn(ts_path)
    log_info(f"Autoviz core zh_CN: {filled} translated, {partial} partial kept")
    log_ok(f"Updated {ts_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
