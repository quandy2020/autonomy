#!/usr/bin/env python3
"""
将 MovingAI 等文本 .map 文件批量转换为 PGM + YAML，
使其与 city 目录下的地图格式一致。

处理的目录：
- maze/maze-map
- random/random-map
- terrain/weighted-map
- room/room-map
- warcraft
- starcraft
- dragon
- baldurs

输出：
- 对每个 *.map 生成同名的 *.pgm 和 *.yaml，保存在原目录下。
"""

import os
from pathlib import Path
from typing import List


BASE_DIR = Path(__file__).resolve().parent


def read_grid_from_map(path: Path) -> List[str]:
    """
    读取 .map 文本文件，返回纯栅格行（不含 header）。

    兼容 MovingAI 格式：
        type octile
        height 512
        width 512
        map
        ....@...
        ...
    """
    lines = path.read_text().splitlines()
    # 去掉空行
    lines = [ln.rstrip("\n") for ln in lines if ln.strip() != ""]

    # 寻找 'map' 行
    grid_start = 0
    for i, ln in enumerate(lines):
        if ln.strip().lower() == "map":
            grid_start = i + 1
            break

    grid = lines[grid_start:]
    if not grid:
        raise RuntimeError(f"{path} 中未找到有效栅格数据")
    return grid


def grid_to_pgm_values(grid: List[str]) -> List[List[int]]:
    """
    将字符栅格转为 PGM 灰度值（0~255）。

    简单约定（足够用于路径规划示例）：
    - '.'、'G'、'S' 视为可通行 => 高灰度（接近白）
    - 其它字符视为障碍 => 低灰度（黑）
    """
    free_val = 254  # 接近白
    occ_val = 0    # 障碍为黑色

    rows = []
    for row in grid:
        vals = []
        for c in row:
            if c in (".", "G", "S"):
                vals.append(free_val)
            else:
                vals.append(occ_val)
        rows.append(vals)
    return rows


def write_pgm(path: Path, grid_vals: List[List[int]]) -> None:
    """
    写出简单 ASCII PGM (P2) 文件，方便任何工具读取。
    """
    height = len(grid_vals)
    width = len(grid_vals[0]) if height > 0 else 0
    maxval = 255

    lines = ["P2", f"{width} {height}", str(maxval)]
    for row in grid_vals:
        lines.append(" ".join(str(v) for v in row))

    path.write_text("\n".join(lines) + "\n")


def write_yaml(path: Path, image_name: str, resolution: float = 0.05) -> None:
    """
    生成与 city 目录相同格式的 YAML 文件。

    示例 (Berlin_0_512.yaml)：
        image: "Berlin_0_512.pgm"
        resolution: 0.05
        origin: [0.0, 0.0, 0.0]
        occupied_thresh: 0.65
        free_thresh: 0.196
        negate: 0
        mode: "trinary"
    """
    content = f'''image: "{image_name}"
resolution: {resolution}
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
mode: "trinary"
'''
    path.write_text(content)


def convert_dir(map_dir: Path, pattern: str = "*.map", resolution: float = 0.05) -> None:
    """
    将某个目录中的所有 .map 转为 pgm + yaml。
    """
    if not map_dir.is_dir():
        print(f"[WARN] 目录不存在，跳过: {map_dir}")
        return

    maps = sorted(map_dir.glob(pattern))
    if not maps:
        print(f"[INFO] 目录中未找到 {pattern}: {map_dir}")
        return

    print(f"[INFO] 转换目录: {map_dir} ({len(maps)} 个 .map)")

    for m in maps:
        try:
            grid = read_grid_from_map(m)
            vals = grid_to_pgm_values(grid)

            pgm_path = m.with_suffix(".pgm")
            yaml_path = m.with_suffix(".yaml")

            write_pgm(pgm_path, vals)
            write_yaml(yaml_path, pgm_path.name, resolution=resolution)

            print(f"  [OK] {m.name} -> {pgm_path.name}, {yaml_path.name}")
        except Exception as e:
            print(f"  [ERR] 处理 {m} 失败: {e}")


def main() -> None:
    # 基础四类地图目录
    maze_dir = BASE_DIR / "maze" / "maze-map"
    random_dir = BASE_DIR / "random" / "random-map"
    terrain_dir = BASE_DIR / "terrain" / "weighted-map"
    room_dir = BASE_DIR / "room" / "room-map"

    # 额外四类地图目录（平铺的 .map 文件）
    warcraft_dir = BASE_DIR / "warcraft"
    warcraft_wc3_dir = BASE_DIR / "warcraft" / "wc3maps512-map"
    starcraft_dir = BASE_DIR / "starcraft"
    dragon_dir = BASE_DIR / "dragon"
    baldurs_dir = BASE_DIR / "baldurs"

    # 分别转换到各自目录下（参考 city 分辨率 0.05）
    convert_dir(maze_dir, "*.map", resolution=0.05)
    convert_dir(random_dir, "*.map", resolution=0.05)
    convert_dir(terrain_dir, "*.map", resolution=0.05)
    convert_dir(room_dir, "*.map", resolution=0.05)
    # warcraft 根目录可能没有 .map，重点转换 wc3maps512-map 子目录
    convert_dir(warcraft_dir, "*.map", resolution=0.05)
    convert_dir(warcraft_wc3_dir, "*.map", resolution=0.05)
    convert_dir(starcraft_dir, "*.map", resolution=0.05)
    convert_dir(dragon_dir, "*.map", resolution=0.05)
    convert_dir(baldurs_dir, "*.map", resolution=0.05)


if __name__ == "__main__":
    main()


