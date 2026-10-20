# 4. bag_convert

ROS1 `.bag` → autolink `.record`，供 `autolink recorder play` 回放。

### 4.1 用法

```bash
python -m autonomy.tools.bag_convert input.bag -o ./data/records --backpack-2d
python -m autonomy.tools.bag_convert --list-types
python -m autonomy.tools.bag_convert --info input.bag
```

在 `src/autonomy` 根或已配置 `PYTHONPATH` 的环境执行。

### 4.2 预设

| 预设 | 用途 |
|------|------|
| `--backpack-2d` | 2D 激光 + IMU（Cartographer 示例） |
| `--vbr` | `/ouster/points` + `/imu/data`（保留 leading `/`） |

| 常用参数 | 含义 |
|----------|------|
| `-o` | 输出目录 |
| `--duration-sec` | 截取时长 |
| `--list-types` | 支持的 ROS→proto 类型 |

### 4.3 依赖与实现

```bash
python -m pip install -r autonomy/autonomy/tools/bag_convert/requirements.txt
```

包路径：`autonomy/autonomy/tools/bag_convert/`（模块名仍为 `autonomy.tools.bag_convert`）。

Cartographer 流程：[Localization · Cartographer](../06_Localization/07_cartographer/guide.md)。
