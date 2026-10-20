# 4. bag_convert

ROS1 `.bag` → autolink `.record`。

**什么时候用：** 手里是 ROS1 `.bag`，要在 autolink `recorder play` 或 Cartographer 示例里回放。

### 用法

在 **`src/autonomy` 根**或已设 `PYTHONPATH` 的环境：

```bash
python -m autonomy.tools.bag_convert input.bag -o ./data/records --backpack-2d
python -m autonomy.tools.bag_convert --list-types
python -m autonomy.tools.bag_convert --info input.bag
```

### 预设

| 预设 | 用途 |
|------|------|
| `--backpack-2d` | 2D 激光 + IMU（Cartographer 示例） |
| `--vbr` | `/ouster/points` + `/imu/data`（保留 leading `/`） |

| 参数 | 含义 |
|------|------|
| `-o` | 输出目录 |
| `--duration-sec` | 截取时长 |

### 依赖

```bash
python -m pip install -r autonomy/autonomy/tools/bag_convert/requirements.txt
```

源码目录：`autonomy/autonomy/tools/bag_convert/`（模块名 `autonomy.tools.bag_convert`）。

### 常见坑

| 现象 | 说明 |
|------|------|
| No module named autonomy.tools | 未在 src/autonomy 根或未安装 editable |
| 播 record 无激光 | 话题名与 preset 不一致；用 `--info` 对 bag |

Cartographer：[Localization · Cartographer](../06_Localization/07_cartographer/guide.md)。
