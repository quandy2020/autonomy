# 4. bag_convert

ROS1 `.bag` → autolink `.record`。

**什么时候用：** 手里是 ROS1 `.bag`，要在 autolink `recorder play` 或 Cartographer 示例里回放。

### 用法

在 **`src/autonomy` 根**（容器 `/workspace/autonomy/src/autonomy`）或已设 `PYTHONPATH` 的环境。Isaac 用 `/usr/bin/python3`，不要用 Kit 的 `python`：

```bash
cd /workspace/autonomy/src/autonomy
export PYTHONPATH="${PWD}:${PYTHONPATH:-}"
/usr/bin/python3 -m autonomy.tools.bag_convert input.bag -o ./data/records --backpack-2d
/usr/bin/python3 -m autonomy.tools.bag_convert --list-types
/usr/bin/python3 -m autonomy.tools.bag_convert --info input.bag
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
| `--compress` | chunk 压缩：`lz4`（默认）、`bz2`、`none`。旧的未压缩 record 仍可播放 |

### 依赖

宿主机（不要用 Anaconda 3.7）：

```bash
cd /home/quandy/workspace/github/autonomy/src/autonomy
/usr/bin/python3.12 -m pip install --user --break-system-packages \
  -r autonomy/tools/bag_convert/requirements.txt
```

SpaceHero 容器：

```bash
cd /workspace/autonomy/src/autonomy
/usr/bin/python3 -m pip install -r autonomy/tools/bag_convert/requirements.txt
```

当前目录若是 `/workspace/autonomy/src`，改为 `-r autonomy/autonomy/tools/bag_convert/requirements.txt`。

源码目录：`autonomy/autonomy/tools/bag_convert/`（模块名 `autonomy.tools.bag_convert`）。

### 常见坑

| 现象 | 说明 |
|------|------|
| No module named autonomy.tools | 未在 src/autonomy 根或未安装 editable |
| 播 record 无激光 | 话题名与 preset 不一致；用 `--info` 对 bag |

Cartographer：[Localization · Cartographer](../06_Localization/07_cartographer/guide.md)。
