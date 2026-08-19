# 配置

Autodriver 使用 **YAML** 配置文件（`autodriver_hardware.yaml`），由 `yaml-cpp` 解析为运行时 `Config`。

唯一配置文件：`config/autodriver_hardware.yaml`。设备均写在顶层 `sensors` 下；进程级默认值在 `Config` 中定义：

| 字段 | 默认值 |
|---|---|
| `node_name` | `autodriver` |
| `plugin_dir` | 空（使用 `AUTODRIVER_PLUGIN_DIR`） |
| `hotplug.enable_udev` | `true` |
| `alignment.enable` | `false` |
| `alignment_window_ms` | 50 |
| `publish_period_ms` | 20 |
| `buffer_capacity` | 32 |

需要覆盖时再写顶层字段，例如：

```yaml
hotplug:
  enable_udev: false
```

## 路径解析

查找顺序（`LoadConfig(configuration_directory, config_basename)`）：

1. 若 `config_basename` 以 `/` 开头 → 视为绝对路径
2. `{WorkRoot}/config/{basename}`
3. 回退 `{prefix}/share/autodriver/config/{basename}`

空文件名默认 `autodriver_hardware.yaml`。

### 环境变量

| 变量 | 含义 |
|---|---|
| `AUTODRIVER_PATH` | 配置根（目录下应有 `config/autodriver_hardware.yaml`） |
| `AUTODRIVER_DISTRIBUTION_HOME` | 安装/发行根 |

```bash
export AUTODRIVER_PATH=/path/to/autodriver
autodriver_hub
```

## 传感器配置总览

所有设备写在顶层 **`sensors`** 下，按类型分组。每条设备为一组 YAML 映射（扁平字段）。

### 支持的类型键

| 键 | 模块 | 默认 `backend` | 运行时 sensor id 前缀 |
|---|---|---|---|
| `lidar_2d` | `Lidar2dModule` | `serial` | `lidar/` |
| `lidar_3d` | `Lidar3dModule` | `serial` | `lidar/` |
| `lidar` | 见 `dimension` | `serial` | `lidar/` |
| `imu` | `ImuModule` | `serial` | `imu/` |
| `gps` | `GpsModule` | `serial` | `gps/` |
| `camera` | `CameraModule` | `realsense` | `camera/` |
| `range` | `RangeModule` | `serial` | `range/` |

合并键 `lidar` 可用 `dimension: 2d|3d`（或 `type`）区分二维/三维雷达。

### 通用字段（所有传感器）

| 字段 | 必填 | 说明 |
|---|---|---|
| `name` | 是 | 设备短名；加载后 id 为 `<前缀><name>`，如 `name: torso_imu` → `imu/torso_imu`。若 `name` 已含 `/` 则原样作为 id |
| `enable` | 否 | `true` 时加载并在进程启动时 attach；`false` 时忽略（默认 `false`）。旧别名 `attach_on_start` 仍兼容 |
| `channel` | 否 | Autolink 发布通道；**字符串或字符串数组**。省略时按类型与 `stream` 自动推导（见下文） |
| `backend` | 否 | 硬件后端；省略时使用上表默认值 |
| `params` | 否 | 额外驱动参数字典，会合并进运行时 `params` |
| `match` | 否 | udev 热插拔匹配规则（见「热插拔匹配」） |
| `module` / `library` | 否 | 高级用法：覆盖插件模块名与 `.so` 路径；通常省略 |

#### `channel` 语义

- **单个字符串**：发布到该 Autolink 话题。
- **字符串数组**：将**同一份**传感器 sample **fan-out** 到多个话题（适合 IMU 等同构多订阅）。
- **不能**用数组表示相机的 color/depth/ir 等多路不同图像流；每路 stream 需单独一条 `camera` 配置。

#### 默认 channel 推导

未写 `channel` 时，Publisher 使用 `ResolveChannel`：

| 类型 | 默认 channel |
|---|---|
| IMU / GPS / Range | `/<id>` |
| Camera（`stream: color` 或省略） | `/<id>/image_raw` |
| Camera（`stream: depth`） | `/<id>/depth/image_raw` |
| Lidar 2D | `/<id>/scan` |
| Lidar 3D | `/<id>/points` |

#### 串口 shorthand

对 `backend: serial`（或未指定 backend 且类型默认为 serial）的设备，下列扁平字段会写入 `params`：

| YAML 字段 | 写入 `params` | 说明 |
|---|---|---|
| `port` | `device` | 串口设备路径，如 `/dev/ttyUSB0`；也可用 `device` |
| `baudrate` | `baud` | 波特率；也可用 `baud` |

上线时建议用 `/dev/serial/by-id/` 固定 `port`，避免 USB 插拔导致编号变化。

#### 嵌套写法（可选）

仍支持将硬件参数写在 `hardware` / `publisher` 子块；`publisher.channel` 仅在顶层未写 `channel` 时作为回退。

```yaml
- name: example
  hardware:
    port: /dev/ttyUSB0
    baudrate: 115200
  publisher:
    channel: /imu/example
    fps: 100
```

---

## lidar_2d — 二维激光雷达

**消息类型**：`sensor_msgs.LaserScan`  
**当前驱动状态**：插件为 attach-only 骨架，串口采集驱动尚未实现；字段为预留配置。

| 字段 | 必填 | 说明 |
|---|---|---|
| `name` | 是 | 如 `front` → id `lidar/front` |
| `enable` | 否 | 是否启用 |
| `channel` | 否 | 默认 `/<id>/scan` |
| `port` | 是* | 串口路径（*启用且 backend 为 serial 时） |
| `baudrate` | 否 | 常见 **115200**（RPLidar / Hokuyo 等）；9600 对多数 2D 雷达偏低 |
| `fps` | 否 | 期望扫描/发布频率 (Hz)，常见 5–20；写入 `publish_rate_hz` |

示例（见 `autodriver_hardware.yaml`）：

```yaml
lidar_2d:
  - name: front
    enable: false
    channel: /lidar/front/scan
    port: /dev/ttyUSB4
    baudrate: 115200
    fps: 10
```

---

## lidar_3d — 三维激光雷达

**消息类型**：`sensor_msgs.PointCloud2`  
**当前驱动状态**：插件为 attach-only 骨架；网口/UDP 驱动尚未实现。

Velodyne 等常见 3D 雷达走**以太网 UDP**，不是 `/dev/ttyUSB`。网口参数建议放在 `params`：

| 字段 | 必填 | 说明 |
|---|---|---|
| `name` | 是 | 如 `vlp16` → id `lidar/vlp16` |
| `enable` | 否 | 是否启用 |
| `channel` | 否 | 默认 `/<id>/points` |
| `fps` | 否 | 期望点云发布频率 (Hz)，VLP-16 常见 10–20 |
| `params.ip` | 否 | 雷达 IP，如 `192.168.1.201` |
| `params.data_port` | 否 | UDP 数据端口，VLP-16 默认 `2368` |
| `params.model` | 否 | 型号标识，供未来驱动使用 |

示例：

```yaml
lidar_3d:
  - name: vlp16
    enable: false
    channel: /lidar/vlp16/points
    fps: 10
    params:
      ip: 192.168.1.201
      data_port: 2368
      model: VLP-16
```

### 合并键 `lidar`

```yaml
lidar:
  - name: front
    dimension: 2d   # 或 type: 2d；3d / lidar3d / pointcloud 表示三维
    port: /dev/ttyUSB4
    baudrate: 115200
```

---

## imu — 惯性测量单元

**消息类型**：`sensor_msgs.Imu`

| 字段 | 必填 | 说明 |
|---|---|---|
| `name` | 是 | 如 `torso_imu` → id `imu/torso_imu` |
| `enable` | 否 | 是否启用 |
| `channel` | 否 | 字符串或数组；数组表示同一份 IMU 数据发布到多个话题 |
| `backend` | 否 | `serial`（默认）、`can`、`realsense` |
| `port` | serial 时 | 串口路径 |
| `baudrate` | serial 时 | 常见 115200 / 460800 / 921600，需与模块输出速率匹配 |
| `fps` | 否 | 期望发布频率 (Hz)；写入 `publish_rate_hz`（串口驱动当前按硬件实际速率输出，尚未限频） |

### backend: serial

| 字段 | 说明 |
|---|---|
| `port` / `baudrate` | 见串口 shorthand |
| `params.accel_scale` / `gyro_scale` | WitMotion 等协议换算系数 |

### backend: can

| 字段 | 说明 |
|---|---|
| `interface` | CAN 网卡，如 `can0` |
| `accel_can_id` / `gyro_can_id` | 加速度/陀螺 CAN ID |
| `can_id` | 合并帧 ID |
| `accel_scale` / `gyro_scale` | 换算系数 |

### backend: realsense

用于 RealSense D435i / D455 等**板载 IMU**，与 `camera` 条目共用同一物理设备（RealSense hub 按 `serial` 或 `index+model` 合并）。

| 字段 | 说明 |
|---|---|
| `model` | 可选；型号过滤子串，如 `D455`。单台设备可省略 |
| `params.serial` | 可选；设备序列号，多台 RealSense 时必填 |
| `params.index` | 可选；同型号设备索引，默认 `0` |

示例：

```yaml
imu:
  - name: torso_imu
    enable: false
    channel:
      - /imu/torso
      - /imu/torso/raw
    port: /dev/ttyUSB0
    baudrate: 460800
    fps: 200

  - name: realsense_d455_imu
    enable: false
    channel: /camera/imu
    backend: realsense
```

---

## gps — 全球导航卫星系统

**消息类型**：`sensor_msgs.NavSatFix`

| 字段 | 必填 | 说明 |
|---|---|---|
| `name` | 是 | 如 `gps_main` → id `gps/gps_main` |
| `enable` | 否 | 是否启用 |
| `channel` | 否 | 常见 `/gps/fix` 或 `/<id>` |
| `backend` | 否 | `serial`（默认）、`can` |
| `port` | serial 时 | 串口路径 |
| `baudrate` | serial 时 | NMEA 模块出厂常见 **9600**；部分 u-blox 配置为 115200 |

`backend: can` 时使用 `interface`、`can_id` 等，见 IMU CAN 字段说明。

示例：

```yaml
gps:
  - name: gps_main
    enable: false
    channel: /gps/fix
    port: /dev/ttyUSB2
    baudrate: 9600
```

---

## camera — 相机

**消息类型**：`sensor_msgs.Image`

| 字段 | 必填 | 说明 |
|---|---|---|
| `name` | 是 | 如 `realsense_d455_color` → id `camera/realsense_d455_color` |
| `enable` | 否 | 是否启用 |
| `channel` | 否 | 单路图像的 Autolink 话题 |
| `backend` | 否 | 当前实现 **`realsense`**（默认） |
| `stream` | realsense 时 | 视频流类型（见下表） |
| `width` / `height` | realsense 时 | 分辨率 |
| `fps` | realsense 时 | 帧率，**会传入 RealSense pipeline** |

### RealSense `stream` 取值

| `stream` | 含义 | 典型 channel（ROS 风格） |
|---|---|---|
| `color` / `rgb`（默认） | RGB 彩色 | `/camera/color/image_raw` |
| `depth` / `z16` | 深度 | `/camera/depth/image_raw` |
| `ir1` / `infrared1` / `ir` | 红外 1 | `/camera/infra1/image_raw` |
| `ir2` / `infrared2` | 红外 2 | `/camera/infra2/image_raw` |

**多路 stream（color + depth + ir）必须写多条 `camera` 配置**，每条一个 `stream` 和一个 `channel`。同一 D455 的多条目共用 RealSense hub（相同 `params.serial` 或相同 `index`+`model`）。

| 字段 | 说明 |
|---|---|
| `model` | 可选；如 `D455`。单台设备可省略 |
| `params.serial` | 可选；多台 RealSense 时按序列号绑定 |

D455 常用分辨率 **848×480 @ 30fps**（深度与彩色对齐）。

示例：

```yaml
camera:
  - name: realsense_d455_color
    enable: false
    channel: /camera/color/image_raw
    backend: realsense
    stream: color
    width: 848
    height: 480
    fps: 30

  - name: realsense_d455_depth
    enable: false
    channel: /camera/depth/image_raw
    backend: realsense
    stream: depth
    width: 848
    height: 480
    fps: 30
```

---

## range — 测距传感器

**消息类型**：`sensor_msgs.Range`  
**当前驱动状态**：插件为 attach-only 骨架，串口驱动尚未实现。

| 字段 | 必填 | 说明 |
|---|---|---|
| `name` | 是 | 如 `range_main` → id `range/range_main` |
| `enable` | 否 | 是否启用 |
| `channel` | 否 | 默认 `/<id>` |
| `port` | 是* | 串口路径（*启用且 backend 为 serial 时） |
| `baudrate` | 否 | 超声波/红外模块常见 9600 或 115200 |
| `fps` | 否 | 期望发布频率 (Hz)，常见 10 |

示例：

```yaml
range:
  - name: range_main
    enable: false
    channel: /range/front
    port: /dev/ttyUSB5
    baudrate: 9600
    fps: 10
```

---

## 热插拔匹配

每条传感器可配置 `match`，供 Linux udev 热插拔匹配（`hotplug.enable_udev: true` 时）：

| 字段 | 说明 |
|---|---|
| `match.subsystem` | 如 `tty`、`usb` |
| `match.device` | 设备节点，如 `/dev/ttyUSB0` |
| `match.vendor` / `match.product` | USB 厂商/产品 ID（十六进制字符串） |
| `match.serial` | 设备序列号 |

配置了 `port` 的 serial 设备会自动填充 `match.subsystem=tty` 与 `match.device=<port>`（若未显式写 `match`）。

无 `match` 的条目不参与 udev，仅响应 `enable: true` 或显式 `Attach`。

---

## 默认内容

`config/autodriver_hardware.yaml` 包含上述各类传感器的示例条目（默认均为 `enable: false`）。

参考 schema：`autodriver/proto/autodriver_conf.proto`（文档用，运行时读 YAML）。

## 代码加载

```cpp
#include "autodriver/config_loader.hpp"

autodriver::Config config = autodriver::LoadConfig();
config = autodriver::LoadConfig("/opt/autodriver",
                                autodriver::kDefaultConfigBasename);
```
