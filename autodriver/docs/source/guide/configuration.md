# 配置

Autodriver 使用 **YAML**（`yaml-cpp`）解析为运行时 `autodriver::Config`。

默认文件：`config/autodriver_hardware.yaml`。设备写在顶层 `sensors` 下；仅 `enable: true`（或旧别名 `attach_on_start`）的条目会进入 `Config`。

## 进程级字段

写在 YAML 根节点（均可省略，使用默认值）：

| YAML 键 | 运行时 | 默认 | 说明 |
|---|---|---|---|
| `node_name` | `node_name` | `autodriver` | Autolink 节点名 |
| `plugin_dir` | `plugins` | 空 | 外置 `.so` 搜索目录；空则用编译宏 `AUTODRIVER_PLUGIN_DIR` |
| `hotplug.enable_udev` | `hotplug.udev` | `true` | 是否启动 udev 线程 |
| `alignment.enable` | `alignment.enable` | `false` | 是否启用 SensorHub 对齐 |
| `alignment.alignment_window_ms` | `options.alignment_window` | `50` | 对齐窗口 (ms) |
| `alignment.publish_period_ms` | `options.publish_period` | `20` | 对齐发布周期 (ms) |
| `alignment.buffer_capacity` | `options.buffer_capacity` | `32` | 每路样本缓冲容量 |
| `compensator.pose_channel` | `compensator.pose_channel` | 空 | `nav_msgs/Odometry` 通道；空则不启 PoseFeeder |

示例：

```yaml
node_name: autodriver
hotplug:
  enable_udev: false
alignment:
  enable: false
  alignment_window_ms: 50
  publish_period_ms: 20
  buffer_capacity: 32
compensator:
  pose_channel: /localization/odom   # 可选；enable_compensator 的 lidar 用
```

## 路径解析

`LoadConfig(configuration_directory, config_basename)` 查找顺序：

1. `config_basename` 以 `/` 开头 → 绝对路径
2. `{WorkRoot}/config/{basename}`
3. 回退 `{prefix}/share/autodriver/config/{basename}`

空文件名默认 `autodriver_hardware.yaml`。`WorkRoot` 优先取 `AUTODRIVER_PATH`。

| 变量 | 含义 |
|---|---|
| `AUTODRIVER_PATH` | 配置根（目录下应有 `config/autodriver_hardware.yaml`） |
| `AUTODRIVER_DISTRIBUTION_HOME` | 安装/发行根（安装树回退） |

```bash
export AUTODRIVER_PATH=/path/to/autodriver
autodriver
```

## 传感器类型键

| YAML 键 | Module | 默认 `backend` | 运行时 id 前缀 | 采集状态 |
|---|---|---|---|---|
| `lidar_2d` | `Lidar2dModule` | `serial` | `lidar/` | attach-only 骨架 |
| `lidar_3d` | `Lidar3dModule` | `velodyne` | `lidar/` | Velodyne / Hesai；其它厂商 stub |
| `lidar` | 由 `dimension`/`type` 决定 | `serial` / `velodyne` | `lidar/` | 同上 |
| `point_cloud` | `PointCloudModule` | `realsense` / `orbbec` | `camera/` | 深度点云 |
| `imu` / `imu_devices` | `ImuModule` | `serial` | `imu/` | serial / can / realsense |
| `gps` / `gps_devices` | `GpsModule` | `serial` | `gps/` | serial / can |
| `camera` | `CameraModule` | `realsense` / `orbbec` / `smartereye` | `camera/` | 图像 |
| `radar` | `RadarModule` | `conti` | `radar/` | stub |
| `microphone` | `MicrophoneModule` | `respeaker` | `mic/` | stub |
| `range` | `RangeModule` | `serial` | `range/` | attach-only 骨架 |

合并键 `lidar`：`dimension: 2d|3d`（或 `type`；`3d` / `lidar3d` / `pointcloud` 表示三维）。

## 通用字段

| 字段 | 必填 | 说明 |
|---|---|---|
| `name` | 是 | 短名；加载后 id 为 `<前缀><name>`。若已含 `/` 则原样作为 id |
| `enable` | 否 | `true` 时进入 Config 并 autostart；默认 `false`（不加载） |
| `channel` | 否 | Autolink 通道；**字符串或字符串数组**。省略则 `ResolveChannel` |
| `backend` | 否 | 硬件后端；省略用上表默认 |
| `params` | 否 | 额外驱动参数，合并进运行时 `params` |
| `match` | 否 | udev 匹配（见下文） |
| `module` / `library` | 否 | 高级：覆盖类名与外置 `.so`；typed 组通常省略 |

### `channel` 语义

- **单个字符串**：发布到该话题
- **字符串数组**：同一份 sample **fan-out** 到多个话题
- **不能**用数组表示相机 color/depth/ir 等多路不同流；每路 stream 写一条 `camera` 配置

### 默认 channel（`ResolveChannel`）

未写 `channel` 时：`"/" + id + 后缀`。

| 类型 | 默认 |
|---|---|
| IMU / GPS / Range | `/<id>` |
| Camera（`stream` 非 `depth`） | `/<id>/image_raw` |
| Camera（`stream: depth`） | `/<id>/depth/image_raw` |
| Lidar 2D | `/<id>/scan` |
| Lidar 3D / PointCloud | `/<id>/points` |
| Radar | `/<id>/radar` |
| Microphone | `/<id>/audio` |

示例 YAML 常显式写 ROS 风格名（如 `/camera/color/image_raw`、`/camera/depth/image_rect_raw`）。`aligned_depth_to_color`、`ir1` 等靠**显式 channel**，不是 `ResolveChannel` 的特殊后缀。

相机还会自动开 **camera_info** 通道（由图像话题推导，对齐 realsense-ros）。

### 串口 shorthand

`backend: serial`（或类型默认 serial）时，扁平字段写入 `params`：

| YAML | `params` | 说明 |
|---|---|---|
| `port` / `device` | `device` | 串口路径 |
| `baudrate` / `baud` | `baud` | 波特率 |

建议用 `/dev/serial/by-id/` 固定 `port`。

### 嵌套写法（可选）

仍支持 `hardware` / `publisher` 子块；`publisher.channel` 仅在顶层未写 `channel` 时回退。

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

## lidar_2d

**消息**：`sensor_msgs.LaserScan`  
**状态**：attach-only；串口驱动未实现。

| 字段 | 说明 |
|---|---|
| `port` / `baudrate` | 预留；常见波特率 115200 |
| `fps` | 写入 `publish_rate_hz` |

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

## lidar_3d

**消息**：`sensor_msgs.PointCloud2`  
**Module**：`Lidar3dModule`  
**backend**：`velodyne` / `udp`（默认 `velodyne`），或 `hesai` / `pandar`（PandarXT / XT32）— UDP 收包 → PacketQueue → 方位角切帧 → Convert 点云。  
其它厂商名（`livox`、`rslidar`、`lslidar`、`seyond`、`vanjee`）已注册为 **stub**（`Create`→nullptr）。

| 字段 | 说明 |
|---|---|
| `params.data_port` | UDP 端口，默认 `2368` |
| `params.packets_per_scan` | 一帧包数**上限**；Velodyne 默认 `75`，Hesai 默认 `180` |
| `params.use_azimuth_cut` | 默认 `true`；按末 block 方位角跨 cut 切帧 |
| `params.scan_cut_angle_deg` | cut 角度（度），默认 `0` |
| `params.packet_queue_capacity` | online 队列容量，默认 `256`（满丢最旧） |
| `params.model` | Velodyne：`VLP-16`；Hesai：`XT32` / `PandarXT` |
| `params.frame_id` | 点云 frame |
| `params.source_type` | `online`（默认）或 `raw_packet`（回放：`PushRawPacket` / `PushScan`） |
| `params.bind_host` | 绑定地址，默认任意 |
| `params.reconnect_attempts` | UDP 断线重连次数 |
| `params.calibration_path` | Velodyne / Hesai beam YAML（`vert_correction`） |
| `params.enable_compensator` | 运动补偿；经 `compensator.pose_channel` / `PoseFeeder` 灌姿 |
| `params.pose_channel` | 覆盖进程级 `compensator.pose_channel` |
| `params.extrinsic_path` | 有则 `world_T_lidar = odom × base_T_lidar` |
| `params.publish_scan` | 先发 `LidarPacketScan` 再发点云（Publisher 仅发点云） |
| `params.scan_channel` | 原始 Scan 通道提示（当前不经 Autolink Writer） |
| `params.world_frame_id` | 补偿世界系，默认 `world` |

点云字段：`x,y,z,intensity`（float32）+ `timestamp`（float64 ns）。静态外参可用 `LoadExtrinsicYaml`。Hesai 默认 XT32 仰角；可用 `calibration_path`（见 `config/params/XT32_calibration.yaml`，`vert_correction` 为度）。

```yaml
lidar_3d:
  - name: vlp16
    enable: false
    channel: /lidar/vlp16/points
    backend: velodyne
    params:
      data_port: 2368
      packets_per_scan: 75
      model: VLP-16
      source_type: online
      use_azimuth_cut: true
      scan_cut_angle_deg: 0
      enable_compensator: false
  - name: xt32
    enable: false
    channel: /lidar/xt32/points
    backend: hesai
    params:
      data_port: 2368
      packets_per_scan: 180
      model: XT32
      source_type: online
      use_azimuth_cut: true
```

---

## point_cloud

**消息**：`sensor_msgs.PointCloud2`（运行时类型 `kLidar3d`）  
**Module**：`PointCloudModule`；**id 前缀**：`camera/`  
**backend**：`realsense`（默认）或 `orbbec`（需 OrbbecSDK）

| 字段 | 说明 |
|---|---|
| `width` / `height` / `fps` | 传入厂商 pipeline |
| `params.model` / `params.serial` / `params.index` | 设备选择 |
| `params.frame_id` | 点云 frame |

```yaml
point_cloud:
  - name: realsense_d455_points
    enable: true
    channel: /camera/depth/color/points
    backend: realsense
    width: 848
    height: 480
    fps: 30
    params:
      model: D455
      frame_id: camera_depth_optical_frame
```

---

## imu

**消息**：`sensor_msgs.Imu`  
**backend**：`serial`（默认）、`can`、`realsense`

| 字段 | 说明 |
|---|---|
| `port` / `baudrate` | serial |
| `interface`、`accel_can_id`、`gyro_can_id`、`can_id` | can |
| `params.model` / `serial` / `index` | realsense 板载 IMU |
| `fps` | 写入 `publish_rate_hz`（串口当前按硬件速率输出） |

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
    enable: true
    channel: /camera/imu
    backend: realsense
    params:
      model: D455
      frame_id: camera_imu_optical_frame
```

---

## gps

**消息**：`sensor_msgs.NavSatFix`  
**backend**：`serial`（默认）、`can`

```yaml
gps:
  - name: gps_main
    enable: false
    channel: /gps/fix
    port: /dev/ttyUSB2
    baudrate: 9600
```

---

## camera

**消息**：`sensor_msgs.Image`（另开 camera_info）  
**backend**：`realsense`（默认）、`orbbec`（需 OrbbecSDK）、`smartereye`（**stub**，无 SDK 时 Create→nullptr）

| `stream` | 含义 | 示例 channel |
|---|---|---|
| `color` / `rgb`（默认） | RGB | `/camera/color/image_raw` |
| `depth` / `z16` | 深度 | `/camera/depth/image_rect_raw` |
| `ir1` / `infrared1` / `ir` | 红外 1（RealSense）/ Orbbec IR | `/camera/infra1/image_rect_raw` |
| `ir2` / `infrared2` | 红外 2（RealSense） | `/camera/infra2/image_rect_raw` |
| `aligned_depth_to_color` | 对齐到彩色的深度（RealSense） | `/camera/aligned_depth_to_color/image_raw` |

多路 stream 写多条 `camera`；同一物理机用相同 `params.serial` 或 `index`+`model` 共用厂商 hub。

设备级选项（可放在 `params`，同机多流共享）：

| 键 | 说明 |
|---|---|
| `emitter_enabled` / `enable_ir_emitter` | IR 投影灯 |
| `frame_id` | 图像 frame |
| `model` / `serial` / `index` | 设备过滤 |

```yaml
camera:
  - name: realsense_d455_color
    enable: true
    channel: /camera/color/image_raw
    backend: realsense
    stream: color
    width: 848
    height: 480
    fps: 30
    params:
      model: D455
      emitter_enabled: false
      frame_id: camera_color_optical_frame
```

---

## radar

**消息**：暂用 `PointCloud2`（`RadarSample` 占位）  
**Module**：`RadarModule`  
**backend**：`conti`（alias `continental`）  
**状态**：**stub** — 注册表存在，`Create` 返回 `nullptr`，直至 Conti ProtocolData + `canbus` 落地。

| 字段 | 说明 |
|---|---|
| `params.interface` | 预留 SocketCAN ifname（如 `can0` / `fake0`） |
| `channel` | 目标话题；默认 `/radar/<name>/radar` |

```yaml
radar:
  - name: front
    enable: false
    channel: /radar/front/objects
    backend: conti
    params:
      interface: can0
```

---

## microphone

**消息**：暂用 `sensor_msgs/Image` 装 PCM（`MicrophoneSample`）  
**Module**：`MicrophoneModule`  
**backend**：`respeaker`  
**状态**：**stub** — 需 PortAudio / USB HID 后再实现真采集。

| 字段 | 说明 |
|---|---|
| `params.device` / `sample_rate` | 预留 |
| `channel` | 默认 `/mic/<name>/audio` |

```yaml
microphone:
  - name: cabin
    enable: false
    channel: /mic/cabin/audio
    backend: respeaker
```

---

## range

**消息**：`sensor_msgs.Range`  
**状态**：attach-only。

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

`hotplug.enable_udev: true` 时：

| 字段 | 说明 |
|---|---|
| `match.subsystem` | 如 `tty`、`usb` |
| `match.device` | 设备节点 |
| `match.vendor` / `match.product` | USB 厂商/产品 ID |
| `match.serial` | 序列号 |

配置了 `port` 的 serial 会自动填 `match.subsystem=tty` 与 `match.device=<port>`（若未显式写 `match`）。空 `match` 不参与 udev。

详见 [生命周期](lifecycle.md)。

## 默认示例内容

`config/autodriver_hardware.yaml` 以 Intel RealSense D455 为主：多路 camera、`point_cloud`、板载 IMU 默认 `enable: true`；串口/激光等示例为 `enable: false`。通道命名对齐 realsense-ros。

## 代码加载

```cpp
#include "autodriver/config_loader.hpp"

autodriver::Config config = autodriver::LoadConfig();
config = autodriver::LoadConfig("/opt/autodriver",
                                autodriver::kDefaultConfigBasename);
```
