# 配置

YAML（`yaml-cpp`）→ 运行时 `autodriver::Config`。默认文件：`config/autodriver_hardware.yaml`。

| 相关 | 链接 |
|---|---|
| 加载路径 / 进程 | [快速开始](quickstart.md) · [使用方式](usage.md) |
| enable / Attach | [生命周期](lifecycle.md) |
| backend / Registry | [架构](architecture.md) · [后端](backends.md) |

**结构约定**

- 设备列表推荐写在根下 `sensors:` 映射（`imu:`、`camera:` …）；也可把各类型键直接写在根上。  
- 仅 **`enable: true`**（或旧别名 `attach_on_start: true`）的条目进入 `Config.sensors`。  
- 本体写在根级 **`chassis:`**（不在 `sensors` 内）。

## 进程级字段

写在 YAML 根节点（均可省略）：

| YAML 键 | 运行时 | 默认 | 说明 |
|---|---|---|---|
| `node_name` | `node_name` | `autodriver` | Autolink 节点名 |
| `plugin_dir` | `plugins` | 空 | 外置 `.so`；空则用 `AUTODRIVER_PLUGIN_DIR` / 编译宏 |
| `hotplug.enable_udev` | `hotplug.udev` | `true` | 是否启动 udev 线程 |
| `alignment.enable` | `alignment.enable` | `false` | 样本 tap 进 SensorHub（见 [数据流](dataflow.md)） |
| `alignment.publish_raw` | `alignment.publish_raw` | `true` | enable 时是否仍把原始样本推 Sink |
| `alignment.publish_aligned` | `alignment.publish_aligned` | `false` | enable 时是否把对齐快照内样本推 Sink |
| `alignment.alignment_window_ms` | `options.alignment_window` | `50` | 对齐窗口 (ms) |
| `alignment.publish_period_ms` | `options.publish_period` | `20` | 对齐发布周期 (ms) |
| `alignment.buffer_capacity` | `options.buffer_capacity` | `32` | 每路样本缓冲容量 |
| `compensator.pose_channel` | `compensator.pose_channel` | 空 | `nav_msgs/Odometry`；空则 PoseFeeder no-op |

```yaml
node_name: autodriver
hotplug:
  enable_udev: false
alignment:
  enable: false
  publish_raw: true          # enable 时是否仍发原始流
  publish_aligned: false     # enable 时是否把对齐快照推 Sink
  alignment_window_ms: 50
  publish_period_ms: 20
  buffer_capacity: 32
compensator:
  pose_channel: /localization/odom   # 可选；需 lidar enable_compensator
```

## 路径解析

`LoadConfig(configuration_directory, config_basename)`：

1. `config_basename` 以 `/` 开头 → 绝对路径  
2. `{WorkRoot}/config/{basename}`  
3. 回退 `{AUTODRIVER_DISTRIBUTION_HOME}/share/autodriver/config/{basename}`

空文件名默认 `autodriver_hardware.yaml`。`WorkRoot` 优先 `AUTODRIVER_PATH`（含 `config/` 的包根，如 `$PWD/autodriver`）。

### `params_file` 合并

| 顺序 | 规则 |
|---|---|
| 1 | 加载 `config/{params_file}` 进 `params` |
| 2 | 条目内扁平 shorthand（`port`/`baudrate`/…）写入 `params` |
| 3 | 条目内 `params:` **覆盖**同名键 |
| 4 | 折叠子项（stream 等）可再覆盖 `frame_id` / `channel` 等 |

厂商细项放 `config/camera|<lidar>/<vendor>/*.yaml`，主列表只写 `params_file`。

```text
config/
  autodriver_hardware.yaml
  examples/orbbec_gemini_330.yaml
  camera/realsense/d455.yaml
  camera/orbbec/gemini_330.yaml
  lidar/velodyne/vlp16.yaml
  lidar/slamtec/a1.yaml
```

## chassis（本体）

根级块；**不依赖** `autonomy/vehicle`。由 `ChassisManager` 消费。

| YAML 键 | 默认 | 说明 |
|---|---|---|
| `enable` | `false` | false 时 Manager Start 为 no-op |
| `name` / `id` | `name`→`chassis/<name>`；或直接 `id` | 实例 id |
| `backend` | `stub` | `ChassisBackendRegistry`（`stub` / 厂商） |
| `cmd_vel_channel` | `/cmd_vel` | TwistStamped |
| `state_channel` | `/robot_state` | RobotState |
| `event_channel` | `/robot_event` | RobotEvent；可空字符串关闭 |
| `odom_channel` | `/odom` | Odometry；空则不发 |
| `watchdog_ms` | `200` | 无新 cmd 则零速；`0` 关闭 |
| `max_linear_speed` / `max_angular_speed` | `0` | `0` = 不限速 |
| `odom_period_ms` | `20` | 状态 / odom 发布周期 |
| `odom_frame_id` / `base_frame_id` | `odom` / `base_link` | Odometry 坐标系 |
| `params` / `params_file` | — | 厂商参数 |

```yaml
chassis:
  enable: false
  name: base
  backend: stub
  cmd_vel_channel: /cmd_vel
  odom_channel: /odom
  watchdog_ms: 200
  odom_period_ms: 20
  max_linear_speed: 1.0
  max_angular_speed: 1.5
  odom_frame_id: odom
  base_frame_id: base_link
```

## 传感器类型键

| YAML 键 | Module | Registry | YAML 默认 backend | id 前缀 | 状态 |
|---|---|---|---|---|---|
| `lidar_2d` | `Lidar2dModule` | `Lidar2dBackendRegistry` | `rplidar` | `lidar/` | RPLidar |
| `lidar_3d` | `Lidar3dModule` | `LidarBackendRegistry` | `velodyne` | `lidar/` | Velodyne/Hesai/Livox；其它 stub |
| `lidar` | 由 `dimension`/`type` | 同上 | `serial` / `velodyne` | `lidar/` | 合并键 |
| `point_cloud` | `PointCloudModule` | `PointCloudBackendRegistry` | `realsense` | `camera/` | 深度点云 |
| `imu` / `imu_devices` | `ImuModule` | `ImuBackendRegistry` | `serial` | `imu/` | serial/can/realsense |
| `gps` / `gps_devices` | `GpsModule` | `GpsBackendRegistry` | `serial` | `gps/` | serial/can |
| `camera` | `CameraModule` | `CameraBackendRegistry` | `realsense` | `camera/` | 图像 |
| `radar` | `RadarModule` | `RadarBackendRegistry` | `conti` | `radar/` | stub |
| `microphone` | `MicrophoneModule` | `MicrophoneBackendRegistry` | `respeaker` | `mic/` | stub |
| `range` | `RangeModule` | — | `serial` | `range/` | attach-only |

合并键 `lidar`：`dimension: 2d|3d`（或 `type`；`3d` / `lidar3d` / `pointcloud` → 三维）。

「YAML 默认」= 条目未写 `backend` 时 loader 填入的值；各 Registry 对 Create 空串再填同一默认（见 [架构](architecture.md)）。

## 通用字段

| 字段 | 必填 | 说明 |
|---|---|---|
| `name` | 是 | 短名 → id `<前缀><name>`；已含 `/` 则原样作 id |
| `enable` | 否 | `true` 进入 Config 并 `autostart`；默认不加载 |
| `channel` | 否 | 字符串或**字符串数组**；省略则 `ResolveChannel` |
| `backend` | 否 | 硬件后端；省略用上表 YAML 默认 |
| `params` | 否 | 驱动键值；覆盖 `params_file` |
| `params_file` | 否 | 相对 `config/` |
| `match` | 否 | udev（见文末） |
| `module` / `library` | 否 | 覆盖类名 / 外置 `.so`；typed 组通常省略 |

### `channel` 语义

- **单个字符串**：发到该话题  
- **字符串数组**：同一份 sample **fan-out** 到多个话题  
- **不能**用数组表示相机 color/depth/ir 多路不同流；每路 stream 一条配置（或折叠 `streams`）

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

示例 YAML 常显式写 ROS 风格名。`aligned_depth_to_color`、`ir1` 等靠**显式 channel**。

**camera_info**：`Publisher` 对每个 image channel 再开一路；帧级仅当样本 `has_camera_info` 时写入。名由 `bridge::CameraInfoChannelForImage`（`bridge/channels.hpp`）推导。

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
**Registry**：`Lidar2dBackendRegistry`  
**backend**：`rplidar`（别名 `slamtec`）；需 rplidar_sdk + `AUTODRIVER_WITH_RPLIDAR`  
厂商参数：`config/lidar/slamtec/{a1,a2,a3}.yaml`。可选 `scripts/create_udev_rules.sh` → `/dev/rplidar`。

| 字段 | 说明 |
|---|---|
| `port` / `baudrate` | 串口；A1/A2M8=`115200`，A3/A2M7/A2M12=`256000` |
| `params_file` | 如 `lidar/slamtec/a1.yaml` |
| `params.frame_id` | 默认 `laser` |
| `params.angle_compensate` | 角度补偿（默认 true） |
| `params.scan_mode` | 空=typical；A3 常用 `Sensitivity` |
| `params.channel_type` | `serial`（默认）/ `tcp` / `udp` |

```yaml
lidar_2d:
  - name: front
    enable: true
    channel: /lidar/front/scan
    backend: rplidar
    port: /dev/ttyUSB0
    baudrate: 115200
    params_file: lidar/slamtec/a1.yaml
    params:
      frame_id: laser
```

---

## lidar_3d

**消息**：`sensor_msgs.PointCloud2`  
**Registry**：`LidarBackendRegistry`  
**backend**：`velodyne`/`udp`（YAML 默认 `velodyne`）、`hesai`/`pandar`、`livox`；`rslidar`/`lslidar`/`seyond`/`vanjee` 为 stub（Create→nullptr）。

| 字段 | 说明 |
|---|---|
| `params.data_port` | UDP 端口，默认 `2368`（Velodyne/Hesai） |
| `params.packets_per_scan` | 一帧包数上限；Velodyne 默认 `75`，Hesai 默认 `180` |
| `params.use_azimuth_cut` | 默认 `true`；方位角切帧 |
| `params.scan_cut_angle_deg` | cut 角度（度），默认 `0` |
| `params.packet_queue_capacity` | online 队列，默认 `256`（满丢最旧） |
| `params.model` | Velodyne：`VLP-16`；Hesai：`XT32`；Livox：`Mid-360` … |
| `params.frame_id` | 点云 frame |
| `params.source_type` | `online`（默认）或 `raw_packet` |
| `params.bind_host` | 绑定地址，默认任意 |
| `params.reconnect_attempts` | UDP 断线重连次数 |
| `params.calibration_path` | Velodyne / Hesai beam YAML |
| `params.enable_compensator` | 运动补偿；经 `compensator.pose_channel` / PoseFeeder |
| `params.pose_channel` | 覆盖进程级 compensator |
| `params.extrinsic_path` | 有则 `world_T_lidar = odom × base_T_lidar` |
| `params.publish_scan` | 先发 `LidarPacketScan` 再发点云 |
| `params.scan_channel` | 原始 Scan 提示（当前不经 Autolink Writer） |
| `params.world_frame_id` | 补偿世界系，默认 `world` |
| `params.host_ip` / `lidar_ip` | Livox SDK2；或 `config_path` JSON |
| `params.broadcast_code` | Livox SDK1 白名单（空=全部） |
| `params.publish_freq` | Livox 组帧频率 Hz |

点云：`point_step=24`（`x,y,z,intensity` + `timestamp` ns）。校准 / 外参为**文件系统路径**。Livox 安装：`scripts/install_livox_sdk*.sh`。

```yaml
lidar_3d:
  - name: vlp16
    enable: false
    channel: /lidar/vlp16/points
    backend: velodyne
    params_file: lidar/velodyne/vlp16.yaml
  - name: xt32
    enable: false
    channel: /lidar/xt32/points
    backend: hesai
    params_file: lidar/hesai/xt32.yaml
  - name: mid360
    enable: false
    channel: /lidar/mid360/points
    backend: livox
    params_file: lidar/livox/mid360.yaml
```

---

## point_cloud

**消息**：`sensor_msgs.PointCloud2`（运行时 `kLidar3d`）  
**Registry**：`PointCloudBackendRegistry`；id 前缀 `camera/`  
**backend**：`realsense`（默认）或 `orbbec`

| 字段 | 说明 |
|---|---|
| `width` / `height` / `fps` | 传入厂商 pipeline |
| `params.model` / `serial` / `index` | 设备选择 |
| `params.frame_id` | 点云 frame |

也可挂在折叠 `camera.point_clouds:` 下。

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
**Registry**：`ImuBackendRegistry`  
**backend**：`serial`（默认）、`can`、`realsense`（需 librealsense）

| 字段 | 说明 |
|---|---|
| `port` / `baudrate` | serial |
| `interface`、`accel_can_id`、`gyro_can_id`、`can_id` | can |
| `params.model` / `serial` / `index` | realsense 板载 IMU |
| `fps` | 写入 `publish_rate_hz`（串口仍跟硬件速率） |

板载也可写在折叠 `camera.imu:`。

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
**Registry**：`GpsBackendRegistry`  
**backend**：`serial`（NMEA，默认）、`can`  
句解析：`GnssParserRegistry`（`nmea` / `nmea0183`），与传输后端正交。

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

**消息**：`sensor_msgs.Image`（另开 camera_info；帧级需 `has_camera_info`）  
**Registry**：`CameraBackendRegistry`  
**backend**：`realsense`（默认）、`orbbec`、`smartereye`（stub）

| `stream` | 含义 | RealSense channel 例 | Orbbec Gemini 330 例 |
|---|---|---|---|
| `color` / `rgb`（默认） | RGB | `/camera/color/image_raw` | `/camera/color/image_raw` |
| `depth` / `z16` | 深度 | `/camera/depth/image_rect_raw` | `/camera/depth/image_raw` |
| `left_ir` / `ir` / `ir1` | 左红外 | `/camera/infra1/image_rect_raw` | `/camera/left_ir/image_raw` |
| `right_ir` / `ir2` | 右红外 | `/camera/infra2/image_rect_raw` | `/camera/right_ir/image_raw` |
| `aligned_depth_to_color` | 对齐深度（RS） | `/camera/aligned_depth_to_color/image_raw` | — |

Orbbec 点云：默认 `/camera/depth/points`；彩色云 `/camera/depth_registered/points`。  
厂商参数：`config/camera/orbbec/gemini_330.yaml`；折叠例：`config/examples/orbbec_gemini_330.yaml`。

### 折叠写法（推荐）

一台物理机一条 `camera`，用 `streams` / `point_clouds` / `imu`；loader 展开为多条 Sensor（id：`camera/<name>_<stream>`、`camera/<name>_points`、`imu/<name>_imu`）。子项可 `enable: false`。

```yaml
camera:
  - name: realsense_d455
    enable: true
    backend: realsense
    params_file: camera/realsense/d455.yaml
    width: 848
    height: 480
    fps: 30
    streams:
      - stream: color
        channel: /camera/color/image_raw
        frame_id: camera_color_optical_frame
      - stream: depth
        channel: /camera/depth/image_rect_raw
        frame_id: camera_depth_optical_frame
    point_clouds:
      - name: points
        channel: /camera/depth/color/points
        frame_id: camera_depth_optical_frame
    imu:
      channel: /camera/imu
      frame_id: camera_imu_optical_frame
```

### 扁平写法（兼容）

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
    params_file: camera/realsense/d455.yaml
    params:
      frame_id: camera_color_optical_frame
```

设备级选项（`params` / `params_file`，同机多流共享）：

| 键 | 说明 |
|---|---|
| `emitter_enabled` / `enable_ir_emitter` | IR 投影灯 |
| `frame_id` | 图像 frame（子项可覆盖） |
| `model` / `serial` / `index` | 设备过滤 |

---

## radar

**消息**：暂用 `PointCloud2`（占位）  
**Registry**：`RadarBackendRegistry`；`backend: conti`（别名 `continental`）  
**状态**：stub（Create→nullptr）

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

**消息**：暂用 `Image` 装 PCM  
**Registry**：`MicrophoneBackendRegistry`；`backend: respeaker`  
**状态**：stub

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
**状态**：attach-only（无 Registry / 无真采集）

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

`hotplug.enable_udev: true` 且编译进 udev 时：

| 字段 | 说明 |
|---|---|
| `match.subsystem` | 如 `tty`、`usb` |
| `match.device` | 设备节点 |
| `match.vendor` / `match.product` | USB ID（可 `0x`，大小写不敏感） |
| `match.serial` | 序列号 |

空 `match` 不参与 udev。serial 且未写 `match`、但有 `port`/`device` 时自动：`subsystem=tty`、`device=<port>`。详见 [生命周期](lifecycle.md)。

## 代码加载

```cpp
#include "autodriver/config_loader.hpp"

autodriver::Config config = autodriver::LoadConfig();
config = autodriver::LoadConfig("/path/to/autodriver",
                                autodriver::kDefaultConfigBasename);
```

```bash
export AUTODRIVER_PATH=/path/to/autodriver
autodriver
autodriver "$AUTODRIVER_PATH" autodriver_hardware.yaml
```
