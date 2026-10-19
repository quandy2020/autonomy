# Hesai（3D）

backend：`hesai`（alias `pandar`）。工厂 `CreateHesaiLidarDriver` 按型号 / 参数分发两条路径：

| 路径 | 何时选用 | 依赖 |
|---|---|---|
| **UDP（自研）** | `model` 为 XT32 / PandarXT 族，且未设 `use_sdk` | 无第三方 SDK |
| **SDK** | `use_sdk: true` / `parser: sdk` / 非 XT32（如 **JT128**、OT128） | [HesaiLidar_SDK_2.0](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0) |

点云统一：`sensor_msgs/PointCloud2`，`point_step=24`（x,y,z,intensity,timestamp）。参数名对齐 [HesaiLidar_ROS_2.0](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0) 的扁平键（无需嵌套 `lidar_udp_type`）。

| | |
|---|---|
| 源码 | `autodriver/lidar/hesai/` |
| XT32 params | `config/lidar/hesai/xt32.yaml` |
| JT128 params | `config/lidar/hesai/jt128.yaml` |
| SDK 模板 | `config/lidar/hesai/sdk.yaml` |
| 验证配置 | `config/examples/jt128_verify.yaml` |

---

## 1. 安装 HesaiLidar_SDK（JT128 / 多机型必选）

```bash
cd $AUTODRIVER_PATH   # autodriver 包根
./scripts/install_hesai_lidar_sdk.sh
# 或指定已克隆源码：
# ./scripts/install_hesai_lidar_sdk.sh /path/to/HesaiLidar_SDK_2.0
```

安装到 `PREFIX/opt/hesai_lidar_sdk`（默认 `PREFIX=/usr/local`），并生成
`lib/cmake/HesaiLidarSDK/HesaiLidarSDKConfig.cmake`。

重编 autodriver：

```bash
cmake -S … -B … -DAUTODRIVER_WITH_HESAI_SDK=ON
cmake --build … --target autodriver autodriver_main -j"$(nproc)"
```

未开 `AUTODRIVER_WITH_HESAI_SDK` 时，XT32 UDP 仍可用；请求 SDK 路径会创建失败并打日志。

---

## 2. JT128 使用方式（已验证）

### 2.1 网络

| 角色 | 默认 IP |
|---|---|
| 主机有线网口 | `192.168.1.100/24`（自行配置） |
| JT128 出厂 | `192.168.1.201` |

```bash
# 例：宿主机 / Docker --net=host
ip link set <eth> up
ip addr add 192.168.1.100/24 dev <eth>   # 若尚未配置
ip link show <eth>    # 需 LOWER_UP
ping -c2 192.168.1.201
```

出厂点云常发往 **`255.255.255.255:2368`**。此时务必让 `host_ip_address` 为空，
使 SDK 绑定 `0.0.0.0:2368`；绑定单播 `192.168.1.100` 会收不到广播包。

### 2.2 角度校正（必配，否则点云扁平）

无 elevation 表时各通道竖直角≈0，autoviz 里呈扁平同心圆。任选：

1. **`use_ptc_connected: true`**：经 PTC（9347）从雷达拉取本机校正（推荐）
2. **`correction_file_path`**：文件回退（SDK 自带样例）

```text
/usr/local/opt/hesai_lidar_sdk/correction/angle_correction/JT128_Angle Correction File.csv
/usr/local/opt/hesai_lidar_sdk/correction/firetime_correction/JT128_Firetime Correction File.csv
```

### 2.3 最小验证配置

主配置可直接用示例（仅开 JT128）：

```bash
export AUTODRIVER_PATH=/path/to/autodriver
export LD_LIBRARY_PATH=/path/to/build/autonomy/lib:$LD_LIBRARY_PATH

$BUILD/bin/autodriver \
  --config-dir "$AUTODRIVER_PATH" \
  --config-file examples/jt128_verify.yaml
```

或写入 `autodriver_hardware.yaml`：

```yaml
lidar_3d:
  - name: jt128
    enable: true
    channel: /lidar/jt128/points
    backend: hesai
    params_file: lidar/hesai/jt128.yaml
    # params:   # 可选覆盖
    #   host_ip_address: ""
    #   use_ptc_connected: true
```

`jt128.yaml` 关键：`use_sdk: true`、`model: JT128`、`hesai_source_type: 1`（在线 UDP）、
较大的 `socket_buffer_size`（约 1.6M pts/s）。

### 2.4 可视化（autoviz）

| 项 | 值 |
|---|---|
| Topic | `/lidar/jt128/points` |
| Fixed Frame | **`hesai_jt128`**（与 `frame_id` 一致） |

无 TF 时不要选 `odom`/`base_link`，否则点云不显示。

### 2.5 常见问题

| 现象 | 处理 |
|---|---|
| `NO-CARRIER` / ping 不通 | 电源灯、网线、主机 RJ45；与 Docker 无关（`--net=host`） |
| 驱动在跑、autoviz 无点 | 广播 vs 单播绑定：清空 `host_ip_address` |
| 点云扁平 | 开 PTC 或配置 `correction_file_path` / `firetimes_path` |
| `Create… nullptr` / Init 失败 | 确认 `-DAUTODRIVER_WITH_HESAI_SDK=ON` 且 SDK 已安装 |
| SHM `msg_size` 过大 | 正常（JT128 帧大）；autolink 会重建 buffer |

串口 `rs485_com` / `rs232_com` 仅 **JT16**（`hesai_source_type: 4`），JT128 用以太网。

---

## 3. PandarXT / XT32（自研 UDP）

不依赖官方 SDK。管线：UDP → 队列 → 方位角切帧 → Convert。包长 1080B，距离单位 4mm。

```yaml
lidar_3d:
  - name: xt32
    enable: true
    channel: /lidar/xt32/points
    backend: hesai
    params_file: lidar/hesai/xt32.yaml
    # params:
    #   calibration_path: …/xt32_calibration.yaml   # 仰角为度
    #   udp_port: 2368   # 可作 data_port 别名
```

未覆盖 XT32M2X 等其它包格式。勿与 Velodyne 校准混用（Hesai 仰角为**度**，Velodyne 为 rad）。

---

## 4. SDK 路径关键参数

| 键 | 说明 |
|---|---|
| `use_sdk` / `parser: sdk` | 强制 SDK；非 XT32 型号也会自动走 SDK |
| `model` | 如 `JT128`、`OT128` |
| `hesai_source_type` | `1` 在线 UDP，`2` pcap，`4` 串口(JT16)，`5` TCP |
| `device_ip_address` | 雷达 IP |
| `udp_port` | 点云目的端口，默认 2368 |
| `host_ip_address` | 绑定/主机 IP；广播场景请留空 |
| `ptc_port` / `use_ptc_connected` | PTC 控制与拉校正 |
| `correction_file_path` / `firetimes_path` | 角度 / 发火时刻文件 |
| `socket_buffer_size` | 收包缓冲（JT128 建议 ≥ 262144000） |
| `frame_id` | 点云 frame（JT128 默认 `hesai_jt128`） |

兼容 ROS2 配置思路，详见 `config/lidar/hesai/sdk.yaml`、`jt128.yaml`。
