(localization-usage)=
# 4. 使用指南

本章按 **Atlas 视觉 SLAM** 与 **AMCL 配置（待集成）** 分节说明。

---

## 4.1 Atlas 使用

> 完整文档：[atlas/guide.md](atlas/guide.md)

### 4.1.1 配置入口

| 项 | 值 |
|----|-----|
| 配置文件 | YAML（`atlas/example/` 各数据集模板） |
| 加载 API | `atlas::config(config_file_path)` |
| 词袋文件 | ORB BoW vocabulary（`.fbow`，回环需要） |
| 系统入口 | `atlas::system` |
| 节点模式 | `localization --localization_mode=atlas` |

### 4.1.2 集成方式

| 路径 | 适用 | 说明 |
|------|------|------|
| **A. C++ API** | 数据集评测 / 自定义应用 | `feed_*_frame()` 完整可用 |
| **B. localization 节点** | 地图加载/保存 | 启动线程；**相机话题 feed 待 bridge** |
| **C. 纯定位** | 已有地图 | `--atlas_map_load` + `disable_mapping_module()` |

### 4.1.3 命令行示例

```bash
localization --localization_mode=atlas \
  --atlas_config=autonomy/localization/atlas/example/euroc/EuRoC_stereo.yaml \
  --atlas_vocab=/path/to/orb_vocab.fbow \
  --atlas_map_save=data/atlas_map.msgpack
```

### 4.1.4 YAML 配置块

`Camera` / `Preprocessing` / `Feature` / `Mapping` / `Tracking` / `KeyframeInserter` / `Relocalizer` / `LoopDetector` / `Initializer` / `System` — 详见 [Atlas §5](atlas/guide.md#5-yaml-配置结构)。

### 4.1.5 API 与调用时机

| API | 何时 |
|-----|------|
| `startup()` | 配置与词袋加载后 |
| `feed_*_frame()` | 每帧图像到达（C++ API） |
| `load_map_database()` | 纯定位，`startup(false)` 后 |
| `save_map_database()` | 退出前持久化 |
| `enable_loop_detector()` | 大场景需回环 |
| `shutdown()` | 停止所有线程 |

---

## 4.2 AMCL 配置（待集成）

### 4.2.1 配置入口

| 项 | 值 |
|----|-----|
| 配置文件 | `config/localization/localization.lua` |
| 子配置 | `config/localization/amcl/amcl.lua` |
| Proto | `autonomy.localization.proto.AmclOptions`（规划中） |

### 4.2.2 启用方式（规划）

```lua
-- config/autonomy.lua
include "localization/localization.lua"

AUTONOMY = {
    localization = AUTONOMY_LOCALIZATION,
}
```

### 4.2.3 依赖话题

| 话题 | 类型 | 方向 |
|------|------|------|
| `/map` | OccupancyGrid | 订阅 |
| `/scan` | LaserScan | 订阅 |
| `/odom` | Odometry | 订阅（可选，用于运动模型） |
| TF `map→odom` | Transform | 发布 |

---

## 4.3 Cartographer 使用

> 完整文档：[cartographer/guide.md](cartographer/guide.md)

### 4.3.1 配置入口

| 项 | 值 |
|----|-----|
| 配置目录 | `config/localization/cartographer/` |
| 默认配置 | `backpack_2d.lua` |
| 二进制 | `localization`（`--localization_mode=cartographer`） |
| Launch | `autonomy/localization/launch/cartographer_2d.launch` |

### 4.3.2 三种运行模式

| 模式 | 配置 | Launch |
|------|------|--------|
| 2D SLAM（内嵌 /map） | `backpack_2d.lua` | `cartographer_2d.launch` |
| 2D SLAM + 独立 grid | `backpack_2d_with_grid.lua` | `localization_server.launch` |
| 纯定位 | `backpack_2d_localization.lua` + `.pbstream` | `cartographer_2d_localization.launch` |

### 4.3.3 数据准备（Backpack bag）

```bash
python -m autonomy.tools.bag_convert INPUT.bag -o ./data/records --backpack-2d
```

### 4.3.4 关键话题

| 方向 | 话题 | 说明 |
|------|------|------|
| 订阅 | `echoes_1` | MultiEchoLaserScan（backpack 默认） |
| 订阅 | `imu` | `use_imu_data=true` 时必需 |
| 订阅 | `tf` / `tf_static` | 传感器外参 |
| 发布 | `map` | OccupancyGrid |
| 发布 | `tracked_pose` | 当前位姿 |
| 发布 | TF | `map→odom` |

### 4.3.5 地图导出

| 方式 | 命令/配置 |
|------|-----------|
| 运行时 PGM | `save_map_image=true` → `data/map.pgm` |
| pbstream | `--save_state_filename=map.pbstream` |
| 离线 YAML+PGM | `cartographer_pbstream_to_map -pbstream_filename=...` |

---

## 4.4 与导航栈集成

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   Driver    │────►│ Localization │────►│  Transform  │
│ camera/scan │     │ Atlas / AMCL │     │  map↔base   │
└─────────────┘     └──────┬───────┘     └──────┬──────┘
                           │                     │
                           ▼                     ▼
                    ┌─────────────┐       ┌─────────────┐
                    │ Map (可选)   │       │  Planning   │
                    └─────────────┘       └─────────────┘
```

| 集成点 | Atlas | AMCL |
|--------|-------|------|
| TF 输出 | 需桥接 `T_cw` → `map→odom` | 原生支持 |
| 静态地图 | 可导出稀疏地图 | 消费 `/map` |
| 传感器 | 相机 | 激光 + 里程计 |

---

## 4.5 算法选型

| 场景 | 推荐 | 理由 |
|------|------|------|
| 激光 2D 建图/定位 | **Cartographer** | 默认后端，backpack 配置完备 |
| 视觉机器人 / 无激光 | Atlas | 单目/双目/RGB-D |
| 已知 2D 地图 + 激光 | AMCL | 成熟、低算力（待集成） |
| 大场景视觉 + 回环 | Atlas + loop | BoW 回环 + GBA |
| 短走廊 / 纹理弱 | 双目/RGB-D Atlas 或 Cartographer | 深度/激光更鲁棒 |

---

## 4.6 性能参考

| 模式 | 典型帧率 | 主要瓶颈 |
|------|----------|----------|
| 单目 640×480 | 20–30 Hz | ORB 提取 + 匹配 |
| 双目 752×480 | 15–25 Hz | 立体匹配 + LBA |
| RGB-D 640×480 | 20–30 Hz | 深度处理 + LBA |
| Loop BA | 异步 | g2o GBA，可能数秒 |

---

## 4.7 故障排查

| 现象 | 可能原因 | 排查 |
|------|----------|------|
| Cartographer 无地图 | 话题名错误 | backpack 需 `echoes_1`，见 [Cartographer §11](cartographer/guide.md#11-故障排查) |
| 长期 Initializing（Atlas） | 纹理不足 / 运动不够 | 增加纹理、做充分平移 |
| 频繁 Lost | 快速运动 / 曝光变化 | 降低速度、提高 `min_fast_threshold` |
| 尺度漂移（单目） | 无回环 | `enable_loop_detector()` |
| 双目不三角化 | 未校正 / baseline 错 | 检查 `focal_x_baseline` |
| RGB-D 深度异常 | `depthmap_factor_` 错误 | 对齐 TUM/Kinect 标准 |
| 地图加载失败 | 格式不匹配 | 检查 `map_format` |
| AMCL 不工作 | 尚未集成 | 使用 Atlas 或等待集成 |

---

## 4.8 调试工具

| 工具 | 用途 |
|------|------|
| `frame_publisher` | 当前帧特征、跟踪状态 |
| `map_publisher` | 稀疏 3D 地图与关键帧 |
| `save_frame_trajectory` | 逐帧轨迹导出 |
| `print_info()` | 打印系统与相机参数 |

```cpp
auto frame_pub = slam.get_frame_publisher();
auto map_pub = slam.get_map_publisher();
slam.print_info();
```
