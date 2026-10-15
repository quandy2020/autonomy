# 01 · 概述

AutoSim 是 Autonomy 的**仿真桥**：统一差速动力学、传感采样与 Autolink 编解码，后端可替换。

| 相关 | 链接 |
|------|------|
| 架构 | [02](02_architecture.md) |
| 后端对照 | [04](04_backends.md) |

---

## 1. 目标

- 为导航 / 感知 / 定位提供可复现的仿真数据流。
- 同一套话题与 TF 约定对接 Autoviz 与实车栈。
- 用 YAML 切换后端与场景，无需改业务代码。

---

## 2. 能力

| 能力 | 说明 |
|------|------|
| 差速 + 里程计 | `Robot` 积分；可选噪声 |
| 2D/3D 激光、相机、IMU | 按配置频率发布 |
| TF / 时钟 | `map→odom→base_link`（nav）或仅 `odom→base_link`（slam） |
| 真值地图 | OccupancyGrid + 可选点云 |
| 键盘遥控 | `teleop` → `/cmd_vel` |
| Scenario 动态体 | 共享 Traffic：障碍与行人（Fake / Habitat / Isaac） |
| RL 运控（可选） | `PolicyDriver` ONNX Twist；或栈内 `policy_controller` |
| IL 数据采集 | `dataset/` → LeRobot v2.1（parquet + video + meta） |

---

## 3. 非目标

| 不做 | 原因 |
|------|------|
| SLAM / 定位算法 | 由 Cartographer、Atlas 等承担 |
| Map / Nav / Loc RPC | 业务栈服务，不在桥接层 |
| 完整数字孪生编辑器 | 场景来自 Habitat GLB / Isaac USD / scenario 生成 |

---

## 4. 后端选型（简）

| 场景 | 推荐 |
|------|------|
| CI / 无 GPU 烟测 | `fake` |
| MP3D / HM3D 数据集 | `habitat` |
| 酷家乐 / Omniverse USD、PhysX、逼真 RGB-D | `isaac` |

细节见 [04 后端](04_backends.md)。
