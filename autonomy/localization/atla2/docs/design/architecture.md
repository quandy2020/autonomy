# 架构设计

## 1. 总体架构

Atla2 采用 **配置驱动的流水线架构**：一次 `Step` 完成「同步包 → 前端里程计 → 健康评估 → 后端融合 → 地图更新 → 状态机推进」。

```text
PushImu / PushImage / PushLidar
        │
        ▼
   SensorSync (软同步窗口)
        │
        ▼
   FrontendBase::Process → OdometryResult
        │
        ├─► DegradationManager::Evaluate
        │
        ▼
   OptimizerBase::UpdateWithSensors
        │
        ▼
   MapManager::UpdateFromOdometry
        │
        ▼
   StateMachine (Initializing / Tracking / Degraded / Lost)
```

## 2. 关键组件

### 2.1 SlamSystem

编排器，持有：

- `SensorSync` — 时间对齐缓冲
- `FrontendBase` — 工厂创建
- `OptimizerBase` — 工厂创建
- `MapManager` — 地图
- `DegradationManager` — 健康与降级建议
- `StateMachine` — 对外状态
- `TransformTree` — 外参

对外 API：`Init` / `InitFromFile` / `Push*` / `Step` / `GetOdometry` / `Reset`。

### 2.2 状态机

| 状态 | 含义 | 进入条件 |
|------|------|----------|
| `Uninitialized` | 未 Init | 默认 / Reset |
| `Initializing` | 初始化中 | `OnInitStart` |
| `Tracking` | 正常跟踪 | Init 成功 / 跟踪 OK |
| `Degraded` | 传感器缺失或协方差膨胀 | 降级评估 |
| `Lost` | 前端失败 | `Process` / `GetResult` 失败 |

丢失后由上层决定重定位或重启前端（当前骨架：标记 Lost，可再次 Init/Reset）。

### 2.3 配置驱动

`Atla2Config` 由平台 YAML 填充：

- `mode` → 前端
- `fusion` → loose / tight（LIVO）
- `backend` → iekf / ceres|graph
- 噪声、窗口、体素、相机内参等

同一二进制换 YAML 即切换机型与模式。

## 3. 并发模型（目标）

当前实现以同步 `Step` 为主；目标线程划分（与 framework 一致）：

| 线程 | 职责 | 优先级建议 |
|------|------|------------|
| 传感器 / 同步 | 入队、对齐 | 高 |
| 前端跟踪 | `Process` | 最高（实时） |
| 后端优化 | 滑动窗口 / IEKF | 中高 |
| 回环 | 检测与优化 | 低，后台 |
| 日志 / IO | 异步写盘 | 最低 |

嵌入式：前端绑大核，回环绑小核；热路径避免动态分配（见部署文档）。

## 4. 错误与降级

```text
LIVO 全传感器 OK ──► kFull
      │ lidar 丢失
      ▼
   LIO only ──► 切 LIO 前端（或忽略视觉因子）
      │ lidar+image 丢失
      ▼
   VIO only / IMU coast
```

`DegradationManager` 输出 `HealthReport`；`SlamSystem` 据此置 `Degraded`。完整「运行时换前端」为后续能力。

## 5. 可观测性

| 信号 | 来源 | 用途 |
|------|------|------|
| `SlamState` | state machine | 飞行控制逻辑 |
| `OdometryResult.cov` | 前端/后端 | 健康门控 |
| 残差 / 迭代次数 | graph backend | 调试 |
| FPS / RSS / 温升 | `tools/benchmark` | 评测 |

## 6. 安全边界

- 核心库 **不链接 ROS**；Autolink / ROS 仅在 interface 层（规划）。
- 传感器数据边界检查在 `sensor/*` 与 sync 完成。
- 热路径优先错误码 / `bool`，避免异常穿透。

## 7. 相关文档

- [structure.md](structure.md)
- [data_flow.md](data_flow.md)
- [sensor_fusion.md](sensor_fusion.md)
- [../deployment/verification.md](../deployment/verification.md)
