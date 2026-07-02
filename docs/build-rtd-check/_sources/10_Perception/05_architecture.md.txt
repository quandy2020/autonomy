# 5. Perception 模块架构设计

本文描述 `autonomy/perception` 的逻辑架构、核心组件关系与运行时数据流。

## 5.1 设计目标

Perception 模块遵循以下设计原则：

1. **插件化扩展**：各感知算法以 `PerceptionInterface` 插件形式注册，支持进程内与动态库加载
2. **配置驱动**：Lua → Protobuf 配置管线，与 planning/control 模块统一
3. **消息标准化**：输出对齐 `commsgs::vision_msgs`，便于与 ROS 生态互通
4. **推理基础设施复用**：视觉深度学习复用 `common/network` ONNX/TensorRT 管线
5. **与 map 解耦**：障碍几何表示可输出至 costmap 或独立障碍列表，供不同下游选择

## 5.2 实现状态

| 组件 | 实现度 | 说明 |
|------|--------|------|
| `PerceptionServer` 声明 | ⏳ | 头文件存在，`.cpp` 未实现 |
| `PerceptionInterface` | ⏳ | 仅 `LoadOptions()`，无插件 API |
| `PerceptionOptions` proto | ⏳ | 空 message |
| Lua 配置加载 | ❌ | 未接入 `system/options.cpp` |
| 系统启动集成 | ❌ | `Autonomy::Start()` 未构造 |
| `common/network` 推理 | ✅ | ONNX/TensorRT 完整 |
| `vision_msgs` 消息 | ✅ | 结构已定义 |
| `ObstacleLayer` 障碍感知 | ✅ | 在 map 模块中实现 |

## 5.3 分层架构

<div class="plan-arch-diagram">

  <div class="plan-arch-layer plan-arch-app">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">应用层</span>
      <span class="plan-arch-title">Prediction / Planning / Navigator</span>
      <span class="plan-arch-sub">消费感知输出的下游模块</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-body-block">
        <div class="nav-body-label">消费数据</div>
        <div class="nav-chip-list">
          <span class="nav-chip">vision_msgs</span>
          <span class="nav-chip">ObstacleArray</span>
          <span class="nav-chip">costmap</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>结构化环境表示</span></div>

  <div class="plan-arch-layer plan-arch-server">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">服务层</span>
      <span class="plan-arch-title">PerceptionServer</span>
      <span class="plan-arch-sub">模块唯一对外服务入口 · <code>perception_server</code> 节点</span>
    </div>
    <div class="plan-arch-body plan-arch-body-cols">
      <div class="nav-body-block">
        <div class="nav-body-label">核心职责</div>
        <ul>
          <li>插件加载、注册与按 <code>plugin_id</code> 调度</li>
          <li>@ <code>frequency</code> 循环处理传感器数据</li>
          <li>多传感器时间同步与 TF 变换</li>
          <li>发布 <code>vision_msgs</code> / 障碍列表</li>
        </ul>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">关键话题</div>
        <div class="nav-chip-list">
          <span class="nav-chip">/scan</span>
          <span class="nav-chip">/camera/image</span>
          <span class="nav-chip">/perception/detections</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>调度感知请求</span></div>

  <div class="plan-arch-split">
    <div class="plan-arch-layer plan-arch-plugin">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">算法层</span>
        <span class="plan-arch-title">PerceptionInterface 插件</span>
      </div>
      <div class="plan-arch-body">
        <div class="nav-body-block">
          <div class="nav-body-label">规划插件</div>
          <div class="nav-chip-list">
            <span class="nav-chip">YoloDetector</span>
            <span class="nav-chip">PointCloudCluster</span>
            <span class="nav-chip">GroundSegmentation</span>
            <span class="nav-chip">MultiObjectTracker</span>
          </div>
        </div>
        <div class="nav-body-block">
          <div class="nav-body-label">核心 API</div>
          <div class="nav-chip-list">
            <span class="nav-chip">Process()</span>
            <span class="nav-chip">GetOutput()</span>
          </div>
        </div>
      </div>
    </div>

    <div class="plan-arch-link">
      <span class="plan-arch-link-text">复用</span>
      <span class="plan-arch-link-arrow">↔</span>
    </div>

    <div class="plan-arch-layer plan-arch-map">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">基础设施层</span>
        <span class="plan-arch-title">common/network + vision_msgs</span>
      </div>
      <div class="plan-arch-body">
        <div class="nav-body-block">
          <div class="nav-body-label">能力</div>
          <div class="nav-chip-list">
            <span class="nav-chip">ONNX Runtime</span>
            <span class="nav-chip">TensorRT</span>
            <span class="nav-chip">NMS / YOLO Decode</span>
          </div>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>原始传感器数据</span></div>

  <div class="plan-arch-layer plan-arch-post">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">输入层</span>
      <span class="plan-arch-title">Driver / Sensor / Simulation</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-chip-list">
        <span class="nav-chip">LaserScan</span>
        <span class="nav-chip">PointCloud2</span>
        <span class="nav-chip">Image</span>
        <span class="nav-chip">Depth</span>
      </div>
    </div>
  </div>

</div>

## 5.4 运行时数据流

### 5.4.1 目标数据流（完整实现后）

```
Driver/Sim
  ├─ LaserScan ──────────────→ ObstacleLayer ──→ costmap
  ├─ PointCloud2 ──→ PerceptionServer
  │                    ├─ GroundSegmentation
  │                    ├─ EuclideanClustering
  │                    └─ ObstacleList ──→ Prediction
  └─ Image ────────→ PerceptionServer
                       ├─ YoloDetector (common/network)
                       ├─ MultiObjectTracker (Kalman)
                       └─ Detection2DArray ──→ Prediction / Visualization
```

### 5.4.2 当前数据流

```
激光/仿真 ──→ ObstacleLayer ──→ costmap ──→ Planning → Control
```

## 5.5 插件接口设计（规划）

参照 `ControllerInterface` 模式：

```cpp
class PerceptionInterface {
public:
    virtual void Configure(const PerceptionOptions& opts) = 0;
    virtual void Process(const SensorData& input) = 0;
    virtual Detection2DArray GetDetections() const = 0;
    virtual void Reset() = 0;
};
```

## 5.6 与 map 模块的关系

| 路径 | 说明 |
|------|------|
| **独立路径** | Perception 输出 `ObstacleArray`，由 Prediction 消费 |
| **融合路径** | Perception 结果写入 `ObstacleLayer`，与激光观测融合 |
| **当前路径** | 仅激光经 `ObstacleLayer` 写入 costmap，Perception 未参与 |

## 5.7 配置管线

```
config/perception/perception.lua
        │
        ▼
common::LoadOptions()  ──→  PerceptionOptions (proto)
        │
        ▼
PerceptionServer(options)
        │
        ▼
Autonomy::Start() 构造并启动
```

当前断点：`LoadOptions()` 返回空 proto；`autonomy.lua` 未引用 `AUTONOMY_PERCEPTION`。

## 5.8 扩展路线图

| 阶段 | 内容 |
|------|------|
| P0 | 完善 `PerceptionOptions` proto + Lua 加载 + `Autonomy` 接入 |
| P1 | 实现 `YoloDetector` 插件，接线 `common/network` |
| P2 | 点云地面分割 + 欧氏聚类插件 |
| P3 | 多目标跟踪（Kalman + 匈牙利关联） |
| P4 | 与 `prediction` 模块数据流对接 |
