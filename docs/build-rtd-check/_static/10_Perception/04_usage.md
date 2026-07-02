(perception-usage)=
# 4. 使用指南

### 4.1 配置

**入口文件**：`config/perception/perception.lua` → `config/autonomy.lua`

> **注意**：当前 `AUTONOMY_PERCEPTION` 为空表，且未挂入 `AUTONOMY` 根表；`system/options.cpp` 亦未加载 perception 配置。以下为**规划中的配置结构**。

```lua
-- config/autonomy.lua（规划）
AUTONOMY = {
    perception = AUTONOMY_PERCEPTION,
}
```

**规划字段**：

| 字段 | 说明 | 默认建议 |
|------|------|----------|
| `enabled` | 是否启动 PerceptionServer | `false`（当前） |
| `frequency` | 感知循环频率 (Hz) | `10.0` |
| `plugins` | 启用的感知插件列表 | 待实现 |
| `yolo.model_path` | ONNX 模型路径 | — |
| `yolo.confidence_threshold` | 检测置信度阈值 | `0.5` |
| `yolo.nms_threshold` | NMS IoU 阈值 | `0.45` |

### 4.2 PerceptionServer API（规划）

| API | 用途 | 状态 |
|-----|------|------|
| `Start()` / `Shutdown()` | 启停感知循环 | ❌ |
| `Process(sensor_data)` | 处理单帧传感器数据 | ❌ |
| `GetDetections()` | 获取最新检测结果 | ❌ |
| `GetObstacles()` | 获取障碍列表 | ❌ |

### 4.3 vision_msgs 消息

感知输出使用 `commsgs::vision_msgs` 命名空间：

| 消息 | 用途 |
|------|------|
| `BoundingBox2D` | 2D 旋转检测框（像素坐标） |
| `BoundingBox2DArray` | 带 Header 的 2D 框数组 |
| `BoundingBox3D` | 3D 检测框（米制） |
| `Detection2D` | 单目标：框 + 分类 + 置信度 |
| `Detection2DArray` | 2D 检测数组 |
| `Detection3D` / `Detection3DArray` | 3D 检测 |
| `Classification` | 分类结果 |

**BoundingBox2D 示例**：

```cpp
commsgs::vision_msgs::BoundingBox2D box;
box.center.position.x = 320.0;  // 像素
box.center.position.y = 240.0;
box.center.theta = 0.0;         // 弧度，逆时针为正
box.size_x = 100.0;
box.size_y = 80.0;
```

### 4.4 common/network 推理 API

| API | 说明 |
|-----|------|
| `BackendFactory::Create(opts)` | 创建 ONNX/TensorRT 引擎 |
| `Engine::Run(inputs, &outputs)` | 执行推理 |
| `Engine::Warmup()` | 静态 shape 模型预热 |
| `RunPipeline(sample, opts)` | 预处理 + 推理一体化 |
| `Decode(...)` | YOLO 网格头解码 |
| `Nms(...)` | 非极大值抑制 |

**线程安全**：`Engine` 非线程安全，每线程一个实例或外部加锁。

### 4.5 障碍感知（costmap 路径，当前可用）

激光障碍感知通过 `map/costmap_2d` 配置，无需 PerceptionServer：

```lua
-- config/map/costmap.lua
obstacle_layer = {
    observation_sources = "scan",
    scan = {
        topic = "/scan",
        max_obstacle_height = 2.0,
        min_obstacle_height = 0.0,
        obstacle_range = 2.5,
        raytrace_range = 3.0,
        marking = true,
        clearing = true,
    },
}
```

C++ 侧 `ObstacleLayer::updateBounds()` 将观测写入 costmap 栅格。

### 4.6 系统集成（规划）

```text
// system::Autonomy 构造流程（规划）
perception_ = std::make_shared<perception::PerceptionServer>(options_.perception_options());
perception_->Start();

// 传感器回调
sensor_consumer->OnImage() → perception_->Process(image);
sensor_consumer->OnLaserScan() → costmap_->updateLaser(scan);  // 当前路径
```

```
相机/激光 ──→ PerceptionServer ──→ vision_msgs
                    │
                    └──→ ObstacleLayer ──→ costmap ──→ Planning
```

### 4.7 驱动层转发（规划）

`config/driver/driver.lua` 中相机配置预留了向 perception 转发：

```lua
front_camera = {
    topic = "/camera/front/image_raw",
    forward_targets = {"perception"},
}
```

### 4.8 故障排查

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| costmap 无障碍 | 激光 topic 未订阅 | 检查 `observation_sources` 与 topic 名 |
| ONNX 推理失败 | 模型路径错误或 EP 不可用 | 确认 `model_path`；CUDA EP 需 ORT GPU 版 |
| TF 变换失败 | 传感器 frame 未发布 | 检查 `sensor_frame` 与 transform 树 |
| 检测框偏移 | 内参或预处理不匹配 | 核对 `GetSpatialSize` 与训练分辨率 |
| PerceptionServer 未启动 | 模块未接入 Autonomy | 当前预期行为；使用 costmap 路径 |
