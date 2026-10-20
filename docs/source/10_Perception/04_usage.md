(perception-usage)=
# 4. 使用指南

### 4.1 配置（规划）

入口：`config/perception/perception.lua` → `autonomy.lua`。当前空表，未挂入 `AUTONOMY`。

| 字段 | 说明 | 建议 |
|------|------|------|
| `enabled` | 启动 PerceptionServer | `false` |
| `frequency` | Hz | `10.0` |
| `plugins` | 插件列表 | 待实现 |
| `yolo.model_path` | ONNX 路径 | — |
| `yolo.confidence_threshold` | 置信度 | `0.5` |
| `yolo.nms_threshold` | NMS IoU | `0.45` |

### 4.2 PerceptionServer API（规划）

| API | 用途 | 状态 |
|-----|------|------|
| `Start()` / `Shutdown()` | 启停 | — |
| `Process(sensor_data)` | 单帧处理 | — |
| `GetDetections()` / `GetObstacles()` | 取结果 | — |

### 4.3 vision_msgs

| 消息 | 用途 |
|------|------|
| `BoundingBox2D` / `BoundingBox2DArray` | 2D 框 |
| `BoundingBox3D` | 3D 框 |
| `Detection2D` / `Detection2DArray` | 框 + 分类 |
| `Detection3D` / `Detection3DArray` | 3D 检测 |
| `Classification` | 分类 |

```cpp
commsgs::vision_msgs::BoundingBox2D box;
box.center.position.x = 320.0;
box.center.position.y = 240.0;
box.center.theta = 0.0;
box.size_x = 100.0;
box.size_y = 80.0;
```

### 4.4 common/network API

| API | 说明 |
|-----|------|
| `BackendFactory::Create(opts)` | 创建引擎 |
| `Engine::Run` / `Warmup` | 推理 / 预热 |
| `RunPipeline` | 预处理 + 推理 |
| `Decode` / `Nms` | YOLO 解码 / NMS |

`Engine` 非线程安全：每线程一实例或外部加锁。

### 4.5 障碍感知（当前可用）

```lua
obstacle_layer = {
    observation_sources = "scan",
    scan = {
        topic = "/scan",
        obstacle_range = 2.5,
        raytrace_range = 3.0,
        marking = true,
        clearing = true,
    },
}
```

### 4.6 故障排查

| 现象 | 处理 |
|------|------|
| costmap 无障碍 | 查 `observation_sources` / topic |
| ONNX 失败 | 查 `model_path`、CUDA EP |
| TF 失败 | 查 `sensor_frame` |
| PerceptionServer 未启动 | 预期行为；用 costmap 路径 |

### 4.7 参见

- [§2](02_quickstart.md) · [§7](07_vision_detection.md) · [§8](08_obstacle_perception.md) · [14 Commsgs](../14_Commsgs/00_guide.md)
