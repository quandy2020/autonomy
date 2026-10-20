(perception-usage)=
# 4. 使用指南

调 costmap 观测源、写 `common/network` 推理、或查 `vision_msgs` 字段；**不是** PerceptionServer 产品 API 手册。

| 块 | 说明 |
|----|------|
| Costmap 障碍层 | 当前车上默认可用，[§8](08_obstacle_perception.md) |
| `common/network` | ONNX/TensorRT 推理，`Engine` 非线程安全 |
| `vision_msgs` | 检测框等 C++ struct，边界走 ToProto 时需核对 [14](../14_Commsgs/09_survey.md) |
| `PerceptionServer` | 仅独立进程 `perception_main` 或未来总装；`Start()` 建 node，探索开关只打 WARN |

- `config/perception/perception.lua` / `PerceptionOptions` 里的 YOLO 字段是**预留**，未驱动 `perception/base`。  
- 文档若出现 `Process()` / `GetDetections()`，仓库里**没有**这些接口。  
- Follow 输出是 GridMap 话题，**不会**写入 OccupancyGrid。

- **只要导航避障** → 只配 costmap，不必开 perception 进程。  
- **要视觉话题** → base.pb.txt + base DAG。  
- **要跟人** → follow 配置 + MPPI GridMap critic。

### PerceptionOptions（proto，非 Base 任务表）

入口文件：`autonomy/perception/conf/*.pb.txt`、Lua 代码生成项。与 Base 的 `base.proto` `tasks` **不是同一张配置**。

| 字段 | 含义 | 备注 |
|------|------|------|
| `enabled` | Server 是否 `Start` | 默认常 false |
| `enable_rgbd_exploration` | 探索后端 | 未构建则 WARN |
| `frequency` 等 | 预留 | 未接 Base |

### vision_msgs 常用类型

| 消息 | 用途 |
|------|------|
| `BoundingBox2D` / `BoundingBox2DArray` | 2D 框 |
| `Detection2D` / `Detection2DArray` | 框 + 分类 |
| `BoundingBox3D` / `Detection3D*` | 3D |

```cpp
commsgs::vision_msgs::BoundingBox2D box;
box.center.position.x = 320.0;
box.center.position.y = 240.0;
box.size_x = 100.0;
box.size_y = 80.0;
```

### common/network 要点

| API | 说明 |
|-----|------|
| `BackendFactory::Create(opts)` | 创建引擎 |
| `Engine::Run` / `Warmup` | 推理 / 预热 |
| `RunPipeline` | 预处理 + 推理 |
| `Decode` / `Nms` | YOLO 后处理 |

### 障碍层配置示例

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

### 常见坑

| 现象 | 处理 |
|------|------|
| costmap 无障碍 | topic、TF、`data_type` |
| ONNX EP 失败 | `model_path`、CUDA 与 EP 名 |
| TF 查不到 | `sensor_frame` 与 buffer |
| Server 起了无业务 | 预期；改用 base/follow |

### 延伸阅读

[§2](02_quickstart.md) · [§7](07_vision_detection.md) · [§8](08_obstacle_perception.md) · [14 Commsgs](../14_Commsgs/00_guide.md)
