# 7. 视觉检测与 ONNX 推理

若需在 **Base 之外** 自写推理节点，或理解 `common/network` 与 `perception/base` 的分工。

`autonomy::common::network` 是**与业务无关**的 ONNX/TensorRT 引擎封装；YOLO 多任务、MoGe 深度在 **`perception/base`** 里接好话题（[§10](10_base.md)）。

- `Engine` **非线程安全**：每线程一实例或外部加锁。  
- Base 里部分 YOLO head 的 decode 仍在补全，开任务前确认 engine 已按 README 导出。  
- 本库不发布 ROS/commsgs 话题，只给 tensor 进出的 API。

与 Base 的关系：

| 层 | 位置 |
|----|------|
| 引擎 | `common/network` |
| 任务调度、话题、MoGe | `perception/base/tasks/*` |

插件式 `YoloDetector : PerceptionInterface` 在文档里出现过，**不是**当前 Base 的主路径；Base 用 `base_component` + proto tasks。

| 目标 | 做法 |
|------|------|
| 开箱检测话题 | [§10 Base](10_base.md) |
| 嵌入新模型 | network + 自写后处理 |
| 导航避障 | [§8 costmap](08_obstacle_perception.md)，不必 ONNX |

### 延伸阅读

```text
common/network/
├── network.hpp
├── backend/          # ONNX / TensorRT
├── pipeline/         # RunPipeline
└── detail/preprocess, postprocess/
```

### 最小流程

```text
Image → Resize/Norm/Layout → Engine::Run → Decode + NMS → 自定义或 vision_msgs
```

```cpp
InferenceOptions opts;
opts.model_path = "models/yolov8n.onnx";
opts.onnx.execution_provider = "cpu";
auto engine = BackendFactory::Create(opts);
engine->Warmup();
FloatTensorMap inputs{{"images", input_tensor}}, outputs;
engine->Run(inputs, &outputs);
```

### ONNX 常用选项

| 字段 | 说明 |
|------|------|
| `execution_provider` | `cpu` / `cuda` |
| `device_id` | CUDA 设备 |
| `graph_optimization_level` | 0–3 |

构建：`USE_ONNX`、`USE_TENSORRT`、OpenCV。

### 常见坑

| 现象 | 说明 |
|------|------|
| Warmup 无效果 | 动态 shape 时可能 no-op |
| 与 Base 模型路径混用 | Base 用 `config/perception/base/*.pb.txt` |

### 延伸阅读

[§3.3 数学](03_math.md) · [14 vision_msgs](../14_Commsgs/08_nav_planning_msgs.md)
