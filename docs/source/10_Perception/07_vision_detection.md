# 7. 视觉检测与 ONNX 推理

命名空间：`autonomy::common::network`。业务无关推理框架，供 Perception 插件调用。

### 7.1 目录

```
network/
├── network.hpp
├── common/ # Tensor, InferenceOptions
├── backend/ # ONNX / TensorRT
├── pipeline/ # RunPipeline
└── detail/{preprocess,postprocess}/
```

### 7.2 流程

```
Image → Resize/Norm/Layout → Engine::Run → Decode + NMS → BoundingBox2D
```

### 7.3 API

```cpp
InferenceOptions opts;
opts.model_path = "models/yolov8n.onnx";
opts.onnx.execution_provider = "cpu"; // 或 "cuda"
auto engine = BackendFactory::Create(opts);
engine->Warmup();

FloatTensorMap inputs{{"images", input_tensor}}, outputs;
engine->Run(inputs, &outputs);

// 或：auto result = RunPipeline(sample, opts);
```

### 7.4 预处理 / 后处理

| 步骤 | 文件 |
|------|------|
| Resize / Norm / Layout | `detail/preprocess/` |
| YOLO Decode / NMS | `detail/postprocess/boxes.cpp`, `nms.cpp` |

公式见 [§3.3](03_math.md)。输出映射 `vision_msgs::BoundingBox2D`。

### 7.5 ONNX 配置

| 字段 | 说明 | 默认 |
|------|------|------|
| `execution_provider` | `cpu` / `cuda` | CPU |
| `device_id` | CUDA 索引 | 0 |
| `use_io_binding` | 静态 shape 跳过 memcpy | false |
| `intra_op_num_threads` | 算子内线程 | 0 |
| `graph_optimization_level` | 0–3 | -1 |

### 7.6 约束与构建

| 项 | 说明 |
|----|------|
| 线程 | `Engine` 非线程安全 |
| Warmup | 动态 shape 为 no-op |
| 依赖 | `USE_ONNX`、`USE_TENSORRT`、OpenCV |

### 7.7 插件集成（规划）

```cpp
class YoloDetector : public PerceptionInterface {
    void Process(const sensor_msgs::Image& img) override {
        auto result = RunPipeline({.image = ToCvMat(img)}, opts_);
        detections_ = ToVisionMsgs(result);
    }
};
```

业务侧视觉任务在 [§10 Base](10_base.md)（YOLO26 / MoGe），本页为底层 `common/network` 引擎。

### 7.8 参见

- [§10 Base](10_base.md) · [§3](03_math.md) · [14 vision_msgs](../14_Commsgs/00_guide.md)
