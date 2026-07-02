# 7. 视觉检测与 ONNX 推理

本文描述 Autonomy 视觉感知基础设施 `autonomy/common/network` 的架构、API 与 YOLO 检测管线。

---

## 7.1 模块定位

`common/network` 提供**与业务无关**的深度学习推理框架，供 Perception 插件（如 `YoloDetector`）调用。命名空间：`autonomy::common::network`。

```
network/
├── network.hpp              # 推荐入口
├── common/                  # Tensor, InferenceOptions
├── backend/                 # Backend, BackendFactory, Engine
│   ├── onnx/                # ONNX Runtime
│   └── tensorrt/            # TensorRT（可选）
├── pipeline/                # RunPipeline
└── detail/
    ├── preprocess/          # 图像 resize, normalize, layout
    └── postprocess/           # YOLO decode, NMS, 分类
```

---

## 7.2 推理流程

```
输入图像 (uint8/float)
    │
    ▼
预处理 (detail/preprocess/)
    ├─ Resize → 模型输入尺寸 (H×W)
    ├─ Normalize → [0,1] 或 ImageNet mean/std
    └─ Layout → NCHW / NHWC
    │
    ▼
Engine::Run(TensorMap)
    ├─ ONNX Runtime / TensorRT
    └─ 输出 TensorMap (float)
    │
    ▼
后处理 (detail/postprocess/)
    ├─ Decode → 网格预测 → 边界框
    ├─ NMS → 去重
    └─ Detection2D / BoundingBox2D
```

---

## 7.3 核心 API

### 7.3.1 创建引擎

```cpp
#include "autonomy/common/network/network.hpp"

InferenceOptions opts;
opts.model_path = "models/yolov8n.onnx";
opts.onnx.execution_provider = "cpu";       // 或 "cuda"
opts.onnx.intra_op_num_threads = 4;

auto engine = BackendFactory::Create(opts);
if (!engine->Warmup()) {
    // 动态 shape 模型 Warmup 为 no-op
}
```

### 7.3.2 执行推理

```cpp
FloatTensorMap inputs;
inputs["images"] = input_tensor;  // shape: [1, 3, H, W]

FloatTensorMap outputs;
auto status = engine->Run(inputs, &outputs);
```

### 7.3.3 一体化管线

```cpp
Sample sample;
sample.image = cv_image;
sample.named_tensors = {};  // 可选：直接提供量化张量

auto result = RunPipeline(sample, opts);
// result.detections → BoundingBox2D 列表
```

---

## 7.4 预处理细节

| 步骤 | 文件 | 说明 |
|------|------|------|
| Resize | `detail/preprocess/resize.cpp` | 双线性插值至模型输入尺寸 |
| Normalize | `detail/preprocess/norm.cpp` | `(pixel - mean) / std` |
| Layout | `detail/preprocess/layout.cpp` | HWC → CHW 转换 |
| Dims | `detail/preprocess/dims.cpp` | 解析输入 spatial size |

**GetSpatialSize**：从模型 metadata 读取 $H \times W$。

---

## 7.5 YOLO 后处理

### 7.5.1 网格解码

`detail/postprocess/boxes.cpp` 实现 grid-style YOLO head 解码：

$$
b_x = \sigma(t_x) + j, \quad b_y = \sigma(t_y) + i
$$

详见 [03_math.md §3.3](03_math.md#33-2d-目标检测yolo-系列)。

### 7.5.2 NMS

`detail/postprocess/nms.cpp`：

1. 按 `score` 降序排列
2. 贪心抑制 IoU $> \tau_{nms}$ 的框
3. 返回保留框索引

### 7.5.3 输出转换

解码结果映射到 `commsgs::vision_msgs::BoundingBox2D`：

```cpp
BoundingBox2D box;
box.center.position.x = cx;   // 像素
box.center.position.y = cy;
box.center.theta = 0.0;
box.size_x = width;
box.size_y = height;
```

---

## 7.6 ONNX Runtime 配置

| 字段 | 说明 | 默认 |
|------|------|------|
| `execution_provider` | `""` / `"cpu"` / `"cuda"` | CPU |
| `device_id` | CUDA 设备索引 | 0 |
| `use_io_binding` | 静态 shape 时跳过 output memcpy | false |
| `intra_op_num_threads` | 算子内并行线程 | 0（ORT 默认） |
| `graph_optimization_level` | 图优化级别 0–3 | -1（默认） |

---

## 7.7 线程安全与性能

| 约束 | 说明 |
|------|------|
| 线程安全 | `Engine` **非**线程安全；每线程一个实例或外部 mutex |
| 动态 shape | `Warmup()` 对非静态 shape 为 no-op |
| IoBinding | 仅当所有输出 shape 完全静态时启用 |
| INT8 量化 | 通过 `Sample::named_tensors` 直接提供校准张量 |

---

## 7.8 与 Perception 插件的集成（规划）

```cpp
class YoloDetector : public PerceptionInterface {
    std::unique_ptr<network::Engine> engine_;

    void Process(const sensor_msgs::Image& img) override {
        Sample sample{ .image = ToCvMat(img) };
        auto result = RunPipeline(sample, opts_);
        detections_ = ToVisionMsgs(result);
    }
};
```

---

## 7.9 构建依赖

| 依赖 | CMake 选项 | 说明 |
|------|------------|------|
| ONNX Runtime | `USE_ONNX` | 默认推理后端 |
| TensorRT | `USE_TENSORRT` | NVIDIA GPU 加速 |
| OpenCV | 默认 | 图像读写与 resize |

源码列表见根 `CMakeLists.txt` 中 `NETWORK_CXX17_SRCS`。
