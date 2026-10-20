# 2. 快速开始

### 2.1 可用能力

| 能力 | 入口 | 说明 |
|------|------|------|
| 激光 → costmap | `config/map/costmap.lua` + `ObstacleLayer` | 导航默认路径 |
| ONNX 推理 | `common/network/network.hpp` | 需自行接线 |
| `vision_msgs` | `commsgs/vision_msgs.hpp` | 检测框结构 |

### 2.2 障碍感知（costmap）

```lua
-- config/map/costmap.lua
obstacle_layer = {
    observation_sources = "scan",
    scan = {
        topic = "/scan",
        sensor_frame = "laser_link",
        data_type = "LaserScan",
        marking = true,
        clearing = true,
    },
}
```

### 2.3 ONNX 推理

```cpp
#include "autonomy/common/network/network.hpp"
using namespace autonomy::common::network;

InferenceOptions opts;
opts.model_path = "/path/to/yolov8.onnx";
opts.onnx.execution_provider = "cpu"; // 或 "cuda"

auto engine = BackendFactory::Create(opts);
engine->Warmup();

FloatTensorMap inputs, outputs;
// 填充 inputs；后处理见 detail/postprocess/
engine->Run(inputs, &outputs);
```

### 2.4 未来接入（规划）

1. 编辑 `config/perception/perception.lua`
2. `autonomy.lua` 设 `perception = AUTONOMY_PERCEPTION`
3. `system/options.cpp` 加载；`Autonomy::Start()` 构造 `PerceptionServer`

```lua
AUTONOMY_PERCEPTION = {
    enabled = true,
    frequency = 10.0,
    yolo = {
        model_path = "models/yolov8n.onnx",
        confidence_threshold = 0.5,
        nms_threshold = 0.45,
    },
}
```

### 2.5 联调

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
autolink_launch autonomy.launch
```

### 2.6 依赖

| 依赖 | 用途 | 选项 |
|------|------|------|
| OpenCV | 图像预处理 | 默认 |
| ONNX Runtime | 推理 | `USE_ONNX` |
| TensorRT | GPU | `USE_TENSORRT` |

### 2.7 参见

- [04 Running](../04_Running/02_quickstart.md) · [§7 视觉](07_vision_detection.md) · [§8 障碍](08_obstacle_perception.md)
