# 2. 快速开始

### 2.1 当前可用能力

Perception 模块本体尚未接入 `Autonomy::Start()`，但以下**相关能力**已可独立使用：

| 能力 | 入口 | 说明 |
|------|------|------|
| 激光障碍 → costmap | `config/map/costmap.lua` + `ObstacleLayer` | 导航栈默认障碍感知路径 |
| ONNX 推理管线 | `autonomy/common/network/network.hpp` | 视觉模型推理（需自行接线） |
| `vision_msgs` 消息 | `autonomy/commsgs/vision_msgs.hpp` | 检测框数据结构 |

### 2.2 障碍感知（通过 costmap，推荐）

导航栈默认通过 `map/costmap_2d` 完成障碍感知，无需启动 `PerceptionServer`：

```lua
-- config/map/costmap.lua（示例片段）
AUTONOMY_MAP = {
    costmap_2d = {
        plugins = { "obstacle_layer", "inflation_layer" },
        obstacle_layer = {
            observation_sources = "scan",
            scan = {
                topic = "/scan",
                sensor_frame = "laser_link",
                data_type = "LaserScan",
                marking = true,
                clearing = true,
            },
        },
    },
}
```

启动完整导航栈后，激光数据经 `ObstacleLayer` 写入代价地图，供 `planning` / `control` 使用。

### 2.3 ONNX 视觉推理（C++ 直接使用）

```cpp
#include "autonomy/common/network/network.hpp"

using namespace autonomy::common::network;

// 1. 创建推理引擎
InferenceOptions opts;
opts.model_path = "/path/to/yolov8.onnx";
opts.onnx.execution_provider = "cpu";  // 或 "cuda"

auto engine = BackendFactory::Create(opts);
engine->Warmup();

// 2. 预处理 + 推理
FloatTensorMap inputs;
// ... 填充图像张量（见 detail/preprocess/image.hpp）

FloatTensorMap outputs;
engine->Run(inputs, &outputs);

// 3. 后处理（YOLO 检测框解码 + NMS）
// 见 detail/postprocess/boxes.hpp, nms.hpp
```

### 2.4 Perception 模块（未来接入步骤）

当 `PerceptionServer` 实现完成后，预期接入方式为：

1. 编辑 `config/perception/perception.lua`
2. 在 `config/autonomy.lua` 中设置 `perception = AUTONOMY_PERCEPTION`
3. 在 `system/options.cpp` 中加载 perception 配置
4. 在 `Autonomy::Start()` 中构造并启动 `PerceptionServer`

```lua
-- config/perception/perception.lua（规划示例）
AUTONOMY_PERCEPTION = {
    enabled = true,
    frequency = 10.0,
    plugins = {
        -- "obstacle_detector:ObstacleDetector",
        -- "yolo_detector:YoloDetector",
    },
    yolo = {
        model_path = "models/yolov8n.onnx",
        confidence_threshold = 0.5,
        nms_threshold = 0.45,
    },
}
```

```lua
-- config/autonomy.lua
AUTONOMY = {
    map = AUTONOMY_MAP,
    planning = AUTONOMY_PLANNER,
    controller = AUTONOMY_CONTROLLER,
    perception = AUTONOMY_PERCEPTION,  -- 待实现后启用
}
```

### 2.5 进程内仿真验证

使用 `autonomy_nav_test` 可在无 ROS 环境下验证障碍感知链路（激光由 costmap 静态地图模拟）：

```bash
export AUTONOMY_BT_PLUGIN_PATH=/workspace/autonomy/build/lib
./bin/autonomy_nav_test \
  --configuration_directory=/workspace/autonomy/src/autonomy/config \
  --start_x=1 --start_y=1 --goal_x=5 --goal_y=5
```

详见 [12 Simulation · nav_test](../12_Simulation/06_nav_test.md)。

### 2.6 依赖检查

| 依赖 | 用途 | 构建选项 |
|------|------|----------|
| OpenCV | 图像预处理 | 默认启用 |
| ONNX Runtime | 神经网络推理 | 可选 `USE_ONNX` |
| TensorRT | GPU 加速推理 | 可选 `USE_TENSORRT` |
| PCL | 点云处理（规划） | 视 map 配置 |
