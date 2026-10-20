# 2. 快速开始

若需在无整车「感知 Server」的前提下，先跑通激光避障、或自己接 ONNX、或启动 Base/Follow 组件。

| 目标 | 走哪条 |
|------|--------|
| 导航避障（默认） | 激光/点云 → costmap，见 [§8](08_obstacle_perception.md) |
| 视觉检测/跟踪/深度 | `perception/base` + DAG，见 [§10](10_base.md) |
| 跟人走 | base 出 tracks/depth 后开 follow，见 [§11](11_follow.md) |
| 只试 ONNX 引擎 | `common/network`，见 [§7](07_vision_detection.md) |

`PerceptionServer` 只有 `Start`/`Shutdown` 和探索开关告警，**不接** Base/Follow；视觉任务用各自 DAG/launch，不要等 `autonomy.lua` 里一个总开关。

- Base/Follow 与 costmap **是两套栈**，话题和规划入口不同。  
- 模型权重在 `config/perception/base/`，仓库里通常没有，需自行导出。  
- `config/perception/perception.lua` 存在，但**未**挂进主进程 `Autonomy::Start()`（与文档旧描述不同，以代码为准）。

### 障碍感知（costmap，可立刻用）

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

### ONNX 最小调用

```cpp
#include "autonomy/common/network/network.hpp"
using namespace autonomy::common::network;

InferenceOptions opts;
opts.model_path = "/path/to/model.onnx";
opts.onnx.execution_provider = "cpu"; // 或 "cuda"

auto engine = BackendFactory::Create(opts);
engine->Warmup();
FloatTensorMap inputs, outputs;
engine->Run(inputs, &outputs);
```

后处理见 `common/network/detail/postprocess/`；业务侧多任务见 [§10 Base](10_base.md)。

### 多进程联调

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
autolink_launch autonomy.launch
```

### 构建选项

| 依赖 | 用途 |
|------|------|
| OpenCV | 图像预处理 |
| ONNX Runtime | `USE_ONNX` |
| TensorRT | `USE_TENSORRT` |

### 常见坑

| 现象 | 先查 |
|------|------|
| 开了 `perception.enabled` 仍无检测 | Server 不驱动 Base；改跑 base DAG |
| costmap 空 | `/scan`、TF、`observation_sources` |
| exploration 无输出 | 后端未编，见 [§12](12_exploration.md) |

### 延伸阅读

[04 Running](../04_Running/02_quickstart.md) · [§7 视觉](07_vision_detection.md) · [§8 障碍](08_obstacle_perception.md)
