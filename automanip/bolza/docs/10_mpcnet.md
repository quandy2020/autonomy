# 10　MPC-Net

- 作用：MPC → 神经网络策略；部署用 ONNX 替代在线求解
- C++：`bolza/mpcnet/`（数据生成、rollout、ONNX 推理）
- 宏：`BOLZA_HAS_ONNX`
- **无 Python / pybind 封装**（训练与绑定已移出本仓库）

## 10.1　组件

| 组件 | 位置 |
|:---|:---|
| 状态 / 动作映射 | `mpcnet_definition_base.hpp` |
| 与训练侧交换 | `mpcnet_interface_base.*` |
| 采样与评估 | `rollout/mpcnet_data_*` · `mpcnet_rollout_manager.*` |
| 推理 | `control/mpcnet_onnx_controller.*` |

## 10.2　部署

```cpp
#if defined(BOLZA_HAS_ONNX)
#include <bolza/mpcnet/control/mpcnet_onnx_controller.hpp>
#endif
```

1. 实现 Definition + Interface（基于已有 `RobotInterface`）
2. 导出 `.onnx`
3. 接入 MRT；策略路径用绝对路径
4. 梯度爆炸：开 gradient clipping 并调阈值

## 10.3　示例库（无 ROS / 无 RaiSim）

- C++：`examples/ballbot_mpcnet` · `examples/legged_robot_mpcnet`（`BOLZA_BUILD_MPCNET_EXAMPLES`）
- `raisim=true` 会抛错（本仓库不构建 RaiSim）
