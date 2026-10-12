# 6　配置

- 格式：`.info`（`bolza::property_tree` INFO 子集），非 YAML
- 模板：`examples/cartpole/config/mpc/task.info`
- 示例路径：编译期宏（如 `BOLZA_EXAMPLE_TASK`）；自建节点自行传入

## 6.1　分块

| 块 | 加载 | 内容 |
|:---|:---|:---|
| `<model>_parameters` | 示例代码 | 质量、几何、限幅 |
| `ddp` | `ddp::LoadSettings` | 迭代、线程、ODE、线搜索/LM |
| `mpc` | `mpc::LoadSettings` | 时域、MRT/MPC 频率、冷热启动 |
| `rollout` | `rollout::LoadSettings` | 积分器、步长 |

## 6.2　常用字段

| 字段 | 用途 |
|:---|:---|
| `ddp.maxNumIterations` | 单次 `advanceMpc` 迭代上限；实时常 $1$ |
| `ddp.nThreads` | 并行线程 |
| `ddp.useFeedbackPolicy` | $u=u_{ff}+K(x-\bar x)$ |
| `ddp.displayShortSummary` | 耗时摘要 |
| `mpc.timeHorizon` | 预测时域（秒） |
| `mpc.mrtDesiredFrequency` | MRT（Hz，示例常 400） |
| `mpc.mpcDesiredFrequency` | 求解线程（Hz） |
| `mpc.coldStart` | 每周期冷启动 |

- 键名以 `*_settings.hpp` / `loadSettings` 为准
- SQP / IPM / SLP 各有 settings

## 6.3　CppAD 与宏

- `libraryFolder`（如 `/tmp/bolza/<name>`）须可写
- 多机器人勿共用；删目录可强制重生

```cpp
MyInterface(taskFile, libraryFolder, /*verbose=*/false);
```

- 示例宏：`BOLZA_EXAMPLE_TASK` · `BOLZA_EXAMPLE_URDF` · 参考文件宏
- 特性宏：`BOLZA_HAS_PINOCCHIO` · `BOLZA_HAS_COAL` · `BOLZA_HAS_ONNX`
