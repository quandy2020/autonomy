# 10 · 工具

| 工具 | 用途 |
| --- | --- |
| `../tools/configuration_validator_main.cpp` | 校验三份主配置、平台 preset、模型 manifest |
| `../tools/benchmark_main.cpp` | VoxelGrid/KeyframeFilter/CBF/ChunkBlender P50/P99 |
| `../tools/build_linux.sh` | Linux + autolink + Engine 适配器构建脚本 |
| `../tools/goal_sender_main.cpp` | 发布 `/navigation/goal`（点到点/探索） |
| `../tools/export_defm_onnx.py` | DeFM ViT-S/14 PyTorch → ONNX 导出 |
| `../tools/vntpa/` | Python 训练/评估/导出 ONNX 脚手架 |
| `../tools/memory_inspector_main.cpp` / `../tools/replay_main.cpp` | 记忆检查、回放（旧工具，待迁移） |
| `../tools/process_test.py` | 进程回归脚本（旧） |

Linux 构建：

```sh
../tools/build_linux.sh /path/to/automsgs/build build-linux
```

部署入口需在 `Runner::Create` 前调用
`navigation::planning::RegisterEnginePlugins()` 注册 DeFM/VNTPA 工厂。
