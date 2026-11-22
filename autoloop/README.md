# Autoloop

机器人端云数据闭环：机上 `data_agent` 旁路采集与上传，云端入库、转换、治理、训练、发布，再经 `model_sync` 回灌机上。

不替代 `autonomy` 的 planning / control / perception；机上采集与 `autonomy.recorder` 对齐（四元组、MCAP、P0–P3）。

## 布局

- `cmake/` CMake 模块
- `autoloop/common` 协议与薄封装（path/time 转发 autolink）
- `autoloop/robot` 机上代理、同步、bringup
- `autoloop/cloud` 入库 → 转换 → 治理 → 训练 → 发布
- `docs/` 闭环设计（回溯 / 追查 / 回放 / 可视化 / 回灌），见 [docs/README.md](docs/README.md)

包含根：`AUTOLOOP_SRC_ROOT` → `autoloop/`。头文件写成 `#include "common/..."`、`"robot/..."`、`"cloud/..."`。

## 构建

```bash
cmake --preset default
cmake --build build
```
