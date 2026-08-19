# Autodriver 文档

Autodriver 是 autonomy 栈中的传感器采集库：插件化驱动、可选时间对齐（SensorHub），并通过 `bridge::Publisher` 将样本发布到 Autolink 通道。

## 阅读路径

1. [快速开始](guide/quickstart.md) — 环境、构建、`autodriver_hub` 与示例
2. [配置](guide/configuration.md) — `autodriver_hardware.yaml`、环境变量
3. [FAQ](faq.md) — 路径、插件、RealSense 等常见问题

## 架构概览

```
硬件 / 模拟源 → SensorPlugin（采集）→ SensorManager
                                         ├─ SensorHub（可选对齐）
                                         └─ bridge::Publisher → Autolink Writer
```

- **libautodriver**：采集、插件加载、Hub
- **libautodriver_bridge**：唯一 Autolink 发布路径
- **配置**：`AUTODRIVER_PATH` + `config/autodriver_hardware.yaml`

## 常用命令

```bash
export AUTODRIVER_PATH=$PWD
export LD_LIBRARY_PATH=$PWD/build/lib:$LD_LIBRARY_PATH

./build/bin/autodriver_hub
ctest --test-dir build
```
