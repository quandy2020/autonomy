# Autodriver 文档

YAML → `SensorModule` 采集 → 可选 `SensorHub` 对齐 → `bridge::Publisher` → Autolink。  
**Module 按模态固定；Driver 按厂商 Registry 插拔。**

## 阅读路径

| # | 页 | 内容 |
|---|---|---|
| 1 | [快速开始](guide/quickstart.md) | 构建、运行、launch、CMake 开关 |
| 2 | [使用方式](guide/usage.md) | 环境变量、进程、嵌入、故障速查 |
| 3 | [架构](guide/architecture.md) | 分层、Module↔Registry、加厂商 |
| 4 | [数据流](guide/dataflow.md) | 各模态字节→话题 |
| 5 | [配置](guide/configuration.md) | YAML 全字段 |
| 6 | [生命周期](guide/lifecycle.md) | enable、Attach、udev |
| 7 | [后端](guide/backends.md) | Stream/CAN/各 backend |
| 8 | [传感器手册](sensor/index.md) | 厂商页 |
| 9 | [测试](guide/testing.md) | ctest 与用例表 |
| 10 | [API](api/overview.md) | C++ 接口 |
| 11 | [FAQ](faq.md) | 排障 |

## 数据路径（缩略）

```
硬件 → Stream/CAN/SDK → Driver → SensorSample
     → SampleSink → Publisher → Autolink Writer
```

## 命令

```bash
export AUTODRIVER_PATH=$PWD/src/autonomy/autodriver   # 含 config/
export LD_LIBRARY_PATH=$PWD/build/lib:$LD_LIBRARY_PATH
./build/bin/autodriver
ctest --test-dir build -R test_ --output-on-failure
```

包 README：[README.md](../../README.md)。
