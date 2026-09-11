# Autodriver 文档

配置经 YAML 加载后，由 `SensorModule` 采集，可选经 `SensorHub` 时间对齐，再由 `bridge::Publisher` 发布至 Autolink。  
**Module 按模态固定；Driver 经厂商 Registry 插拔。**

> 正文描述以当前仓库源码与配置的静态分析为依据；构建、测试与实机命令若未在本文档维护环境中执行，不视为运行验证结论。术语以 [术语](guide/glossary.md) 为准。

## 阅读路径

与 MkDocs 导航一致，建议按下列顺序阅读：

| # | 页 | 内容 |
|---|---|---|
| 1 | [快速开始](guide/quickstart.md) | 构建、运行、launch、CMake 开关 |
| 2 | [使用方式](guide/usage.md) | 环境变量、进程、嵌入、故障排查 |
| 3 | [配置](guide/configuration.md) | YAML 全字段 |
| 4 | [生命周期](guide/lifecycle.md) | enable、Attach、udev |
| 5 | [术语](guide/glossary.md) | 模态 / 传感·本体 / 折叠 / 旁路等 |
| 6 | [架构](guide/architecture.md) | 分层、Module↔Registry、加厂商 |
| 7 | [数据流](guide/dataflow.md) | 各模态字节→话题 |
| 8 | [后端](guide/backends.md) | Stream/CAN/各 backend |
| 9 | [测试](guide/testing.md) | ctest 与用例表 |
| 10 | [传感器手册](sensor/index.md) | 厂商页 |
| 11 | [API](api/overview.md) | C++ 接口 |
| 12 | [FAQ](faq.md) | 排障 |

## 数据路径（概要）

```text
硬件 → Stream/CAN/SDK → Driver → SensorSample
     → SampleSink → Publisher → Autolink Writer
```

## 命令

以下命令根据工程配置整理；路径随仓库布局可能不同（见 [快速开始](guide/quickstart.md)）。

```bash
export AUTODRIVER_PATH=$PWD/autodriver   # 含 config/ 的包根
export LD_LIBRARY_PATH=$PWD/build/lib:$LD_LIBRARY_PATH
./build/bin/autodriver
ctest --test-dir build -R test_ --output-on-failure
```

包 README：[README.md](../../README.md)。
