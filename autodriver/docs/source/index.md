# Autodriver 文档

YAML → 传感采集 / 底盘执行 → Autolink。  
**Module 按模态固定；Driver 经厂商 Registry 插拔。**

> 正文以仓库源码与配置为准；未在文档环境中执行的构建/实机命令，不视为运行验证结论。术语见 [术语](guide/glossary.md)。

## 阅读路径

| # | 页 | 内容 |
|---|---|---|
| 1 | [快速开始](guide/quickstart.md) | 构建、环境、launch / DAG |
| 2 | [使用方式](guide/usage.md) | 进程、嵌入、排障 |
| 3 | [配置](guide/configuration.md) | YAML 全字段 |
| 4 | [本体 chassis](guide/chassis.md) | 通道、JetAuto / L1-W、DAG 二选一 |
| 5 | [生命周期](guide/lifecycle.md) | enable、Attach、udev |
| 6 | [术语](guide/glossary.md) | 模态 / 折叠 / 旁路 |
| 7 | [架构](guide/architecture.md) | 分层、Registry、加厂商 |
| 8 | [数据流](guide/dataflow.md) | 字节 → 话题 |
| 9 | [后端](guide/backends.md) | Stream / CAN / 各 backend |
| 10 | [测试](guide/testing.md) | ctest |
| 11 | [传感器手册](sensor/index.md) | 厂商页 |
| 12 | [API](api/overview.md) | C++ |
| 13 | [FAQ](faq.md) | 排障 |

## 路径概要

```text
传感: 硬件 → Driver → SampleSink → Publisher → Autolink
底盘: /cmd_vel · mode/tool → SafetyGate → ChassisDriver → RobotState / odom
启动: autodriver（传感）+ mainboard DAG（JetAuto XOR L1-W）
```

## 命令

```bash
export AUTODRIVER_PATH=$PWD/autodriver
export AUTOLINK_DAG_PATH=$AUTODRIVER_PATH/dag
export AUTOLINK_LIB_PATH=$PWD/build/lib
export LD_LIBRARY_PATH=$PWD/build/lib:$LD_LIBRARY_PATH
./build/bin/autodriver
./build/bin/autodriver --pair-joy                    # DualSense 蓝牙配对
./build/bin/autodriver --pair-joy --pair-mode usb    # USB / 驱动
autolink launch start autodriver.launch   # 需 AUTOLINK_LAUNCH_PATH
```

手柄遥操与配对：[使用 · DualSense](guide/usage.md#21-dualsense-遥操) · [配置 · joy](guide/configuration.md#41-手柄遥操joy默认索尼-dualsenseps5)。

包 README：[README.md](../../README.md)。
