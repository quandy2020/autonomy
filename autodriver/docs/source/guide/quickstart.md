# 快速开始

在 autonomy 根目录（含顶层 `CMakeLists.txt`）构建 Autodriver，并运行 hub 示例。

## 1. 环境

```bash
cd src/autonomy
export AUTODRIVER_PATH=$PWD/autodriver        # 须含 config/autodriver_hardware.yaml
export LD_LIBRARY_PATH=$PWD/build/lib:$LD_LIBRARY_PATH
export PATH=$PWD/build/bin:$PATH
```

与 Autolink 联调时通常还需：

```bash
export AUTOLINK_PATH=$PWD/autolink
```

调试日志：`export GLOG_logtostderr=1`。

## 2. 构建

在 autonomy 根目录：

```bash
cmake -S . -B build -DBUILD_AUTODRIVER=ON
cmake --build build -j8
```

| 开关 | 默认 | 作用 |
|---|---|---|
| `AUTODRIVER_BUILD_TEST` | ON | 单元测试（`ctest`） |
| `AUTODRIVER_BUILD_EXAMPLES` | ON | `autodriver_demo` |
| `AUTODRIVER_BUILD_DOCS` | ON | MkDocs 目标 `docs` |
| `AUTODRIVER_WITH_REALSENSE` | ON | RealSense 驱动（需 librealsense2） |

## 3. 运行 hub

### 直接运行

默认读取 `config/autodriver_hardware.yaml`：

```bash
autodriver_hub
```

编辑 YAML：将已连接传感器的 `enable` 设为 `true`。

`autodriver_hub` 参数：

1. **configuration_directory**（可选）：配置根目录
2. **configuration_file**（可选）：默认 `autodriver_hardware.yaml`

### 使用 autolink launch 启动（推荐）

```bash
export AUTODRIVER_PATH=$PWD/autodriver
export AUTOLINK_LAUNCH_PATH=$PWD/autodriver/launch
export LD_LIBRARY_PATH=$PWD/build/lib:$LD_LIBRARY_PATH
export PATH=$PWD/build/bin:$PATH

autolink launch start autodriver.launch
```

停止：

```bash
autolink launch stop autodriver.launch
```

查看运行状态：

```bash
autolink launch list
```

launch 文件位于 `launch/autodriver.launch`。  
`autodriver_hub` 默认配置 `exception_handler: respawn`，崩溃后最多自动重启 3 次。  
需要自定义配置目录时，可在 launch 文件的 `<process_name>` 中传参：

```xml
<process_name>autodriver_hub /path/to/config autodriver_hardware.yaml</process_name>
```

## 4. Demo

```bash
./build/bin/autodriver_demo
```

## 5. ROS 2 集成

```yaml
driver_hub_node:
  ros__parameters:
    configuration_directory: ""
    configuration_file: "autodriver_hardware.yaml"
```

## 6. 下一步

| 需求 | 文档 |
|---|---|
| 配置字段 | [配置](configuration.md) |
| 路径与插件问题 | [FAQ](../faq.md) |
