# Autolink 项目

Autolink 是一个轻量、高性能的分布式通信与任务调度框架，灵感来源于 Apollo Cyber RT。Autolink 提供节点 `Node` 抽象以及基于 Channel 的发布/订阅（`Writer`/`Reader`）通信、Service/Client RPC、服务发现、定时器与调度、日志、数据记录等能力，适用于机器人、中间件、边缘计算等场景。

本项目使用 Bazel 构建系统，提供了完整的异步编程、调度、定时器等核心功能。

## 项目结构

```
.
├── MODULE.bazel       # Bazel 模块定义（Bazel 8+ bzlmod）
├── .bazelversion      # 指定 Bazel 版本
├── .bazelrc           # Bazel 配置文件（支持平台特定配置）
├── autolink/          # 源代码目录
│   ├── BUILD.bazel    # 统一 autolink 库
│   ├── base/          # 基础工具库（队列、锁、对象池、信号等）
│   ├── blocker/       # 阻塞器（用于数据同步）
│   ├── class_loader/  # 类加载器（动态库加载）
│   ├── common/        # 通用功能库（日志、文件操作、初始化、状态管理等）
│   ├── component/     # 组件系统（定时器组件等）
│   ├── context/       # 上下文管理（线程本地存储）
│   ├── croutine/      # 协程实现
│   ├── data/          # 数据分发和管理（缓存、通道、数据融合等）
│   ├── event/         # 事件系统（性能事件缓存）
│   ├── logger/        # 日志系统
│   ├── mainboard/     # 主板模块（模块控制器）
│   ├── message/       # 消息系统（消息包装、工厂等）
│   ├── node/          # 节点功能（Reader、Writer、Service、Client）
│   ├── parameter/     # 参数系统（参数服务器和客户端）
│   ├── plugin_manager/# 插件管理器
│   ├── profiler/      # 性能分析器
│   ├── proto/         # Protocol Buffers 定义
│   ├── scheduler/     # 调度器（经典调度、编排调度）
│   ├── service/       # 服务系统（服务端和客户端）
│   ├── service_discovery/# 服务发现（拓扑管理）
│   ├── sysmo/         # 系统监控
│   ├── task/          # 任务管理（异步任务执行）
│   ├── time/          # 时间管理（Time, Duration, Clock, Rate）
│   ├── timer/         # 定时器（时间轮实现）
│   ├── transport/     # 传输层（SHM、RTPS、Intra 通信）
│   ├── examples/      # 示例程序
│   │   ├── talker.cpp # 发送消息示例
│   │   ├── listener.cpp # 接收消息示例
│   │   └── proto/     # 示例 proto 定义
│   └── io/            # IO 模块（Linux 特定，macOS 上已排除）
├── scripts/           # 构建脚本
│   └── autolink.py    # 统一构建脚本（自动平台检测）
├── docs/              # 文档目录
├── cmake/             # CMake 配置文件
└── .vscode/           # VS Code 配置（调试配置、任务配置等）
```

## 特性

- **节点与通信**: 通过 `Node` 创建 `Writer`/`Reader` 实现发布/订阅；支持 QoS 配置与本地/跨进程传输
- **服务与客户端**: `Service<Request, Response>` 与 `Client<Request, Response>` 实现请求/响应模式
- **传输层**: 内存队列、共享内存（SHM）、RTPS 等多种传输后端（根据配置与编译选项选择）
- **服务发现**: 拓扑管理、角色注册，自动追踪节点/通道/服务状态
- **调度与协程**: 经典/编排两类调度策略与协程支持
- **日志与统计**: 统一日志、性能统计与指标收集
- **记录与回放**: 记录器与播放器，便于数据复现与离线分析（计划中）

## 依赖

项目使用以下外部依赖（通过 Bazel bzlmod 管理）：

- **rules_cc** (0.2.14) - C++ 构建规则
- **rules_proto** (7.0.2) - Protocol Buffers 规则
- **protobuf** (29.0) - Protocol Buffers 库
- **glog** (0.7.0) - Google 日志库
- **googletest** (1.17.0) - Google 测试框架
- **nlohmann_json** (3.12.0) - JSON 库
- **fastdds** (3.1.1) - Fast DDS 库
- **platforms** (0.0.10) - 平台定义（用于跨平台构建）

## 快速开始

### 前置要求

- **Bazel 8.0+** - 请参考 [Bazel 官方文档](https://bazel.build/install) 安装
- **C++17** 或更高版本
- **macOS/Linux** 系统
- **Python 3.x** - 用于运行构建脚本

### 构建项目

#### 使用构建脚本（推荐）

构建脚本会自动检测平台（macOS/Linux）并应用相应的编译选项：

```bash
# 构建所有目标
python3 scripts/autolink.py build

# 构建特定目标
python3 scripts/autolink.py build //autolink:talker

# 构建统一 autolink 库
python3 scripts/autolink.py build //autolink:autolink

# 详细输出
python3 scripts/autolink.py build -v

# 调试模式构建（包含调试符号，无优化）
python3 scripts/autolink.py build --debug //autolink:talker
```

#### 使用 Bazel 直接构建

```bash
# 构建所有目标（自动应用平台配置）
bazel build //...

# 手动指定平台配置
bazel build --config=macos //...    # macOS
bazel build --config=linux //...   # Linux

# 调试模式构建（包含调试符号，无优化）
bazel build --config=dbg //autolink:talker
bazel build --config=macos --config=dbg //autolink:autolink

# 构建统一 autolink 库
bazel build //autolink:autolink

# 构建示例程序
bazel build //autolink:talker
bazel build //autolink:listener
```

### 运行程序

#### 使用构建脚本

```bash
# 运行 talker 示例
python3 scripts/autolink.py run //autolink:talker

# 运行 listener 示例
python3 scripts/autolink.py run //autolink:listener

# 调试模式运行
python3 scripts/autolink.py run --debug //autolink:talker
```

#### 使用 Bazel 直接运行

```bash
# 运行 talker 示例
bazel run //autolink:talker

# 运行 listener 示例
bazel run //autolink:listener

# 调试模式运行
bazel run --config=dbg //autolink:talker
```

### 运行测试

#### 使用构建脚本

```bash
# 运行所有测试
python3 scripts/autolink.py test

# 运行特定测试
python3 scripts/autolink.py test //autolink:log_test
python3 scripts/autolink.py test //autolink:timer_test

# 调试模式运行测试
python3 scripts/autolink.py test --debug //autolink:log_test

# 显示详细测试输出
python3 scripts/autolink.py test -v
```

#### 使用 Bazel 直接运行

```bash
# 运行所有测试
bazel test //...

# 运行特定测试
bazel test //autolink/:log_test
bazel test //autolink/time:time_test
bazel test //autolink/timer:timer_test
```

### 安装构建产物

```bash
# 安装 autolink 库和头文件到系统目录（默认安装 //autolink:autolink）
python3 scripts/autolink.py install --prefix /usr/local

# 安装特定可执行文件
python3 scripts/autolink.py install //autolink:talker

# 安装到指定目录
python3 scripts/autolink.py install //autolink:talker --prefix /usr/local/bin

# 安装文档和 CMake 配置文件
python3 scripts/autolink.py install --prefix /usr/local --install-docs --install-cmake
```

### 清理构建

```bash
# 清理构建产物
python3 scripts/autolink.py clean

# 完全清理（包括缓存）
python3 scripts/autolink.py clean --expunge

# 或使用 Bazel
bazel clean
bazel clean --expunge
```

### 查询目标

```bash
# 查询所有二进制目标
python3 scripts/autolink.py query 'kind(cc_binary, //...)'

# 查询 autolink 库的依赖
python3 scripts/autolink.py query 'deps(//autolink:autolink)'

# 或使用 Bazel
bazel query 'kind(cc_binary, //...)'
```

## 平台特定配置

项目支持 macOS 和 Linux 平台的特定编译选项，构建脚本会自动检测平台并应用相应配置：

- **macOS**: 对齐分配支持、链接器警告抑制等
- **Linux**: 链接器警告抑制、位置无关代码等

配置定义在 `.bazelrc` 文件中，可以通过 `--config=macos` 或 `--config=linux` 手动指定。

**注意**：`io` 模块使用 Linux 特定的 epoll API，在 macOS 上已自动排除。如需在 macOS 上使用 IO 功能，需要实现基于 kqueue 的替代方案。

## 调试配置

项目支持调试模式构建，包含完整的调试符号和无优化编译：

### 使用构建脚本

```bash
# 调试模式构建
python3 scripts/autolink.py build --debug //autolink:talker

# 调试模式运行测试
python3 scripts/autolink.py test --debug //autolink:log_test

# 调试模式运行程序
python3 scripts/autolink.py run --debug //autolink:talker
```

### 使用 Bazel

```bash
# 调试模式构建
bazel build --config=dbg //autolink:talker

# 结合平台配置
bazel build --config=macos --config=dbg //autolink:autolink

# 调试模式运行测试
bazel test --config=dbg //autolink:log_test

# 调试模式运行程序
bazel run --config=dbg //autolink:talker
```

### VS Code 调试

项目包含 VS Code 调试配置（`.vscode/launch.json` 和 `.vscode/tasks.json`），支持：

- **C++ Debug (Bazel)**: 调试可执行文件（如 `talker`）
- **C++ Debug Test (Bazel)**: 调试测试程序（如 `transport_shm_talker_test`）
- **C++ Attach (Bazel)**: 附加到运行中的进程

调试配置会自动使用调试模式构建目标。使用方法：

1. 在 VS Code 中打开项目
2. 按 `F5` 或点击调试按钮开始调试
3. 选择相应的调试配置（C++ Debug (Bazel) 或 C++ Debug Test (Bazel)）
4. 设置断点后开始调试

**注意**：调试前会自动执行预构建任务（`bazel-build-debug` 或 `bazel-build-debug-test`），确保使用调试模式构建。

### 调试配置说明

调试模式（`--config=dbg`）包含：
- 调试符号（`-g`）
- 无优化（`-O0`）
- 不剥离符号（`--strip=never`）
- 定义 `DEBUG` 宏

快速构建模式（`--config=fastbuild`）包含：
- 调试符号（`-g`）
- 轻度优化（`-O1`）

## 库结构

### 统一 autolink 库

项目提供了一个统一的 `autolink` 库，包含所有子模块：

```python
# 在 BUILD.bazel 中使用
deps = ["//autolink:autolink"]
```

这将自动包含：
- `base` - 基础工具（队列、锁、对象池、信号等）
- `blocker` - 阻塞器（用于数据同步）
- `class_loader` - 类加载器（动态库加载）
- `common` - 通用功能（日志、文件操作、初始化、状态管理等）
- `component` - 组件系统（定时器组件等）
- `context` - 上下文管理（线程本地存储）
- `croutine` - 协程实现
- `data` - 数据分发和管理（缓存、通道、数据融合等）
- `event` - 事件系统（性能事件缓存）
- `logger` - 日志系统
- `mainboard` - 主板模块（模块控制器）
- `message` - 消息系统（消息包装、工厂等）
- `node` - 节点功能（Reader、Writer、Service、Client）
- `parameter` - 参数系统（参数服务器和客户端）
- `plugin_manager` - 插件管理器
- `profiler` - 性能分析器
- `scheduler` - 调度器（经典调度、编排调度）
- `service` - 服务系统（服务端和客户端）
- `service_discovery` - 服务发现（拓扑管理）
- `sysmo` - 系统监控
- `task` - 任务管理（异步任务执行）
- `time` - 时间管理（Time, Duration, Clock, Rate）
- `timer` - 定时器（时间轮实现）
- `transport` - 传输层（SHM、RTPS、Intra 通信）
- `proto_cc` - Protocol Buffers C++ 代码生成

### 子模块库

也可以单独使用各个子模块：

```python
deps = [
    "//autolink/base:base",              # 基础工具
    "//autolink/blocker:blocker",        # 阻塞器
    "//autolink/class_loader:class_loader", # 类加载器
    "//autolink/common:common",          # 通用功能
    "//autolink/component:component",    # 组件系统
    "//autolink/context:context",        # 上下文管理
    "//autolink/croutine:croutine",      # 协程
    "//autolink/data:data",              # 数据管理
    "//autolink/event:event",            # 事件系统
    "//autolink/logger:logger",          # 日志系统
    "//autolink/mainboard:mainboard",    # 主板模块
    "//autolink/message:message",        # 消息系统
    "//autolink/node:node",              # 节点功能
    "//autolink/parameter:parameter",    # 参数系统
    "//autolink/plugin_manager:plugin_manager", # 插件管理器
    "//autolink/profiler:profiler",      # 性能分析器
    "//autolink/scheduler:scheduler",    # 调度器
    "//autolink/service:service",        # 服务系统
    "//autolink/service_discovery:service_discovery", # 服务发现
    "//autolink/sysmo:sysmo",            # 系统监控
    "//autolink/task:task",              # 任务管理
    "//autolink/time:time",              # 时间管理
    "//autolink/timer:timer",            # 定时器
    "//autolink/transport:transport",    # 传输层
    "//autolink/proto:proto_cc",         # Proto C++ 代码
]
```

**注意**：由于项目使用统一的 `autolink` 库，建议直接使用 `//autolink:autolink` 而不是单独引用各个子模块。

## 使用示例

### 基本使用

```cpp
#include "autolink/common/init.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/time.hpp"

int main(int argc, char* argv[]) {
    // 初始化 autolink
    if (!autolink::Init(argv[0])) {
        AERROR << "Failed to initialize autolink";
        return 1;
    }
    
    // 使用时间功能
    auto now = autolink::Time::Now();
    AINFO << "Current time: " << now;
    
    // 清理
    autolink::Clear();
    return 0;
}
```

### 节点与发布/订阅

创建一个节点并使用 `Writer`/`Reader` 进行发布/订阅通信：

```cpp
#include "autolink/common/init.hpp"
#include "autolink/autolink.hpp"
#include "autolink/examples/proto/examples.pb.h"

using autolink::examples::proto::Chatter;

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);
    
    // 创建节点
    auto node = autolink::CreateNode("talker", "");
    
    // 创建 Writer（发布者）
    auto writer = node->CreateWriter<Chatter>("/chatter");
    
    // 创建 Reader（订阅者）
    auto reader = node->CreateReader<Chatter>(
        "/chatter",
        [](const std::shared_ptr<Chatter>& msg) {
            AINFO << "Received: " << msg->message();
        });
    
    // 发布消息
    Chatter msg;
    msg.set_message("hello autolink");
    msg.set_sequence(1);
    writer->Write(msg);
    
    // 等待消息处理
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    autolink::Clear();
    return 0;
}
```

### 服务与客户端

使用 `Service` 和 `Client` 实现请求/响应模式：

```cpp
#include "autolink/common/init.hpp"
#include "autolink/autolink.hpp"
#include "autolink/examples/proto/examples.pb.h"

using autolink::examples::proto::Chatter;

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);
    
    auto node = autolink::CreateNode("service_node", "");
    
    // 创建 Service（服务端）
    auto service = node->CreateService<Chatter, Chatter>(
        "example_service",
        [](const std::shared_ptr<Chatter>& req, std::shared_ptr<Chatter>& res) {
            // 处理请求并生成响应
            res->set_message("Response: " + req->message());
            res->set_sequence(req->sequence() + 1000);
        });
    
    // 创建 Client（客户端）
    auto client = node->CreateClient<Chatter, Chatter>("example_service");
    
    // 发送请求
    auto request = std::make_shared<Chatter>();
    request->set_message("Hello Service");
    request->set_sequence(1);
    
    auto response = client->SendRequest(request);
    if (response) {
        AINFO << "Response: " << response->message();
    }
    
    autolink::Clear();
    return 0;
}
```

### 使用定时器

```cpp
#include "autolink/timer/timer.hpp"
#include "autolink/common/init.hpp"
#include "autolink/common/log.hpp"

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);
    
    // 创建一次性定时器
    autolink::Timer timer(1000, []() {
        AINFO << "Timer fired!";
    }, true);
    
    timer.Start();
    
    // 等待定时器触发
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    timer.Stop();
    autolink::Clear();
    return 0;
}
```

### 使用异步任务

```cpp
#include "autolink/task/task.hpp"
#include "autolink/common/init.hpp"
#include "autolink/common/log.hpp"

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);
    
    // 执行异步任务
    auto future = autolink::Async([]() {
        AINFO << "Running async task";
        return 42;
    });
    
    int result = future.get();
    AINFO << "Result: " << result;
    
    autolink::Clear();
    return 0;
}
```

### 使用 Proto

```cpp
#include "autolink/examples/proto/examples.pb.h"
#include "autolink/common/init.hpp"

using autolink::examples::proto::Chatter;

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);
    
    // 创建 proto 消息
    Chatter chatter;
    chatter.set_message("Hello");
    chatter.set_sequence(1);
    
    autolink::Clear();
    return 0;
}
```

**注意**：具体类型 `Req/Res` 与传输后端、QoS 等需参考 `autolink/proto` 与 `autolink/transport` 配置。

## 构建脚本使用

`scripts/autolink.py` 提供了统一的构建接口，支持自动平台检测：

```bash
# 查看帮助
python3 scripts/autolink.py --help

# 查看特定命令的帮助
python3 scripts/autolink.py build --help
python3 scripts/autolink.py test --help
python3 scripts/autolink.py install --help
```

### 构建脚本功能

- **build** - 构建目标（自动应用平台配置，支持 `--debug` 调试模式）
- **test** - 运行测试（自动应用平台配置，支持 `--debug` 调试模式）
- **run** - 运行可执行文件（自动应用平台配置，支持 `--debug` 调试模式）
- **install** - 安装构建产物（库、头文件、文档、CMake 配置）
- **clean** - 清理构建产物
- **query** - 查询 Bazel 目标
- **format** - 格式化 C/C++ 源代码（使用 clang-format）

所有命令都支持 `--debug` 或 `-d` 参数来启用调试模式构建。

### 代码格式化

使用 `format` 命令可以自动格式化项目中的 C/C++ 源代码：

```bash
# 格式化所有 C/C++ 源文件
python3 scripts/autolink.py format

# 查看将被格式化的文件列表（不实际格式化）
python3 scripts/autolink.py format --dry-run
```

**注意**：需要先安装 `clang-format` 工具：
- macOS: `brew install clang-format`
- Linux: `sudo apt-get install clang-format` 或 `sudo yum install clang-tools-extra`

格式化工具会自动查找 `clang-format`、`clang-format-17`、`clang-format-16`、`clang-format-15`、`clang-format-14` 等版本。在 macOS 上，会自动使用 Apple Command Line Tools 提供的 `clang-format`。

## 配置

### 运行时配置

- 运行时配置文件位于 `autolink/conf/` 目录
- 默认配置文件：`autolink/conf/autolink.pb.conf`
- QoS 和传输模式可在代码中通过 `RoleAttributes.qos_profile` 和 `transport::OptionalMode` 指定

### 日志配置

Autolink 使用 Google Logging (glog) 库进行日志输出。可以通过环境变量控制日志行为：

```bash
# 将日志输出到标准错误输出
export GLOG_logtostderr=1

# 不同时输出到日志文件（设置为 0）
export GLOG_alsologtostderr=0

# 在标准错误输出中使用颜色
export GLOG_colorlogtostderr=1

# 设置最小日志级别（0=INFO, 1=WARNING, 2=ERROR, 3=FATAL）
export GLOG_minloglevel=0
```

**日志级别说明：**
- `0` (INFO): 输出所有级别的日志（INFO、WARNING、ERROR、FATAL）
- `1` (WARNING): 只输出 WARNING、ERROR、FATAL
- `2` (ERROR): 只输出 ERROR、FATAL
- `3` (FATAL): 只输出 FATAL

**常用配置组合：**

```bash
# 开发调试模式（彩色输出，显示所有日志）
export GLOG_logtostderr=1
export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0

# 生产模式（只显示警告和错误）
export GLOG_logtostderr=1
export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=0
export GLOG_minloglevel=1
```

**注意：** 这些环境变量需要在运行程序之前设置，或者在运行命令中直接指定：

```bash
GLOG_logtostderr=1 GLOG_minloglevel=0 ./bin/autolink.examples.talker
```

### 传输模式

Autolink 支持多种传输模式：

- **INTRA**: 进程内通信（内存队列）
- **SHM**: 共享内存通信（跨进程）
- **RTPS**: RTPS 网络通信（跨机器）
- **HYBRID**: 混合模式（自动选择最优传输方式）

## 在其他项目中使用 Autolink

### CMake 项目

Autolink 提供了 CMake 支持，可以在 CMake 项目中使用：

```cmake
# 添加 FindAutolink.cmake 路径
list(APPEND CMAKE_MODULE_PATH "/path/to/autolink/cmake")

# 或设置 CMAKE_PREFIX_PATH（如果已安装）
set(CMAKE_PREFIX_PATH "/usr/local")

# 查找 Autolink
find_package(Autolink REQUIRED)

# 链接到你的目标
target_link_libraries(your_target PRIVATE Autolink::Autolink)
```

详细使用方法请参考 [CMake 使用文档](docs/cmake-usage.md)。

### 示例项目

查看 `examples/cmake_example/` 目录获取完整的 CMake 使用示例。

## 开发指南

### 代码风格

- 遵循项目内已有的代码风格
- 使用 `clang-format` 格式化代码（运行 `python3 scripts/autolink.py format`）
- 代码格式配置文件：`.clang-format`

### 提交规范

- 建议以小步提交（small commits）方式进行
- 每个功能或修复应附带必要的单元测试
- 提交信息应清晰描述变更内容

### 测试

- 编写单元测试时使用 `autolink_test()` 宏
- 测试文件命名规范：`*_test.cpp`
- 运行测试：`python3 scripts/autolink.py test` 或 `bazel test //...`

## 贡献

欢迎通过 Pull Request 参与贡献：

1. Fork 本仓库
2. 创建功能分支（建议使用 `feat/*`、`fix/*` 等命名规范）
3. 提交变更并确保测试通过
4. 发起 Pull Request

### 贡献指南

- 确保代码通过所有测试
- 添加必要的文档和注释
- 遵循项目的代码风格和规范
- 在 PR 中清晰描述变更内容和原因

## 更多信息

- [Bazel 官方文档](https://bazel.build/docs)
- [Bazel bzlmod 文档](https://bazel.build/docs/bzlmod)
- [Protocol Buffers 文档](https://protobuf.dev/)
- [CMake 使用文档](docs/cmake-usage.md)

---

## English / 英文

### Autolink

A lightweight, high-performance framework for distributed communication and task scheduling, inspired by Apollo Cyber RT. Autolink provides the `Node` abstraction, Channel-based pub/sub (`Writer`/`Reader`), Service/Client RPC, service discovery, timers and scheduling, logging, and more—ideal for robotics, middleware, and edge computing scenarios.

### Features

- **Node & Pub/Sub**: Create `Writer`/`Reader` via `Node` with QoS configurations and pluggable transport backends
- **Service & Client**: Request/Response model with `Service<Request, Response>` and `Client<Request, Response>`
- **Transport**: In-memory queue, SHM, RTPS and others (selected by config and build options)
- **Service Discovery**: Topology management and role registration tracking nodes/channels/services
- **Scheduler & Coroutines**: Classic/Choreography policies and coroutine support
- **Logging & Metrics**: Unified logging and performance statistics

### Quick Start

```cpp
#include "autolink/common/init.hpp"
#include "autolink/autolink.hpp"
#include "autolink/examples/proto/examples.pb.h"

using autolink::examples::proto::Chatter;

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);
    
    // Create a node
    auto node = autolink::CreateNode("talker", "");
    
    // Create Writer
    auto writer = node->CreateWriter<Chatter>("/chatter");
    
    // Create Reader
    auto reader = node->CreateReader<Chatter>(
        "/chatter",
        [](const std::shared_ptr<Chatter>& msg) {
            // handle message
        });
    
    // Publish
    Chatter msg;
    msg.set_message("hello autolink");
    writer->Write(msg);
    
    autolink::Clear();
    return 0;
}
```

### Build

This project uses Bazel build system. See the Chinese documentation above for detailed build instructions.

**Quick commands:**
```bash
# Build all targets
python3 scripts/autolink.py build

# Run tests
python3 scripts/autolink.py test

# Format code
python3 scripts/autolink.py format
```

### Configuration

- Runtime configurations: `autolink/conf/`
- QoS and transport mode: Set via `RoleAttributes.qos_profile` and `transport::OptionalMode`

### Contributing

Contributions are welcome via Pull Requests:
1. Fork this repository
2. Create a feature branch (e.g., `feat/*`, `fix/*`)
3. Commit your changes and open a PR

For more details, see the Chinese documentation above.

## 许可证

Copyright 2025 The Openbot Authors

Licensed under the Apache License, Version 2.0
