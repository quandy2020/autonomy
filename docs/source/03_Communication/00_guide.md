(communication-guide)=
# 0. Communication 通信框架指南

`autolink` 是 Autonomy 的 **分布式通信运行时**，对标 Apollo Cyber RT / ROS 2 DDS 层。

| 本文 §0 | 相关文档 |
|---------|----------|
| 上手、环境、工具 | [§1 架构 Architecture](01_architecture.md) · [§13 综述 Survey](13_survey.md) |

<div class="nav-costmap-banner">
  <strong>autolink 在 Autonomy 栈中的位置</strong>
  <span class="nav-costmap-detail">算法 struct → commsgs ToProto → Writer/Reader → 跨进程</span>
  <span class="nav-costmap-arrow">架构分层与数据流见 §1 →</span>
</div>

---

## 0.1 文档地图

### 按角色阅读

| 角色 | 阅读顺序 |
|------|----------|
| 新手 | §0.2 → [§2 节点 Node](02_node.md) → [§3 通道 Channel](03_channel.md) |
| 写 Component | [§10 组件 Component](10_component.md) → [§9 启动 Launch](09_launch.md) → [§12 调度 Scheduler](12_scheduler.md) |
| 写 Action / Plugin | [§5 动作 Action](05_action.md) · [§7 插件 Plugin](07_plugin.md) |
| 选型 / 对比 | [§13 综述 Survey](13_survey.md) |

### 章节目录

| § | 文档 | 内容 |
|---|------|------|
| 0 | 本指南 | 快速开始、环境变量、CLI、示例索引 |
| 1 | [架构 Architecture](01_architecture.md) | 分层、数据流、生命周期、环境变量详表 |
| 2 | [节点 Node](02_node.md) | 通信句柄、生命周期 |
| 3 | [通道 Channel](03_channel.md) | Writer/Reader、QoS、POD、录包 |
| 4 | [服务 Service](04_service.md) | RPC 请求–响应 |
| 5 | [动作 Action](05_action.md) | 长任务 Goal/Feedback/Result |
| 6 | [参数 Parameter](06_parameter.md) | 全局键值参数 |
| 7 | [插件 Plugin](07_plugin.md) | 可插拔算法、描述文件 |
| 8 | [日志 Log](08_log.md) | AsyncLogger、宏、GLOG 配置、使用案例 |
| 9 | [启动 Launch](09_launch.md) | DAG 配置、多进程编排、部署路径 |
| 10 | [组件 Component](10_component.md) | 多输入 `Proc`、DataVisitor |
| 11 | [时间 Time / Rate / Timer](11_timer.md) | Time/Rate/Clock、Timer、TimerComponent |
| 12 | [调度 Scheduler](12_scheduler.md) | classic / choreography |
| 13 | [综述 Survey](13_survey.md) | ROS2/Cyber 对比、模式选型 |

---

## 0.2 快速开始

```cpp
#include "autolink/autolink.hpp"

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);
    auto node = autolink::CreateNode("talker");
    auto writer = node->CreateWriter<MyMsg>("/channel/name");
    auto reader = node->CreateReader<MyMsg>("/channel/name",
        [](const std::shared_ptr<MyMsg>& msg) { AINFO << "rx"; });
    autolink::WaitForShutdown();
}
```

**三步环境**：

```bash
git clone --recurse-submodules <repo>
source autolink/autolink/setup.bash          # 设置 PATH、AUTOLINK_PATH 等
export AUTOLINK_PATH=<含 conf/autolink.pb.conf 的目录>
```

**编译示例**：

```bash
cmake -S autolink -B build/autolink \
  -DAUTOLINK_BUILD_EXAMPLES=ON -DAUTOLINK_BUILD_TOOLS=ON
cmake --build build/autolink -j8
```

详见 `autolink/examples/cpp/README.md`、`autolink/examples/python/README.md`。

---

## 0.3 全局约束

| 约束 | 说明 |
|------|------|
| 先 `Init()` | 未初始化时 `CreateNode` 返回 null |
| `node_name` 唯一 | 拓扑内不可重复；多实例加 PID 后缀 |
| channel 名完全一致 | 含 `/` 前缀习惯须两端相同 |
| 消息类型一致 | 同 protobuf 或 POD 包装类型 |
| 回调勿阻塞 | Reader / `Proc` 内避免长 I/O；重逻辑移交工作线程 |
| `AUTOLINK_PATH` 一致 | 同机多进程须指向同一配置根 |

---

## 0.4 核心模块速览

| autolink | ROS 2 | 文档 |
|----------|-------|------|
| Node | Node | [§2](02_node.md) |
| Channel + Writer/Reader | topic + pub/sub | [§3](03_channel.md) |
| Service/Client | Service | [§4](04_service.md) |
| Action | Action | [§5](05_action.md) |
| Parameter | Parameter | [§6](06_parameter.md) |
| PluginManager | pluginlib | [§7](07_plugin.md) |
| AINFO / glog | rclcpp logging | [§8](08_log.md) |
| Launch (mainboard) | `ros2 launch` | [§9 启动 Launch](09_launch.md) |
| Component + DAG | — | [§10](10_component.md) |
| Time / Rate / Timer | Timer / Clock | [§11](11_timer.md) |
| Scheduler | Executor | [§12](12_scheduler.md) |

**消息**：算法内 `autonomy::commsgs::*` struct，边界 `ToProto()`。详见 [commsgs](../14_Commsgs/03_schema.md)。

---

## 0.5 环境变量（常用）

完整说明见 [§1.6 环境与路径变量](01_architecture.md#16-环境与路径变量)。

| 变量 | 用途 |
|------|------|
| `AUTOLINK_PATH` | 运行时根目录（须含 `conf/autolink.pb.conf`） |
| `AUTOLINK_DISTRIBUTION_HOME` | 安装前缀；Python 绑定路径 |
| `AUTOLINK_DAG_PATH` | `.dag` 搜索路径 |
| `AUTOLINK_LIB_PATH` | Component `.so` 搜索路径 |
| `AUTOLINK_LAUNCH_PATH` | `.launch` 搜索路径 |
| `AUTOLINK_SCHED_CONF` | 调度配置文件路径 |
| `AUTOLINK_PLUGIN_*` | 插件描述与 `.so` 路径（见 [§7](07_plugin.md)） |
| `AUTOLINK_DOMAIN_ID` | 跨主机发现域 ID（默认见 `setup.bash`） |
| `GLOG_*` | 日志级别与输出目录（见 [§8](08_log.md)） |

---

## 0.6 CLI 工具

构建时加 `-DAUTOLINK_BUILD_TOOLS=ON`（`autolink/tools/`）：

| 工具 | 用途 |
|------|------|
| `mainboard` | 加载 DAG、启动 Component |
| `autolink_launch` | 解析 `.launch`、编排多进程 |
| `autolink_channel` | 列出 channel 与类型 |
| `autolink_node` | 查看节点信息 |
| `autolink_service` / `autolink_action` | 调试 RPC / Action |
| `autolink_monitor` | 实时消息监控 |
| `autolink_recorder` | 录包 / 回放 |

录包示例：`autolink_recorder record -a -o x.record` · `play -f x.record`（源码 `examples/cpp/record.cpp`）。

---

## 0.7 示例索引

路径均相对于 `autolink/examples/`。

| 场景 | 可执行文件 / 脚本 | 源码 |
|------|-------------------|------|
| Talker / Listener | `autolink_example_talker` + `listener` | `cpp/talker.cpp` · `cpp/listener.cpp` |
| 单进程 POD | `autolink_example_pod_talker_listener` | `cpp/pod_talker_listener.cpp` |
| Service | `autolink_example_service` | `cpp/service.cpp` |
| Action | `action_listener` + `action_talker` | `cpp/action_*.cpp` |
| Parameter | `autolink_example_paramserver` | `cpp/paramserver.cpp` |
| Record | `autolink_example_record` | `cpp/record.cpp` |
| Component | `mainboard -d .../common.dag` | `cpp/common_component_example/` |
| TimerComponent | `mainboard -d .../timer.dag` | `cpp/timer_component_example/` |
| Python pub/sub | `py_talker.py` + `py_listener.py` | `python/` |
| Python service | `py_service.py` + `py_client.py` | `python/` |
| Python Time/Rate | `py_time.py` | `python/py_time.py` |
| Python Timer | `py_timer.py` | `python/py_timer.py` |

---

**导航**：[§1 架构 Architecture →](01_architecture.md) · [§13 综述 Survey →](13_survey.md)

(communication-usage)=
