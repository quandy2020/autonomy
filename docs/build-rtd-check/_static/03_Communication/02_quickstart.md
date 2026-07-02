# 2. 快速开始

### 2.1 三步启用

1. 在进程入口调用 `autolink::Init(argv[0])`
2. 使用 `autolink::CreateNode` 创建节点
3. 通过 `CreateWriter` / `CreateReader` 建立 Channel 通信

### 2.2 最小 Talker / Listener

**Talker（发布者）**

```cpp
#include "autolink/autolink.hpp"
#include "autolink/time/rate.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);

    auto node = autolink::CreateNode("talker");
    auto writer = node->CreateWriter<autonomy::commsgs::sensor_msgs::Imu>("/sensor/imu");

    autolink::Rate rate(10.0);  // 10 Hz
    while (autolink::OK()) {
        auto msg = std::make_shared<autonomy::commsgs::sensor_msgs::Imu>();
        msg->header.stamp = autolink::Time::Now();
        msg->header.frame_id = "imu_link";
        writer->Write(msg);
        rate.Sleep();
    }

    autolink::WaitForShutdown();
    return 0;
}
```

**Listener（订阅者）**

```cpp
#include "autolink/autolink.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);

    auto node = autolink::CreateNode("listener");
    auto reader = node->CreateReader<autonomy::commsgs::sensor_msgs::Imu>(
        "/sensor/imu",
        [](const std::shared_ptr<autonomy::commsgs::sensor_msgs::Imu>& msg) {
            AINFO << "IMU frame=" << msg->header.frame_id;
        });

    autolink::WaitForShutdown();
    return 0;
}
```

### 2.3 编译与运行

```bash
# 在 Autonomy 工程内（CMake）
cd build && cmake .. && make talker listener

# 两个终端分别运行
./bin/talker
./bin/listener
```

> 也可参考 `autolink/examples/cpp/` 与 `autolink/examples/python/` 中的官方示例。

### 2.4 使用 mainboard + DAG 启动组件

适合将算法封装为 `Component` 并由 `mainboard` 统一加载：

**1. 实现组件**

```cpp
#include "autolink/component/component.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"

class ImuRelayComponent : public autolink::Component<
    autonomy::commsgs::sensor_msgs::Imu> {
public:
    bool Init() override { return true; }

    bool Proc(const std::shared_ptr<autonomy::commsgs::sensor_msgs::Imu>& msg)
        override {
        // 处理并转发
        return true;
    }
};
AUTOLINK_REGISTER_COMPONENT(ImuRelayComponent)
```

**2. DAG 文件**（`imu_relay.dag`）

```protobuf
module_config {
  module_library : "libimu_relay_component.so"
  components {
    class_name : "ImuRelayComponent"
    config {
      name : "imu_relay"
      readers {
        channel: "/sensor/imu"
        qos_profile: { depth: 10 }
      }
    }
  }
}
```

**3. Launch 文件**（`imu.launch`）

```xml
<autolink>
  <module>
    <name>imu_relay</name>
    <dag_conf>imu_relay.dag</dag_conf>
    <process_name>imu_relay</process_name>
  </module>
</autolink>
```

**4. 启动**

```bash
mainboard -d imu_relay.dag
# 或
autolink_launch imu.launch
```

### 2.5 环境准备

| 步骤 | 命令 / 说明 |
|------|-------------|
| 子模块 | `git clone --recurse-submodules` 确保 `autolink/` 存在 |
| 依赖 | protobuf、glog、FastDDS（RTPS 可选） |
| 环境变量 | `source autolink/autolink/setup.bash` |
| Docker | 参见 [04 Running · Docker 运行时](../04_Running/05_docker_runtime.md) |

### 2.6 下一步

| 目标 | 文档 |
|------|------|
| 理解通信模型 | [§3 概念](03_concepts.md) |
| API 详解 | [§4 使用指南](04_usage.md) |
| 架构深入 | [§5 架构设计](05_architecture.md) |
| 消息类型 | [commsgs §2](../14_Commsgs/02_quickstart.md) |
