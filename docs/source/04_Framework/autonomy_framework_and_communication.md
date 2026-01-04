# DDS 通信

## 1. 简介

Autolink 提供了基于 DDS（Data Distribution Service）的分布式通信框架，支持以下通信模式：

* **Topic 通信**：基于发布/订阅模式的点对点消息通信（Writer/Reader）
* **Service/Client**：基于请求/响应模式的 RPC 通信

Autolink 框架封装了底层的 DDS 实现（如 FastDDS），提供了简洁易用的 API。框架会自动处理服务发现、消息序列化、传输等底层细节。

## 2. 使用方式

### 2.1 初始化框架

在使用 Autolink 之前，需要先初始化框架：

```cpp
#include "autolink/common/init.hpp"

int main(int argc, char* argv[]) {
    // 初始化 autolink 框架
    autolink::Init(argv[0]);
    
    // ... 使用 autolink API ...
    
    // 清理资源（可选，程序退出时自动清理）
    autolink::WaitForShutdown();
    return 0;
}
```

### 2.2 创建 Node

Node 是 Autolink 中的基本构建单元，所有通信功能都通过 Node 来创建。

```cpp
#include "autolink/autolink.hpp"

// 使用 autolink 命名空间
using namespace autolink;

// 创建 Node
auto node = CreateNode("my_node_name", "");  // 第二个参数是命名空间，可以为空
```

**[说明]**

* **Node**：类型为 `std::unique_ptr<autolink::Node>`
* **node_name**：节点的唯一名称（字符串类型），在同一个进程中不能重复
* **name_space**：可选的命名空间（字符串类型），默认为空字符串

**注意**：如果 `autolink::Init()` 未被调用，`CreateNode` 可能返回 `nullptr`，使用时需要检查返回值。

### 2.3 创建 Publisher（Writer）

Writer 用于向 Channel 发布消息。

```cpp
#include "autolink/autolink.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"

auto node = CreateNode("publisher_node");

// 创建 Writer
auto writer = node->CreateWriter<autonomy::commsgs::sensor_msgs::Imu>("/sensor/imu");

// 发布消息（方式1：使用对象）
autonomy::commsgs::sensor_msgs::Imu imu;
imu.header.frame_id = "imu_frame";
imu.header.stamp.sec = 1234567890;
imu.header.stamp.nanosec = 123456789;
imu.linear_acceleration.x = 1.0;
imu.linear_acceleration.y = 2.0;
imu.linear_acceleration.z = 3.0;
imu.angular_velocity.x = 4.0;
imu.angular_velocity.y = 5.0;
imu.angular_velocity.z = 6.0;
writer->Write(imu);

// 发布消息（方式2：使用 shared_ptr）
auto imu_msg = std::make_shared<autonomy::commsgs::sensor_msgs::Imu>();
imu_msg->header.frame_id = "imu_frame";
imu_msg->header.stamp = autolink::Time::Now();
imu_msg->linear_acceleration.x = 1.0;
writer->Write(imu_msg);
```

**[说明]**

* **MessageT**：消息类型，例如 `autonomy::commsgs::sensor_msgs::Imu`
* **channel_name**：Channel 名称（字符串类型），相同名称的 Writer 和 Reader 会自动建立连接
* **Write()**：发布消息的方法，支持对象和 `shared_ptr` 两种形式
* **返回值**：`std::shared_ptr<Writer<MessageT>>`，如果创建失败返回 `nullptr`

**提示**：可以使用 `writer->HasReader()` 来检查是否有 Reader 订阅了该 Channel。

### 2.4 创建 Subscription（Reader）

Reader 用于从 Channel 订阅消息。

```cpp
#include "autolink/autolink.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"

auto node = CreateNode("subscription_node");

// 定义回调函数
auto callback = [](const std::shared_ptr<autonomy::commsgs::sensor_msgs::Imu>& msg) {
    AINFO << "Received IMU: frame_id=" << msg->header.frame_id
          << ", angular_velocity x=" << msg->angular_velocity.x
          << ", y=" << msg->angular_velocity.y
          << ", z=" << msg->angular_velocity.z
          << ", linear_acceleration x=" << msg->linear_acceleration.x
          << ", y=" << msg->linear_acceleration.y
          << ", z=" << msg->linear_acceleration.z;
};

// 创建 Reader（方式1：使用 channel_name）
auto reader = node->CreateReader<autonomy::commsgs::sensor_msgs::Imu>(
    "/sensor/imu", callback);

// 创建 Reader（方式2：使用 ReaderConfig，可配置更多选项）
autolink::ReaderConfig config;
config.channel_name = "/sensor/imu";
config.pending_queue_size = 10;  // 消息缓存队列大小
auto reader2 = node->CreateReader<autonomy::commsgs::sensor_msgs::Imu>(config, callback);
```

**[说明]**

* **MessageT**：消息类型，必须与 Writer 发布的消息类型一致
* **channel_name**：Channel 名称，必须与 Writer 使用的 Channel 名称相同
* **callback**：回调函数，类型为 `std::function<void(const std::shared_ptr<MessageT>&)>`
* **pending_queue_size**：消息缓存队列大小，默认为 1。如果消息处理速度慢于接收速度，旧消息可能会被丢弃。增大此值可以缓存更多消息
* **返回值**：`std::shared_ptr<Reader<MessageT>>`，如果创建失败返回 `nullptr`

**注意**：Reader 必须绑定回调函数才能接收消息。如果没有提供回调，可以通过 `node->Observe()` 方法手动轮询消息。

### 2.5 创建 Service

Service 用于提供 RPC 服务，处理客户端的请求并返回响应。

```cpp
#include "autolink/autolink.hpp"
#include "autonomy/commsgs/example_msgs.hpp"  // 假设有 Request 和 Response 消息类型

auto node = CreateNode("service_node");

// 定义服务回调函数
auto service_callback = [](
    const std::shared_ptr<autonomy::commsgs::example_msgs::Request>& request,
    std::shared_ptr<autonomy::commsgs::example_msgs::Response>& response) {
    
    AINFO << "Service received request: " << request->data();
    
    // 处理请求并设置响应
    response = std::make_shared<autonomy::commsgs::example_msgs::Response>();
    response->set_result("Processed: " + request->data());
    response->set_status_code(200);
};

// 创建 Service
auto service = node->CreateService<autonomy::commsgs::example_msgs::Request,
                                    autonomy::commsgs::example_msgs::Response>(
    "/my_service", service_callback);

if (!service) {
    AERROR << "Failed to create service";
    return -1;
}
```

**[说明]**

* **Request**：请求消息类型
* **Response**：响应消息类型
* **service_name**：服务名称（字符串类型），用于标识服务
* **service_callback**：服务回调函数，类型为 `std::function<void(const std::shared_ptr<Request>&, std::shared_ptr<Response>&)>`
  - 第一个参数：接收到的请求（只读）
  - 第二个参数：需要设置的响应（可修改）
* **返回值**：`std::shared_ptr<Service<Request, Response>>`，如果创建失败返回 `nullptr`

### 2.6 创建 Client

Client 用于调用 Service，发送请求并接收响应。

```cpp
#include "autolink/autolink.hpp"
#include "autonomy/commsgs/example_msgs.hpp"

auto node = CreateNode("client_node");

// 创建 Client
auto client = node->CreateClient<autonomy::commsgs::example_msgs::Request,
                                  autonomy::commsgs::example_msgs::Response>(
    "/my_service");

if (!client) {
    AERROR << "Failed to create client";
    return -1;
}

// 等待服务可用（可选）
if (!client->WaitForService(std::chrono::seconds(5))) {
    AERROR << "Service not available";
    return -1;
}

// 发送同步请求
auto request = std::make_shared<autonomy::commsgs::example_msgs::Request>();
request->set_data("Hello Service");

auto response = client->SendRequest(request, std::chrono::seconds(5));
if (response) {
    AINFO << "Response received: " << response->result();
    AINFO << "Status code: " << response->status_code();
} else {
    AWARN << "Request timeout or failed";
}

// 发送异步请求
auto future = client->AsyncSendRequest(request);
// 可以继续做其他工作...
auto response2 = future.get();  // 获取结果
```

**[说明]**

* **Request**：请求消息类型，必须与 Service 的 Request 类型一致
* **Response**：响应消息类型，必须与 Service 的 Response 类型一致
* **service_name**：服务名称，必须与 Service 使用的名称相同
* **SendRequest()**：同步发送请求，会阻塞直到收到响应或超时
  - 参数1：请求消息（`shared_ptr<Request>` 或 `Request` 对象）
  - 参数2：超时时间（`std::chrono::seconds`，默认 5 秒）
  - 返回值：`shared_ptr<Response>`，如果失败或超时返回 `nullptr`
* **AsyncSendRequest()**：异步发送请求，立即返回 `std::shared_future<shared_ptr<Response>>`
* **WaitForService()**：等待服务可用，可以设置超时时间
* **ServiceIsReady()**：检查服务是否可用（非阻塞）

## 3. 使用案例

### 3.1 发布地图并订阅

以下示例展示了如何创建 Publisher 发布地图消息：

```cpp
#include "autolink/autolink.hpp"
#include "autonomy/commsgs/map_msgs.hpp"

auto node = CreateNode("map_publisher_node");

// 创建地图发布者
auto map_publisher = node->CreateWriter<autonomy::commsgs::map_msgs::OccupancyGrid>("/map");

// 发布地图
autonomy::commsgs::map_msgs::OccupancyGrid occupancy_grid;
occupancy_grid.header.frame_id = "map";
occupancy_grid.header.stamp = autolink::Time::Now();
occupancy_grid.info.resolution = 0.05;
occupancy_grid.info.width = 100;
occupancy_grid.info.height = 100;
// ... 设置地图数据 ...
map_publisher->Write(occupancy_grid);
```

订阅地图的示例：

```cpp
auto node = CreateNode("map_subscriber_node");

auto map_callback = [](
    const std::shared_ptr<autonomy::commsgs::map_msgs::OccupancyGrid>& map) {
    AINFO << "Received map: width=" << map->info.width
          << ", height=" << map->info.height
          << ", resolution=" << map->info.resolution;
};

auto map_reader = node->CreateReader<autonomy::commsgs::map_msgs::OccupancyGrid>(
    "/map", map_callback);
```

### 3.2 完整的 Talker-Listener 示例

**Talker（发布者）：**

```cpp
#include "autolink/autolink.hpp"
#include "autolink/time/time.hpp"
#include "autolink/time/rate.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);
    
    auto node = CreateNode("talker");
    auto writer = node->CreateWriter<autonomy::commsgs::sensor_msgs::Imu>("/sensor/imu");
    
    autolink::Rate rate(10.0);  // 10 Hz
    
    int seq = 0;
    while (autolink::OK()) {
        auto msg = std::make_shared<autonomy::commsgs::sensor_msgs::Imu>();
        msg->header.stamp = autolink::Time::Now();
        msg->header.frame_id = "imu_frame";
        msg->linear_acceleration.x = 1.0;
        msg->linear_acceleration.y = 2.0;
        msg->linear_acceleration.z = 3.0;
        msg->angular_velocity.x = 0.1;
        msg->angular_velocity.y = 0.2;
        msg->angular_velocity.z = 0.3;
        
        writer->Write(msg);
        AINFO << "Published message seq=" << seq++;
        
        rate.Sleep();
    }
    
    autolink::WaitForShutdown();
    return 0;
}
```

**Listener（订阅者）：**

```cpp
#include "autolink/autolink.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"

void MessageCallback(
    const std::shared_ptr<autonomy::commsgs::sensor_msgs::Imu>& msg) {
    AINFO << "Received IMU: frame_id=" << msg->header.frame_id
          << ", linear_acceleration=(" 
          << msg->linear_acceleration.x << ", "
          << msg->linear_acceleration.y << ", "
          << msg->linear_acceleration.z << ")"
          << ", angular_velocity=("
          << msg->angular_velocity.x << ", "
          << msg->angular_velocity.y << ", "
          << msg->angular_velocity.z << ")";
}

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);
    
    auto node = CreateNode("listener");
    auto reader = node->CreateReader<autonomy::commsgs::sensor_msgs::Imu>(
        "/sensor/imu", MessageCallback);
    
    if (!reader) {
        AERROR << "Failed to create reader";
        return 1;
    }
    
    AINFO << "Listener started, waiting for messages...";
    autolink::WaitForShutdown();
    return 0;
}
```

### 3.3 Service/Client 示例

**Service（服务端）：**

```cpp
#include "autolink/autolink.hpp"
#include "autonomy/commsgs/example_msgs.hpp"

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);
    
    auto node = CreateNode("calculator_service");
    
    auto service = node->CreateService<autonomy::commsgs::example_msgs::CalcRequest,
                                        autonomy::commsgs::example_msgs::CalcResponse>(
        "/calculator",
        [](const std::shared_ptr<autonomy::commsgs::example_msgs::CalcRequest>& req,
           std::shared_ptr<autonomy::commsgs::example_msgs::CalcResponse>& resp) {
            resp = std::make_shared<autonomy::commsgs::example_msgs::CalcResponse>();
            
            if (req->operation() == "add") {
                resp->set_result(req->a() + req->b());
            } else if (req->operation() == "multiply") {
                resp->set_result(req->a() * req->b());
            } else {
                resp->set_result(0);
                resp->set_error("Unknown operation");
            }
        });
    
    if (!service) {
        AERROR << "Failed to create service";
        return 1;
    }
    
    AINFO << "Service started";
    autolink::WaitForShutdown();
    return 0;
}
```

**Client（客户端）：**

```cpp
#include "autolink/autolink.hpp"
#include "autonomy/commsgs/example_msgs.hpp"

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);
    
    auto node = CreateNode("calculator_client");
    
    auto client = node->CreateClient<autonomy::commsgs::example_msgs::CalcRequest,
                                      autonomy::commsgs::example_msgs::CalcResponse>(
        "/calculator");
    
    if (!client->WaitForService(std::chrono::seconds(5))) {
        AERROR << "Service not available";
        return 1;
    }
    
    auto request = std::make_shared<autonomy::commsgs::example_msgs::CalcRequest>();
    request->set_a(10);
    request->set_b(20);
    request->set_operation("add");
    
    auto response = client->SendRequest(request);
    if (response) {
        AINFO << "Result: " << response->result();
    } else {
        AERROR << "Request failed";
    }
    
    return 0;
}
```

## 4. 注意事项

1. **初始化顺序**：在使用任何 Autolink API 之前，必须先调用 `autolink::Init()`
2. **节点名称唯一性**：同一进程中的节点名称必须唯一
3. **消息类型匹配**：Writer 和 Reader 的消息类型必须完全一致
4. **Channel 名称匹配**：发布者和订阅者必须使用相同的 Channel 名称
5. **Service 名称匹配**：Service 和 Client 必须使用相同的服务名称
6. **线程安全**：Autolink API 通常是线程安全的，但同一对象的并发调用需要注意
7. **资源清理**：Node、Writer、Reader 等对象使用智能指针管理，通常不需要手动释放

## 5. 高级功能

### 5.1 服务发现

Autolink 框架提供了自动服务发现功能。当创建 Writer/Reader 或 Service/Client 时，框架会自动注册到服务发现系统中，并建立连接。无需手动配置连接信息。

### 5.2 QoS 配置

可以通过 `RoleAttributes` 或 `ReaderConfig` 配置 QoS（Quality of Service）参数，包括：
- 消息队列大小
- 可靠性模式
- 历史记录策略
- 等等

### 5.3 传输后端

Autolink 支持多种传输后端：
- 内存队列（In-memory queue）
- 共享内存（SHM）
- RTPS/DDS
- 等等

传输后端的选择通常通过配置文件控制，对用户透明。
