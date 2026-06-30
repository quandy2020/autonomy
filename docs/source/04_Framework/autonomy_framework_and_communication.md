# DDS 通信

## 1. 简介

Autolink 提供了基于 DDS（Data Distribution Service）的分布式通信框架，支持以下通信模式：

* **Topic 通信**：基于发布/订阅模式的点对点消息通信（Writer/Reader）
* **Service/Client**：基于请求/响应模式的 RPC 通信
* **Action**：基于目标/反馈/结果的长时间运行任务通信机制

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

### 2.7 创建 Action Server

Action Server 用于处理长时间运行的任务，支持目标接受、取消和进度反馈。

```cpp
#include "autolink/autolink.hpp"
#include "autolink/action/action.hpp"
#include "autonomy/commsgs/example_msgs.hpp"  // 假设有 Action 类型定义

using ActionT = autonomy::commsgs::example_msgs::ExampleAction;
using Goal = ActionT::Goal;

auto node = CreateNode("action_server_node");

// 创建 Action Server
auto server = autolink::action::CreateServer<ActionT>(
    node,
    "/my_action",
    // handle_goal: 决定是否接受目标
    [](const autolink::action::GoalUUID& uuid,
       std::shared_ptr<const Goal> goal) {
        AINFO << "Goal received: " << autolink::action::ToString(uuid);
        // 返回 ACCEPT_AND_EXECUTE、ACCEPT_AND_DEFER 或 REJECT
        return autolink::action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // handle_cancel: 决定是否接受取消请求
    [](std::shared_ptr<autolink::action::ServerGoalHandle<ActionT>> goal_handle) {
        return autolink::action::CancelResponse::ACCEPT;
    },
    // handle_accepted: 目标被接受时调用，用于执行目标
    [](std::shared_ptr<autolink::action::ServerGoalHandle<ActionT>> goal_handle) {
        // 在新线程中执行目标（推荐做法）
        std::thread([goal_handle]() {
            goal_handle->Execute();  // 开始执行
            // ... 执行任务 ...
            // goal_handle->PublishFeedback(feedback);  // 发布反馈
            // goal_handle->Succeed(result);  // 标记成功
        }).detach();
    });
```

**[说明]**

* **ActionT**：Action 类型，需要定义 `Goal`、`Feedback`、`Result` 三个嵌套类型
* **action_name**：Action 名称（字符串类型），用于标识 Action
* **handle_goal**：目标接受回调，返回 `GoalResponse::ACCEPT_AND_EXECUTE`、`ACCEPT_AND_DEFER` 或 `REJECT`
* **handle_cancel**：取消请求回调，返回 `CancelResponse::ACCEPT` 或 `REJECT`
* **handle_accepted**：目标接受后的回调，通常在此启动新线程执行目标
* **返回值**：`std::shared_ptr<Server<ActionT>>`，如果创建失败返回 `nullptr`

**ServerGoalHandle 常用方法**：
- `GetGoal()`：获取目标消息
- `GetGoalId()`：获取目标 UUID
- `Execute()`：开始执行目标
- `PublishFeedback(feedback)`：发布反馈消息
- `Succeed(result)`：标记成功完成
- `Canceled(result)`：标记已被取消
- `Abort(result)`：标记失败中止
- `IsCanceling()`：检查是否收到取消请求

### 2.8 创建 Action Client

Action Client 用于发送目标并接收反馈和结果。

```cpp
#include "autolink/autolink.hpp"
#include "autolink/action/action.hpp"
#include "autonomy/commsgs/example_msgs.hpp"

using ActionT = autonomy::commsgs::example_msgs::ExampleAction;
using Goal = ActionT::Goal;
using Feedback = ActionT::Feedback;

auto node = CreateNode("action_client_node");

// 创建 Action Client
auto client = autolink::action::CreateClient<ActionT>(node, "/my_action");

if (!client) {
    AERROR << "Failed to create action client";
    return -1;
}

// 等待服务器就绪（可选）
while (!client->ActionServerIsReady()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// 创建目标
Goal goal;
goal.set_target_value(100);

// 配置发送选项
autolink::action::Client<ActionT>::SendGoalOptions options;
options.goal_response_callback = [](
    std::shared_ptr<autolink::action::ClientGoalHandle<ActionT>> handle) {
    if (handle) {
        AINFO << "Goal accepted";
    } else {
        AERROR << "Goal rejected";
    }
};
options.feedback_callback = [](
    std::shared_ptr<autolink::action::ClientGoalHandle<ActionT>> handle,
    std::shared_ptr<const Feedback> feedback) {
    AINFO << "Feedback: " << feedback->progress();
};
options.result_callback = [](
    const autolink::action::ClientGoalHandle<ActionT>::WrappedResult& result) {
    AINFO << "Result code: " << static_cast<int>(result.code);
};

// 异步发送目标
auto goal_future = client->AsyncSendGoal(goal, options);
auto goal_handle = goal_future.get();

if (goal_handle) {
    // 获取结果
    auto result_future = client->AsyncGetResult(goal_handle);
    auto wrapped_result = result_future.get();
    
    // 也可以取消目标
    // auto cancel_future = client->AsyncCancelGoal(goal_handle);
}
```

**[说明]**

* **ActionT**：Action 类型，必须与 Server 使用的类型一致
* **action_name**：Action 名称，必须与 Server 使用的名称相同
* **AsyncSendGoal()**：异步发送目标，返回 `std::shared_future<std::shared_ptr<GoalHandle>>`
* **AsyncGetResult()**：异步获取结果，返回 `std::shared_future<WrappedResult>`
* **AsyncCancelGoal()**：异步请求取消目标，返回 `std::shared_future<bool>`
* **ActionServerIsReady()**：检查 Action 服务器是否就绪
* **SendGoalOptions**：发送选项，可设置目标响应、反馈和结果回调

**ClientGoalHandle 常用方法**：
- `GetGoalId()`：获取目标 UUID
- `GetStatus()`：获取当前状态
- `AsyncGetResult()`：获取结果的 future

### 2.9 Action 类型定义（Proto3）

在使用 Action 之前，需要先在 `.proto` 文件中定义 Action 类型。Action 类型必须遵循以下规则：

**基本结构**：

```protobuf
syntax = "proto3";

package your.package.name;

// 可选：定义错误码枚举（推荐）
enum YourActionErrorCode {
    YOUR_ACTION_ERROR_NONE = 0;
    YOUR_ACTION_ERROR_UNKNOWN = 100;
    YOUR_ACTION_ERROR_TIMEOUT = 101;
    // ... 其他错误码
}

// Action 定义
message YourActionAction {
    // 必须定义三个嵌套 message
    
    message Goal {
        // 定义目标参数
        // 例如：目标位置、速度、超时时间等
        string target_name = 1;
        float target_value = 2;
        // ... 其他字段
    }

    message Feedback {
        // 定义反馈信息
        // 例如：当前进度、状态信息等
        float progress = 1;  // 进度百分比 (0.0-1.0)
        string status = 2;   // 状态描述
        // ... 其他字段
    }

    message Result {
        // 定义结果信息
        // 通常包含错误码和错误消息
        YourActionErrorCode error_code = 1;
        string error_msg = 2;
        // ... 其他结果字段
    }

    // Action message 本身必须包含这三个字段
    Goal goal = 1;
    Feedback feedback = 2;
    Result result = 3;
}
```

**定义规则**：

1. **语法版本**：必须使用 `syntax = "proto3";` 声明
2. **命名规范**：Action message 名称应以 `Action` 结尾，例如 `NavigateToPoseAction`、`ComputePathAction`
3. **嵌套 Message**：必须在 Action message 内部定义三个嵌套 message：
   - `Goal`：定义目标参数，客户端发送给服务器
   - `Feedback`：定义反馈信息，服务器在执行过程中发送给客户端
   - `Result`：定义结果信息，服务器在任务完成时发送给客户端
4. **字段定义**：Action message 本身必须包含三个字段（字段编号固定）：
   - `Goal goal = 1;`
   - `Feedback feedback = 2;`
   - `Result result = 3;`
5. **错误码枚举**（可选但推荐）：
   - 为每个 Action 定义错误码枚举，命名格式为 `XxxActionErrorCode`
   - 错误码枚举应在 Action message 之前定义
   - 必须包含 `ERROR_NONE = 0;` 表示成功
   - 其他错误码应从 100 开始编号
6. **字段编号**：所有字段必须使用明确的字段编号（`= 1`, `= 2`, ...），不能使用 `reserved` 或跳过编号
7. **字段类型**：可以使用任何 proto3 支持的类型（`string`, `int32`, `float`, `bool`, 自定义 message 等）
8. **导入依赖**：可以使用 `import` 导入其他 proto 文件中的类型

**完整示例**：

```protobuf
syntax = "proto3";

package autonomy.tasks.example.proto;

import "autonomy/commsgs/proto/geometry_msgs.proto";
import "autonomy/commsgs/proto/builtin_interfaces.proto";

// 错误码枚举
enum NavigateToPoseErrorCode {
    NAVIGATE_TO_POSE_ERROR_NONE = 0;
    NAVIGATE_TO_POSE_ERROR_UNKNOWN = 100;
    NAVIGATE_TO_POSE_ERROR_TIMEOUT = 101;
    NAVIGATE_TO_POSE_ERROR_TF_ERROR = 102;
    NAVIGATE_TO_POSE_ERROR_NO_PATH_FOUND = 103;
}

// Action 定义
message NavigateToPoseAction {
    message Goal {
        // 目标位置
        autonomy.commsgs.proto.geometry_msgs.PoseStamped pose = 1;
        
        // 行为树插件 ID（可选）
        string behavior_tree = 2;
    }

    message Feedback {
        // 当前位置
        autonomy.commsgs.proto.geometry_msgs.PoseStamped current_pose = 1;
        
        // 到目标的距离
        float distance_remaining = 2;
        
        // 导航状态
        string navigation_state = 3;
        
        // 已用时间
        autonomy.commsgs.proto.builtin_interfaces.Duration elapsed_time = 4;
    }

    message Result {
        // 错误码
        NavigateToPoseErrorCode error_code = 1;
        
        // 错误消息
        string error_msg = 2;
        
        // 总耗时
        autonomy.commsgs.proto.builtin_interfaces.Duration total_elapsed_time = 3;
    }

    // 必须包含这三个字段
    Goal goal = 1;
    Feedback feedback = 2;
    Result result = 3;
}
```

**注意事项**：

1. **字段编号不可重复**：Goal、Feedback、Result 内部的字段编号可以独立编号（都从 1 开始），但 Action message 本身的三个字段必须使用 1、2、3
2. **嵌套消息命名**：嵌套的 `Goal`、`Feedback`、`Result` 消息名称必须严格按照这三个名称，不能使用其他名称
3. **字段可选性**：在 proto3 中，所有字段默认都是可选的（没有 `required` 关键字）
4. **默认值**：数字类型默认为 0，字符串默认为空字符串，布尔值默认为 false
5. **枚举值**：枚举的第一个值必须为 0（用于默认值）
6. **命名空间**：在 C++ 代码中，Action 类型的完整命名空间为 `package::ActionName`，嵌套类型为 `package::ActionName::Goal` 等

**C++ 中的使用**：

定义好 proto 文件后，在 C++ 代码中可以这样使用：

```cpp
// 使用 Action 类型
using ActionT = autonomy::tasks::example::proto::NavigateToPoseAction;
using Goal = ActionT::Goal;
using Feedback = ActionT::Feedback;
using Result = ActionT::Result;

// 创建 Action Server
auto server = autolink::action::CreateServer<ActionT>(...);

// 创建 Action Client
auto client = autolink::action::CreateClient<ActionT>(node, "/navigate_to_pose");
```

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

### 3.4 Action 通信示例

Action 是一种用于长时间运行任务的通信机制，与 Service 不同，Action 支持目标执行过程中的反馈（Feedback）和可取消性。

**Action Server（服务端）：**

```cpp
#include "autolink/autolink.hpp"
#include "autolink/action/action.hpp"
#include "autonomy/commsgs/example_msgs.hpp"  // 假设有 Action 类型定义

// 假设 Action 类型为：ExampleAction，包含 Goal、Feedback、Result
using ActionT = autonomy::commsgs::example_msgs::ExampleAction;
using Goal = ActionT::Goal;
using Feedback = ActionT::Feedback;
using Result = ActionT::Result;

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);
    
    auto node = autolink::CreateNode("action_server_node");
    
    // 创建 Action Server
    auto server = autolink::action::CreateServer<ActionT>(
        node,
        "/example_action",
        // handle_goal: 决定是否接受目标
        [](const autolink::action::GoalUUID& uuid,
           std::shared_ptr<const Goal> goal) {
            AINFO << "Goal received, UUID: " 
                  << autolink::action::ToString(uuid);
            // 可以根据目标内容决定是否接受
            // 返回 REJECT、ACCEPT_AND_EXECUTE 或 ACCEPT_AND_DEFER
            return autolink::action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        // handle_cancel: 决定是否接受取消请求
        [](std::shared_ptr<autolink::action::ServerGoalHandle<ActionT>> 
           goal_handle) {
            AINFO << "Cancel request received for goal: "
                  << autolink::action::ToString(goal_handle->GetGoalId());
            // 返回 REJECT 或 ACCEPT
            return autolink::action::CancelResponse::ACCEPT;
        },
        // handle_accepted: 目标被接受时的回调，用于执行目标
        [](std::shared_ptr<autolink::action::ServerGoalHandle<ActionT>> 
           goal_handle) {
            // 在新的线程中执行目标
            std::thread([goal_handle]() {
                auto goal = goal_handle->GetGoal();
                AINFO << "Executing goal...";
                
                // 开始执行（将状态从 ACCEPTED 转为 EXECUTING）
                goal_handle->Execute();
                
                // 模拟长时间运行的任务
                for (int i = 0; i < 10; ++i) {
                    // 检查是否被取消
                    if (goal_handle->IsCanceling()) {
                        AINFO << "Goal canceled, aborting...";
                        auto result = std::make_shared<Result>();
                        result->set_status("Canceled");
                        goal_handle->Canceled(result);
                        return;
                    }
                    
                    // 发布反馈
                    auto feedback = std::make_shared<Feedback>();
                    feedback->set_progress(i * 10);
                    feedback->set_status("Processing...");
                    goal_handle->PublishFeedback(feedback);
                    
                    // 模拟工作
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                
                // 任务完成，设置结果
                auto result = std::make_shared<Result>();
                result->set_status("Succeeded");
                result->set_final_value(100);
                goal_handle->Succeed(result);
            }).detach();
        });
    
    if (!server) {
        AERROR << "Failed to create action server";
        return 1;
    }
    
    AINFO << "Action server started";
    autolink::WaitForShutdown();
    return 0;
}
```

**Action Client（客户端）：**

```cpp
#include "autolink/autolink.hpp"
#include "autolink/action/action.hpp"
#include "autonomy/commsgs/example_msgs.hpp"
#include <future>

using ActionT = autonomy::commsgs::example_msgs::ExampleAction;
using Goal = ActionT::Goal;
using Feedback = ActionT::Feedback;

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);
    
    auto node = autolink::CreateNode("action_client_node");
    
    // 创建 Action Client
    auto client = autolink::action::CreateClient<ActionT>(
        node, "/example_action");
    
    if (!client) {
        AERROR << "Failed to create action client";
        return 1;
    }
    
    // 等待服务器就绪
    while (!client->ActionServerIsReady()) {
        AINFO << "Waiting for action server...";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 创建目标
    Goal goal;
    goal.set_target_value(100);
    goal.set_timeout(30);
    
    // 发送目标（带回调选项）
    autolink::action::Client<ActionT>::SendGoalOptions options;
    
    // 目标响应回调
    options.goal_response_callback = [](
        std::shared_ptr<autolink::action::ClientGoalHandle<ActionT>> goal_handle) {
        if (goal_handle) {
            AINFO << "Goal accepted, UUID: "
                  << autolink::action::ToString(goal_handle->GetGoalId());
        } else {
            AERROR << "Goal rejected";
        }
    };
    
    // 反馈回调
    options.feedback_callback = [](
        std::shared_ptr<autolink::action::ClientGoalHandle<ActionT>> goal_handle,
        std::shared_ptr<const Feedback> feedback) {
        AINFO << "Feedback received: progress=" << feedback->progress()
              << ", status=" << feedback->status();
    };
    
    // 结果回调
    options.result_callback = [](
        const autolink::action::ClientGoalHandle<ActionT>::WrappedResult& result) {
        AINFO << "Result received, code: " << static_cast<int>(result.code);
        if (result.result) {
            AINFO << "Result status: " << result.result->status();
        }
    };
    
    // 异步发送目标
    auto goal_future = client->AsyncSendGoal(goal, options);
    auto goal_handle = goal_future.get();
    
    if (!goal_handle) {
        AERROR << "Failed to send goal";
        return 1;
    }
    
    // 等待结果
    auto result_future = client->AsyncGetResult(goal_handle);
    auto wrapped_result = result_future.get();
    
    AINFO << "Action completed with code: " 
          << static_cast<int>(wrapped_result.code);
    
    // 也可以取消目标（示例）
    // auto cancel_future = client->AsyncCancelGoal(goal_handle);
    // bool canceled = cancel_future.get();
    // AINFO << "Cancel request " << (canceled ? "accepted" : "rejected");
    
    autolink::WaitForShutdown();
    return 0;
}
```

**[说明]**

* **ActionT**：Action 类型，需要定义三个嵌套类型：`Goal`、`Feedback`、`Result`
* **action_name**：Action 名称（字符串类型），用于标识 Action
* **Server 回调函数**：
  - `handle_goal`：决定是否接受目标，返回 `GoalResponse::ACCEPT_AND_EXECUTE`、`ACCEPT_AND_DEFER` 或 `REJECT`
  - `handle_cancel`：决定是否接受取消请求，返回 `CancelResponse::ACCEPT` 或 `REJECT`
  - `handle_accepted`：目标被接受时调用，通常在这里启动新线程执行目标
* **ServerGoalHandle 方法**：
  - `GetGoal()`：获取目标消息
  - `GetGoalId()`：获取目标 UUID
  - `Execute()`：开始执行目标（状态转为 EXECUTING）
  - `PublishFeedback()`：发布反馈消息
  - `Succeed()`：标记目标成功完成（终端状态）
  - `Canceled()`：标记目标已被取消（终端状态）
  - `Abort()`：标记目标失败中止（终端状态）
  - `IsCanceling()`：检查是否收到取消请求
  - `IsActive()`：检查目标是否处于活跃状态
* **Client 方法**：
  - `AsyncSendGoal()`：异步发送目标，返回 `std::shared_future<std::shared_ptr<GoalHandle>>`
  - `AsyncGetResult()`：异步获取结果，返回 `std::shared_future<WrappedResult>`
  - `AsyncCancelGoal()`：异步请求取消目标，返回 `std::shared_future<bool>`
  - `ActionServerIsReady()`：检查 Action 服务器是否就绪
* **ClientGoalHandle 方法**：
  - `GetGoalId()`：获取目标 UUID
  - `GetStatus()`：获取当前状态
  - `AsyncGetResult()`：获取结果的 future
* **GoalStatus 枚举**：`UNKNOWN`、`ACCEPTED`、`EXECUTING`、`CANCELING`、`SUCCEEDED`、`CANCELED`、`ABORTED`
* **ResultCode 枚举**：`UNKNOWN`、`SUCCEEDED`、`CANCELED`、`ABORTED`

**提示**：
- Action 适用于需要长时间运行、可以中途取消、需要进度反馈的任务
- Server 端通常在 `handle_accepted` 回调中启动新线程执行目标，以保持非阻塞
- Client 端可以同时发送多个目标，每个目标有唯一的 UUID
- Feedback 是单向的（从 Server 到 Client），用于报告执行进度

## 4. 注意事项

1. **初始化顺序**：在使用任何 Autolink API 之前，必须先调用 `autolink::Init()`
2. **节点名称唯一性**：同一进程中的节点名称必须唯一
3. **消息类型匹配**：Writer 和 Reader 的消息类型必须完全一致
4. **Channel 名称匹配**：发布者和订阅者必须使用相同的 Channel 名称
5. **Service 名称匹配**：Service 和 Client 必须使用相同的服务名称
6. **Action 名称匹配**：Action Server 和 Client 必须使用相同的 Action 名称
7. **Action 类型定义**：Action 类型必须定义 `Goal`、`Feedback`、`Result` 三个嵌套类型
8. **线程安全**：Autolink API 通常是线程安全的，但同一对象的并发调用需要注意
9. **资源清理**：Node、Writer、Reader 等对象使用智能指针管理，通常不需要手动释放
10. **Action Server 非阻塞**：Server 端的 `handle_accepted` 回调应该启动新线程执行目标，避免阻塞

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

### 5.4 Action 与 Service 的区别

| 特性 | Service | Action |
|------|---------|--------|
| 执行时间 | 短时间（通常 < 1秒） | 长时间运行（可无限期） |
| 反馈 | 不支持 | 支持（Feedback） |
| 可取消性 | 不支持 | 支持（Cancel） |
| 响应模式 | 请求/响应（同步或异步） | 目标/反馈/结果（异步） |
| 适用场景 | 计算、查询、状态查询 | 导航、任务执行、长时间操作 |

选择建议：
- 使用 **Service** 进行快速的请求/响应操作（如查询状态、计算、设置参数）
- 使用 **Action** 进行长时间运行的任务（如导航到目标点、执行复杂任务），需要进度反馈和可取消性

## 6. Docker 运行环境配置

在宿主机通过 `run_autonomy.py` 启动开发容器时，可将常用参数写入 shell 配置文件（如 `~/.zshrc` 或 `~/.bashrc`），一次配置、长期生效：

```bash
# ~/.zshrc 示例
export AUTONOMY_ENV=/home/quandy/workspace/github/autonomy
export AUTONOMY_DATA_VOLUMES=/mnt/data4t
export AUTONOMY_CONTAINER_NAME=SpaceHero
export AUTONOMY_PORTS=8765:8765
export AUTONOMY_NETWORK=host
```

使配置生效：

```bash
source ~/.zshrc   # 或 source ~/.bashrc
```

启动容器：

```bash
cd autonomy/docker
python3 run_autonomy.py -p x86_64 -n yes
```

**环境变量说明**：

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `AUTONOMY_ENV` | 自动检测仓库根目录 | 宿主机项目路径，挂载为容器内 `/workspace/autonomy` |
| `AUTONOMY_DATA_VOLUMES` | 无 | 数据卷列表，逗号或空格分隔，如 `/mnt/data4t` |
| `AUTONOMY_CONTAINER_NAME` | `SpaceHero` | Docker 容器名称 |
| `AUTONOMY_PORTS` | `8765:8765` | 端口映射，逗号或空格分隔多个，如 `8765:8765,8080:8080` |
| `AUTONOMY_NETWORK` | `host` | Docker 网络模式（Linux 下生效，如 `host`、`bridge`） |
| `AUTONOMY_KEEP_ISAAC_ENTRYPOINT` | 无 | 设为 `1` 时保留 NVIDIA 镜像默认 ENTRYPOINT |
| `DISPLAY` | `:0` | X11 显示转发（GUI） |

进入容器：

```bash
docker exec -it SpaceHero /bin/bash
```

若修改了 `AUTONOMY_CONTAINER_NAME`，将上述命令中的容器名替换为对应值即可。
