# 5. Commsgs 模块架构设计

本文描述 `autonomy/commsgs` 的逻辑架构、双层消息模型与全栈集成关系。

## 5.1 设计目标

commsgs 模块遵循以下设计原则：

1. **ROS 语义兼容**：字段命名、坐标系约定、Stamped/WithCovariance 模式与 ROS 2 `common_msgs` 保持一致
2. **双层分离**：C++ struct 供算法零开销使用，proto 供序列化与跨进程通信
3. **显式边界转换**：`ToProto`/`FromProto` 在模块边界集中调用，便于测试与审计
4. **无 ROS 运行时依赖**：不使用 `rosidl`/`rclcpp`，Autonomy 自研通信栈
5. **可扩展**：新增消息包遵循 proto → hpp → cpp 三层模式

## 5.2 实现状态

| 组件 | 实现度 | 说明 |
|------|--------|------|
| 16 个 proto 包 | ✅ | schema 已定义 |
| C++ struct 定义 | ✅ | 主要消息类型已覆盖 |
| `ToProto`/`FromProto` | ⏳ | 核心包完整，部分 stub |
| PointCloud2 工具 | ✅ | 迭代器 + 字段转换 |
| Nav2 Action/Service | ✅ proto | C++ 层仅 `SpeedLimit` |
| 错误码体系 | ✅ proto | 含 Nav2 遗留别名 |
| 独立 CMake 目标 | ❌ | 随 `libautonomy` glob 编译 |

## 5.3 分层架构

<div class="plan-arch-diagram">

  <div class="plan-arch-layer plan-arch-app">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">应用层</span>
      <span class="plan-arch-title">Planning / Map / Control / Localization</span>
      <span class="plan-arch-sub">算法模块，使用 C++ commsgs struct</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-body-block">
        <div class="nav-body-label">典型消息</div>
        <div class="nav-chip-list">
          <span class="nav-chip">Path</span>
          <span class="nav-chip">PoseStamped</span>
          <span class="nav-chip">OccupancyGrid</span>
          <span class="nav-chip">Twist</span>
          <span class="nav-chip">Imu</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>内存中直接传递 struct / shared_ptr</span></div>

  <div class="plan-arch-layer plan-arch-server">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">消息层</span>
      <span class="plan-arch-title">autonomy::commsgs</span>
      <span class="plan-arch-sub">C++ struct + ToProto/FromProto 转换</span>
    </div>
    <div class="plan-arch-body plan-arch-body-cols">
      <div class="nav-body-block">
        <div class="nav-body-label">核心包</div>
        <div class="nav-chip-list">
          <span class="nav-chip">builtin_interfaces</span>
          <span class="nav-chip">std_msgs</span>
          <span class="nav-chip">geometry_msgs</span>
          <span class="nav-chip">sensor_msgs</span>
          <span class="nav-chip">map_msgs</span>
        </div>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">工具</div>
        <div class="nav-chip-list">
          <span class="nav-chip">PointCloud2Iterator</span>
          <span class="nav-chip">PointCloud2Modifier</span>
          <span class="nav-chip">point_field_conversion</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>ToProto() / FromProto()</span></div>

  <div class="plan-arch-layer plan-arch-plugin">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">序列化层</span>
      <span class="plan-arch-title">commsgs::proto</span>
      <span class="plan-arch-sub">protoc 生成的 google::protobuf::Message</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-body-block">
        <div class="nav-body-label">生成物</div>
        <div class="nav-chip-list">
          <span class="nav-chip">*.pb.h</span>
          <span class="nav-chip">*.pb.cc</span>
          <span class="nav-chip">SerializeToString</span>
          <span class="nav-chip">ParseFromString</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>Autolink wire format</span></div>

  <div class="plan-arch-layer plan-arch-map">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">传输层</span>
      <span class="plan-arch-title">Autolink</span>
      <span class="plan-arch-sub">Writer / Reader / Service / Action</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-body-block">
        <div class="nav-body-label">通信原语</div>
        <div class="nav-chip-list">
          <span class="nav-chip">CreateWriter&lt;T&gt;</span>
          <span class="nav-chip">CreateReader&lt;T&gt;</span>
          <span class="nav-chip">CreateService&lt;Req,Resp&gt;</span>
          <span class="nav-chip">CreateActionClient&lt;ActionT&gt;</span>
        </div>
      </div>
    </div>
  </div>

</div>

## 5.4 文件组织模式

每个消息包遵循固定三文件模式：

```
proto/<pkg>.proto          ← schema 定义（单一事实来源）
<pkg>_msgs.hpp             ← C++ struct + ToProto/FromProto 声明
<pkg>_msgs.cpp             ← 转换函数实现
```

**依赖方向**（仅向下）：

```
geometry_msgs.hpp → std_msgs.hpp → builtin_interfaces.hpp
sensor_msgs.hpp   → geometry_msgs.hpp, std_msgs.hpp
map_msgs.hpp      → geometry_msgs.hpp, std_msgs.hpp
planning_msgs.hpp → geometry_msgs.hpp, std_msgs.hpp
nav_msgs.proto    → planning_msgs.proto, geometry_msgs.proto, error_code.proto
```

## 5.5 构建集成

commsgs 无独立 CMake 目标，通过根工程统一构建：

```text
# cmake/autonomy_sources.cmake
file(GLOB_RECURSE AUTONOMY_SOURCES "autonomy/*.cpp")
file(GLOB_RECURSE ALL_PROTOS "autonomy/*.proto")

# 根 CMakeLists.txt
protoc --cpp_out=${PROJECT_BINARY_DIR} ...
add_library(autonomy SHARED ${AUTONOMY_SOURCES} ${GENERATED_PROTO_SRCS})
target_link_libraries(autonomy protobuf autolink ...)
```

安装时同步安装源 hpp 与生成 `*.pb.h` 到 `include/autonomy/commsgs/`。

## 5.6 运行时数据流

### 5.6.1 进程内 Pub/Sub

```
模块 A                    Autolink                    模块 B
  │                         │                          │
  │  Write(Imu struct)      │                          │
  ├────────────────────────►│                          │
  │                    [序列化]                         │
  │                         ├─────────────────────────►│
  │                         │              callback(shared_ptr<Imu>)
```

Autolink 在 Writer/Reader 边界自动处理序列化，应用代码始终操作 C++ struct。

### 5.6.2 Service / Action

```
Client (proto)  ──request──►  Service (proto)
                                    │
                              FromProto()
                                    │
                              C++ struct 业务逻辑
                                    │
                              ToProto()
                                    │
Client (proto)  ◄─response──  Service (proto)
```

### 5.6.3 跨模块传递（无网络）

同一进程内模块间可直接传递 C++ struct 引用或 `shared_ptr`，无需 proto 转换：

```cpp
// system::Autonomy 内部
controller_->SetPlan(path);  // planning_msgs::Path，无序列化
```

## 5.7 消费模块矩阵

| 模块 | 使用的 commsgs 包 | 典型消息 |
|------|------------------|----------|
| `planning` | `geometry_msgs`, `planning_msgs`, `proto::nav_msgs` | `Path`, `IsPathValid` |
| `map/costmap_2d` | `geometry_msgs`, `map_msgs`, `sensor_msgs` | `OccupancyGrid`, `PointCloud2` |
| `control` | `geometry_msgs`, `planning_msgs`, `builtin_interfaces` | `Twist`, `Path`, `Pose` |
| `transform` | `geometry_msgs`, `builtin_interfaces` | `TransformStamped` |
| `localization` | `geometry_msgs`, `sensor_msgs` | `PoseWithCovariance`, `Imu` |
| `visualization` | 多种 commsgs | `Marker`, `Path`, `OccupancyGrid` |
| `sensor` | `sensor_msgs` | `Imu`, `LaserScan`, `Image` |
| `bridge` | 全部 | ROS ↔ Autonomy 转换 |

## 5.8 扩展点

| 扩展场景 | 步骤 |
|----------|------|
| 新增消息类型 | proto → hpp struct → cpp 转换 → 重编译 |
| 新增消息包 | 新建三文件 + 在消费模块 include |
| 新增 Action | 在 `nav_msgs.proto` 添加 Goal/Feedback/Result |
| 新增错误码 | 在 `error_code.proto` 对应模块段添加枚举值 |
| ROS 桥接 | 在 `bridge` 模块实现 ROS msg ↔ commsgs 转换 |

## 5.9 与 ROS 2 架构对比

| 维度 | ROS 2 | Autonomy commsgs |
|------|-------|------------------|
| 消息定义 | `.msg` / `.idl` → rosidl 生成 | 手写 hpp + proto |
| 序列化 | CDR / Fast-DDS | Protocol Buffers |
| 运行时 | rclcpp | Autolink |
| 类型系统 | 单一生成类型 | C++ struct + proto 双层 |
| 扩展 | 新 .msg 文件 | proto + hpp + cpp 三文件 |
