(commsgs-schema)=
# 3. 消息模型与 Schema 设计

> 各消息包字段详解见 [§6 核心消息](06_core_msgs.md)、[§7 传感器与地图](07_sensor_map_msgs.md)、[§8 导航与规划](08_nav_planning_msgs.md)；与 ROS 对照见 [§9 综述](09_survey.md)。

### 3.1 双层架构

commsgs 采用 **C++ struct（内存层）+ Protocol Buffers（序列化层）** 分离设计：

```
┌─────────────────────────────────────────────────────────────┐
│  应用代码（planning / map / control / transform …）            │
│  使用 C++ struct: commsgs::geometry_msgs::PoseStamped        │
└──────────────────────────┬──────────────────────────────────┘
                           │ ToProto() / FromProto()
┌──────────────────────────▼──────────────────────────────────┐
│  生成代码: commsgs::proto::geometry_msgs::PoseStamped         │
│  (google::protobuf::Message 子类)                           │
└──────────────────────────┬──────────────────────────────────┘
                           │ SerializeToString / ParseFromString
┌──────────────────────────▼──────────────────────────────────┐
│  Autolink / gRPC / 跨进程网络传输                              │
└─────────────────────────────────────────────────────────────┘
```

**设计动机**

| 层次 | 优势 |
|------|------|
| C++ struct | 算法代码无 protobuf 运行时开销；可直接使用 STL；IDE 友好 |
| Proto | 跨语言、版本化 schema、高效二进制序列化 |
| 显式转换 | 边界清晰，便于测试往返一致性 |

### 3.2 命名空间镜像

| C++ | Proto |
|-----|-------|
| `autonomy::commsgs::geometry_msgs::Pose` | `autonomy.commsgs.proto.geometry_msgs.Pose` |
| `autonomy::commsgs::std_msgs::Header` | `autonomy.commsgs.proto.std_msgs.Header` |
| `autonomy::commsgs::proto::nav_msgs::NavigateToPoseAction` | 同左（仅 proto 层） |

每个消息包在 hpp 中声明、cpp 中实现转换函数：

```cpp
proto::geometry_msgs::PoseStamped ToProto(const PoseStamped& data);
PoseStamped FromProto(const proto::geometry_msgs::PoseStamped& proto);
```

嵌套消息递归调用子消息的 `ToProto`/`FromProto`。

### 3.3 Stamped 消息模式

几乎所有时序/空间数据遵循 ROS 约定：

```cpp
struct XxxStamped {
    std_msgs::Header header;   // stamp + frame_id
    Xxx xxx;
};
```

| 字段 | 语义 |
|------|------|
| `header.stamp` | 数据采集或消息生成时间 |
| `header.frame_id` | 数据所在的坐标系名称（TF frame） |

**扩展：`child_frame_id`**

`TransformStamped`、`Odometry` 等除 `header.frame_id`（父坐标系）外，还有 `child_frame_id`（子坐标系）：

```cpp
struct TransformStamped {
    std_msgs::Header header;       // frame_id = 父坐标系
    std::string child_frame_id;    // 子坐标系
    geometry_msgs::Transform transform;
};
```

### 3.4 WithCovariance 模式

带不确定性的估计量使用行主序协方差矩阵：

```cpp
struct PoseWithCovariance {
    Pose pose;
    std::vector<double> covariance;  // 6×6，顺序 (x,y,z,roll,pitch,yaw)
};
```

| 消息类型 | 协方差维度 |
|----------|-----------|
| `PoseWithCovariance` | 6×6 |
| `TwistWithCovariance` | 6×6 |
| `AccelWithCovariance` | 6×6 |
| `Imu`（sensor_msgs） | 方向 3×3、角速度 3×3、线加速度 3×3 |

### 3.5 时间与时长

#### Time

```cpp
struct Time {
    int32 sec;       // 秒（Unix epoch）
    uint32 nanosec;  // 纳秒 [0, 1e9)
};
```

- `Time::Now()` — 基于 `std::chrono::system_clock`
- `ToUnixTimeNanos()` — 合并为 64 位纳秒时间戳
- 支持比较运算符 `<` `<=` `>` `>=` `==` `!=`

#### Duration

```cpp
struct Duration {
    int64_t duration_;  // 内部存储纳秒
};
```

- 支持与 `std::chrono` 互转
- 算术运算带溢出/下溢检查
- `operator-(Time, Time)` 返回 `Duration`

### 3.6 Smart Pointer 宏

多数消息 struct 通过 `AUTONOMY_SMART_PTR_DEFINITIONS(Type)` 生成 ROS 2 风格智能指针别名：

```cpp
struct Path {
    AUTONOMY_SMART_PTR_DEFINITIONS(Path)
    // ...
};

// 生成：Path::SharedPtr, Path::ConstSharedPtr, Path::UniquePtr, make_shared<Path>()
```

便于 Autolink 回调使用 `std::shared_ptr<const T>`。

### 3.7 Proto 编译与安装

根 `CMakeLists.txt` 通过 `protoc` 编译所有 `autonomy/*.proto`：

```text
protoc --cpp_out=${PROJECT_BINARY_DIR} -I ${PROJECT_SOURCE_DIR} ${ABS_FIL}
```

生成物路径：`${BUILD_DIR}/autonomy/commsgs/proto/<name>.pb.h`

安装到：`include/autonomy/commsgs/proto/`

### 3.8 Action / Service Schema

Nav2 风格 Action 在 `nav_msgs.proto` 中定义为嵌套 message：

```protobuf
message NavigateToPoseAction {
  message Goal { ... }
  message Feedback { ... }
  message Result {
    error_code.ErrorCode error_code = 1;
    string error_msg = 2;
  }
}
```

Service 同理（如 `IsPathValid`、`ClearEntireCostmap`），请求/响应作为嵌套 message。

### 3.9 错误码分段

`error_code.proto` 采用 Apollo 风格按模块分段：

| 范围 | 模块 |
|------|------|
| `0` | OK |
| `1000–1099` | Control |
| `3000–3099` | Localization / TF |
| `4000–4599` | Recovery |
| `6000–6299` | Planning / Smoother |
| `7000–7099` | Map |
| `8000–8099` | Routing |
| `9000–9299` | Task / Waypoints |

同时保留 Nav2 / MBF 遗留别名（如 `GET_PATH_NO_PATH_FOUND = 6007`）。

### 3.10 字段类型映射

| ROS / C++ | Proto |
|-----------|-------|
| `float` / `double` | `float` / `double` |
| `int8`–`int64`, `uint8`–`uint64` | 对应整数类型 |
| `std::string` | `string` |
| `std::vector<T>` | `repeated T` |
| `std::vector<float>` 协方差 | `repeated float [packed=true]` |
| 嵌套 struct | 嵌套 `message` |

### 3.11 已知 Schema 不一致

| 问题 | 说明 |
|------|------|
| `builtin_interfaces.proto` 的 `Duration` | proto 含 `stamp` + `frame_id`（类似 Header），C++ `Duration` 为纯纳秒；转换时将纳秒拆为 sec/nanosec 写入 proto |
| `shape_msgs.hpp` 命名空间 | struct 误放在 `builtin_interfaces` 而非 `shape_msgs` |
| 部分 `ToProto` stub | 返回空 proto 或遗漏字段复制，见 [§9.2](09_survey.md#92-实现完整度矩阵) |
