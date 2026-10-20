(commsgs-schema)=
# 3. 消息模型与 Schema

字段详解：[§6–§8](06_core_msgs.md)；ROS 对照：[§9](09_survey.md)。

### 3.1 双层模型

```
应用 C++ struct ──ToProto/FromProto──► proto Message ──Serialize──► Autolink/网络
```

| 层 | 作用 |
|----|------|
| C++ struct | 算法无 protobuf 开销 |
| Proto | 跨语言、版本化、二进制传输 |

命名：`autonomy::commsgs::geometry_msgs::Pose` ↔ `autonomy.commsgs.proto.geometry_msgs.Pose`。

### 3.2 约定模式

| 模式 | 结构 |
|------|------|
| Stamped | `Header`（stamp + frame_id）+ payload |
| child_frame_id | TransformStamped / Odometry：父=`header.frame_id`，子=`child_frame_id` |
| WithCovariance | `pose` + `vector<double>` 6×6 行主序 |
| SmartPtr | `AUTONOMY_SMART_PTR_DEFINITIONS(T)` → SharedPtr 等 |

### 3.3 Time / Duration

| 类型 | 存储 | API |
|------|------|-----|
| `Time` | sec + nanosec | `Now()`、`ToUnixTimeNanos()`、比较运算 |
| `Duration` | int64 纳秒 | 与 `std::chrono` 互转；`Time-Time` |

### 3.4 Proto 构建与 Action

`protoc` → `${BUILD}/autonomy/commsgs/proto/*.pb.h`。Action：`Goal`/`Feedback`/`Result`（含 `error_code`）。

### 3.5 错误码分段

| 范围 | 模块 |
|------|------|
| 0 | OK |
| 1000–1099 | Control |
| 3000–3099 | Localization/TF |
| 4000–4599 | Recovery |
| 6000–6299 | Planning |
| 7000–7099 | Map |
| 8000–8099 | Routing |
| 9000–9299 | Task |

### 3.6 类型映射与已知问题

| C++ | Proto |
|-----|-------|
| 标量 / string / vector | 对应 / string / repeated |
| 协方差 vector\<float\> | packed repeated |

| 问题 | 说明 |
|------|------|
| Duration proto | 形似 Header；转换时拆纳秒 |
| shape_msgs 命名空间 | hpp 误放 builtin_interfaces |
| 部分 ToProto | stub，见 [§9.2](09_survey.md) |

### 3.7 参见
