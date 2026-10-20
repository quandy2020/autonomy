(commsgs-schema)=
# 3. 消息模型与 Schema

查 Header/Stamped/协方差约定，或 Action、error_code 分段。

struct 与 proto 成对；大包体联调前实测。

### 双层模型

```text
C++ struct ──ToProto/FromProto──► proto ──Serialize──► Autolink / 网络
```

命名：`autonomy::commsgs::geometry_msgs::Pose` ↔ `autonomy.commsgs.proto.geometry_msgs.Pose`。

### 约定

| 模式 | 结构 |
|------|------|
| Stamped | Header + payload |
| TF | 父=`header.frame_id`，子=`child_frame_id` |
| WithCovariance | pose/twist + 36 元协方差 |
| SmartPtr | `AUTONOMY_SMART_PTR_DEFINITIONS` |

### Time / Duration

`Time`：sec + nanosec，`Now()`。`Duration`：纳秒，与 chrono 互转。

### Action / error_code

Action：Goal / Feedback / Result（含 error_code）。分段见下表。

| 范围 | 模块 |
|------|------|
| 0 | OK |
| 1000–1099 | Control |
| 3000–3099 | Localization/TF |
| 6000–6299 | Planning |
| 7000–7099 | Map |
| 9000–9299 | Task |

完整枚举：`error_code.proto`。

### 已知问题

Duration proto 与 C++ 拆分方式；shape_msgs 命名空间；部分 ToProto stub — [§9](09_survey.md)。

字段分册：[§6–§8](06_core_msgs.md)。

### 延伸阅读

[§4 用法](04_usage.md)
