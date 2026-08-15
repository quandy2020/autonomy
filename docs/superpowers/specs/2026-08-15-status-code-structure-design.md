# StatusCode 结构整理（分区 + 分阶段去别名）

## 目标

让 `automsgs.msgs.status_msgs.StatusCode` **言简意赅、见名知意、易于维护、不冗余**：

- 单枚举继续服务 `StatusPb.error_code` 与 `rpcs.common.Status.code`
- 结构上分清 **薄码（对外）** / **细码（对内）**
- Nav2/MBF 遗留别名已删除（A0→A1→A2）

## 决策摘要

| 项 | 选择 |
|----|------|
| 遗留别名 | 分阶段删除（A0 隔离 → A1 迁调用 → A2 删别名）✅ |
| RPC vs 模块 | 同一 enum，约定分层（不物理拆两个 enum） |
| 薄/细语义重叠 | 保留两套数值；只统一拼写 |
| 落地形态 | α：一文件分区重排 + 命名卫生；不改数值 |

## 文件结构（当前）

单文件 `status_msgs.proto`，`enum StatusCode` 两段：

1. **薄码（对外）** — `OK` / `UNKNOWN` / 通用 2–99 / 各 RPC 域
2. **细码（对内）** — `CONTROL_*`、`PLANNING_*`、`LOCALIZATION_*`（3000+）、`MAP_*`（7000+）、`TASK_*`（9000+）等

无 `allow_alias`（别名已清零）。

## 命名规则

- 不加全局 `CODE_` / `RPC_`
- 域前缀：`<DOMAIN>_<语义>`
- 取消拼写统一为 **`CANCELLED`**
- 同前缀冲突：靠语义差 + 数值段（如 `LOCALIZATION_UNAVAILABLE=801` vs `LOCALIZATION_NOT_READY=3002`）
- 使用约定：
  - `rpcs.common.Status.code` → 只填薄码
  - `StatusPb.error_code` → 可填细码；出站 fine→thin 映射属适配层（A3，可选）

## 数值分段（不重编号）

| 段 | 层 | 内容 |
|----|----|------|
| 0–1 | 薄 | `OK` / `UNKNOWN` |
| 2–99 | 薄 | 通用 |
| 100–199 | 薄 | `NAVIGATION_*` |
| 200–299 | 薄 | `VEHICLE_*` |
| 300–399 | 薄 | 任务面 `TASK_*` |
| 400–499 | 薄 | `SPEECH_*` |
| 500–599 | 薄 | `CHARGING_*` |
| 600–699 | 薄 | 地图存储 `MAP_*` |
| 700–799 | 薄 | `SENSOR_*` |
| 800–899 | 薄 | 定位 RPC `LOCALIZATION_*` |
| 900–999 | 薄 | `FOLLOW_*` |
| 1000–1099 | 细 | `CONTROL_*` |
| 1100–1199 | 薄 | `MAPPING_*` |
| 1200–1299 | 薄 | `TELEOP_*` |
| 1300–1399 | 薄 | `EXPLORATION_*` |
| 2000–2099 | 薄 | `SYSTEM_*` |
| 3000+ / 4000+ / 4500+ / 6000+ / 6200+ / 7000+ / 8000+ / 9000+ / 9200+ | 细 | 模块诊断 |

1100–1399、2000 插在 CONTROL 与 LOCALIZATION 细码之间为历史布局；靠注释标明薄/细。

## 拼写（已落地）

| 规范名 | 数值 | 说明 |
|--------|------|------|
| `*_CANCELLED` | 各域 | 统一拼写 |
| `TASK_BT_CANCELLED` | 9003 | BT 任务取消；不可与薄码 `TASK_CANCELLED=302` 同名 |

同前缀跨层（保留）：

- `TASK_*=300–303`（任务面）vs `TASK_*=9000+`（BT）
- `MAP_*=600–603`（存储 API）vs `MAP_*=7000+`（costmap）
- `LOCALIZATION_*=800–803` vs `3000+`

若日后仍易混，另开变更给薄码换前缀（如任务面 → `JOB_*`）。

## 阶段划分

| 阶段 | 内容 | 状态 |
|------|------|------|
| **A0** | 分区重排、分层注释、CANCELLED/`TASK_BT_CANCELLED`、文档同步 | ✅ |
| **A1** | `autonomy/` 别名 → 规范细码（controller/planner autolink actions） | ✅ |
| **A2** | 删除 DEPRECATED 段；关闭 `allow_alias` | ✅ |
| **A3** | fine→thin 出站映射（`status_code_map.hpp` + 本文档） | ✅ |

## A3：fine → thin 出站映射

适配层在把内部 `StatusPb.error_code` 填入 `rpcs.common.Status.code` 时调用：

```cpp
#include <automsgs/msgs/status_msgs/status_code_map.hpp>
status.set_code(automsgs::msgs::status_msgs::ToThinStatusCode(fine));
```

规则摘要：

| 细码域 | 薄码落点（典型） |
|--------|------------------|
| 已是薄码 | 原样返回 |
| `CONTROL_*` | `NAVIGATION_*` / `SYSTEM_ESTOP` / `UNAVAILABLE` / `INTERNAL` |
| `LOCALIZATION_*`（3000+） | `LOCALIZATION_UNAVAILABLE` / `LOCALIZATION_LOST` |
| `RECOVERY_*` | `NAVIGATION_*` / `INTERNAL` |
| 行为原语 4500+ | `TELEOP_*` / `INVALID_ARGUMENT` |
| `PLANNING_*` / `SMOOTHER_*` / `ROUTING_*` / `WAYPOINTS_*` | `NAVIGATION_*` / `UNAVAILABLE` / `INTERNAL` |
| `MAP_*`（7000+） | `MAP_INVALID` / `MAP_LOAD_FAILED` / `UNAVAILABLE` |
| `TASK_*`（9000+） | `TASK_CANCELLED` / `TASK_FAILED` / `NAVIGATION_*` / `DEADLINE_EXCEEDED` |
| 未列出的细码 | `INTERNAL` |

实现：`automsgs/core/include/automsgs/msgs/status_msgs/status_code_map.hpp`。

## 相关文档

- `docs/superpowers/specs/2026-08-15-status-code-unify-design.md`（码表统一进 `StatusCode`）
- `automsgs/proto/rpcs/README.md`（RPC 薄码约定）
