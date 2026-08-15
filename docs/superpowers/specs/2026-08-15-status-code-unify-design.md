# Status 码表长期统一（方案 A）

## 目标

单一错误码枚举：`automsgs.msgs.status_msgs.StatusCode`。  
`automsgs.rpcs.common.Status` 的 `code` 字段类型改为 `StatusCode`；删除 `rpcs.common.Code`。

## 命名约定

**不加全局 `CODE_` 前缀**（类型名 `StatusCode` 即命名空间）。

| 类别 | 规则 | 示例 |
|------|------|------|
| 成功/未知 | 无域前缀 | `OK`、`UNKNOWN` |
| 通用 RPC | Google 风格语义名 | `INVALID_ARGUMENT`、`NOT_FOUND` |
| 域错误 | `<DOMAIN>_` | `NAVIGATION_BUSY`、`TELEOP_COLLISION`、`CONTROL_ERROR` |
| 与模块码同名冲突 | 换语义名 + 分段数值 | `LOCALIZATION_UNAVAILABLE=801` vs `LOCALIZATION_NOT_READY=3002` |
| 遗留别名 | 保持旧名（`allow_alias`） | `MOVE_BASE_*`、`SPIN_NONE` |

结构整理（薄/细/DEPRECATED、分阶段去别名）见
`2026-08-15-status-code-structure-design.md`。取消拼写：`CANCELLED`；
`TASK_CANCELED`（9003）规范名为 `TASK_BT_CANCELLED`。

## Breaking

| 项 | 旧 RPC | 新 |
|----|--------|-----|
| 成功 | `code == 1` (`CODE_OK`) | `code == OK`（**0**） |
| 未设置/未知 | `code == 0` (`CODE_UNKNOWN`) | `UNKNOWN == 1` |
| 系统码 | `1000–1003` | **`2000–2003`**（避开 CONTROL 1000+） |

## 保留

- `Status { code, message }` 消息名与字段名（第三方 JSON 键名不变，仅成功数值变）
- `StatusPb { error_code, msg }` 仍给内部 msgs；与 RPC 共用 `StatusCode`

## SYSTEM 重编号

| 旧 | 新 |
|----|-----|
| `CODE_SYSTEM_NOT_READY = 1000` | `SYSTEM_NOT_READY = 2000` |
| `CODE_SYSTEM_ESTOP = 1001` | `SYSTEM_ESTOP = 2001` |
| `CODE_SYSTEM_HARDWARE_FAULT = 1002` | `SYSTEM_HARDWARE_FAULT = 2002` |
| `CODE_SYSTEM_ESTOP_CLEAR_DENIED = 1003` | `SYSTEM_ESTOP_CLEAR_DENIED = 2003` |

其余原 `Code` 数值并入 `StatusCode`（2–99 通用，100+ 各 RPC 域）。
