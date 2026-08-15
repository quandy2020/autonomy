# StatusCode 结构整理（分区 + 分阶段去别名）

## 目标

让 `automsgs.msgs.status_msgs.StatusCode` **言简意赅、见名知意、易于维护、不冗余**：

- 单枚举继续服务 `StatusPb.error_code` 与 `rpcs.common.Status.code`
- 结构上分清 **薄码（对外）** / **细码（对内）** / **DEPRECATED 别名**
- 长期删掉 Nav2/MBF 遗留别名（分阶段 A）

## 决策摘要

| 项 | 选择 |
|----|------|
| 遗留别名 | 分阶段删除（A0 隔离 → A1 迁调用 → A2 删别名） |
| RPC vs 模块 | 同一 enum，约定分层（不物理拆两个 enum） |
| 薄/细语义重叠 | 保留两套数值；只统一拼写 |
| 落地形态 | α：一文件三段重排 + 命名卫生；本阶段不改调用方、不改数值 |

## 文件结构

单文件 `status_msgs.proto`，`enum StatusCode` 固定三段：

1. **薄码（对外）** — `OK` / `UNKNOWN` / 通用 2–99 / 各 RPC 域
2. **细码（对内）** — `CONTROL_*`、`PLANNING_*`、`LOCALIZATION_*`（3000+）、`MAP_*`（7000+）、`TASK_*`（9000+）等
3. **DEPRECATED 别名** — `MOVE_BASE_*`、`FOLLOW_PATH_*`、`COMPUTE_PATH_*`…；禁止新增

`option allow_alias = true` 在别名清零前保留。

## 命名规则

- 不加全局 `CODE_` / `RPC_`
- 域前缀：`<DOMAIN>_<语义>`
- 取消拼写统一为 **`CANCELLED`**
- 同前缀冲突：靠语义差 + 数值段（如 `LOCALIZATION_UNAVAILABLE=801` vs `LOCALIZATION_NOT_READY=3002`）
- 使用约定：
  - `rpcs.common.Status.code` → 只填薄码
  - `StatusPb.error_code` → 可填细码；出站 fine→thin 映射属适配层（A3，本阶段只定规则）

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
| 同数值别名 | DEPRECATED | Nav2/MBF |

1100–1399、2000 插在 CONTROL 与 LOCALIZATION 细码之间为历史布局；本阶段不搬号，靠注释标明薄/细。

## 拼写与冲突名（A0）

| 现状 | 规范名 | 说明 |
|------|--------|------|
| 薄码 `*_CANCELLED` | 不变 | — |
| `CONTROL_CANCELED=1010` | `CONTROL_CANCELLED` | 旧名留 deprecated 别名 |
| `RECOVERY_CANCELED=4001` | `RECOVERY_CANCELLED` | 同上 |
| `PLANNING_CANCELED=6002` | `PLANNING_CANCELLED` | 同上 |
| `TASK_CANCELED=9003` | `TASK_BT_CANCELLED` | 不能改成 `TASK_CANCELLED`（与薄码 302 同名异值） |

同前缀跨层（保留，本阶段不改前缀）：

- `TASK_*=300–303`（任务面）vs `TASK_*=9000+`（BT）
- `MAP_*=600–603`（存储 API）vs `MAP_*=7000+`（costmap）
- `LOCALIZATION_*=800–803` vs `3000+`

若日后仍易混，另开变更给薄码换前缀（如任务面 → `JOB_*`）。

## 阶段划分

| 阶段 | 内容 | 破坏性 |
|------|------|--------|
| **A0** | 三段重排、分层注释、上述改名、README/本 spec 同步；别名保留 | 规范名替换处有 C++ 符号更名（旧名仍可用） |
| **A1** | `autonomy/` 别名调用 → 规范细码 | 调用方 diff |
| **A2** | 删除 DEPRECATED 段；无别名则关 `allow_alias` | 依赖旧符号的外部代码需已迁完 |
| **A3** | 可选：文档/实现 fine→thin 出站映射表 | 适配层 |

## 非目标（A0）

- 不改任何码的数值
- 不实现 fine→thin 网关
- 不迁移 `FOLLOW_PATH_*` 等调用方
- 不物理拆成两个 enum

## 相关文档

- `docs/superpowers/specs/2026-08-15-status-code-unify-design.md`（码表统一进 `StatusCode`）
- `automsgs/proto/rpcs/README.md`（RPC 薄码约定）
