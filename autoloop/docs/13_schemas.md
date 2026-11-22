# 13 Schema 与命名

机器链路用 Protobuf；清单 / UI / 事件用 JSON(L)。通道字节只在 MCAP。

## `autoloop.common.VersionTuple`

| 字段 | 类型 | 约束 |
|------|------|------|
| `policy_version` | string | 非空才 admitted；禁 `latest` |
| `env_version` | string | 地图+标定 |
| `git_sha` | string | 二进制 |
| `seed` | string | 回归 |

## `autoloop.common.DualClock`

| 字段 | 含义 |
|------|------|
| `mono_ns` | 单调轴 |
| `utc_ns` | 墙钟 |

recorder `DualClock` 另有 `time_source`；映射见 [06](06_evidence_chain.md)。

## `autoloop.common.EpisodeSidecar`

| 字段 | 用途 |
|------|------|
| `episode_id` | 主键 |
| `version` | 四元组 |
| `start` / `end` | DualClock |
| `admitted` | 训练 / 正式回放闸 |
| `missing_fields` | 空字段名 |
| `robot_id` | 机号 |
| `priority` | P0–P3（int32） |
| `content_sha256` | MCAP 文件哈希 |

## `autoloop.common.TraceMetadata`

`trace_id` / `span_id` / `parent_span_id` / `robot_id` / `episode_id` / `mono_ns` / `utc_ns`。

## `autoloop.common.DataAgentHealth`

`robot_id`、`latch_set`、`ring_samples`、`snapshots_written`、`upload_pending`、`dropped`、`last_heartbeat_utc_ns`、`last_error`。

## 事件 JSONL

一行一事，禁止二进制 payload。

| 字段 | 用途 |
|------|------|
| `ts_mono_ns` / `ts_utc_ns` | 双时钟；回放只用 mono |
| `trace_id` / `span_id` | 回溯 |
| `event_code` | heartbeat / snapshot / upload / latch / drop / crash / model_rollback |
| `severity` | 仪表密度 |
| `robot_id` / `episode_id` | 主键 |
| `payload` | JSON object |

日志前缀：`[ts_utc][ts_mono][trace_id][file:line]`。

## 转换 meta

MCAP → LeRobot 后 `info.json` / parquet schema 必须含 `episode_id` 与四元组，否则不得进 `lerobot_builder`。preset：`teleop_imitation` / `navigation_imitation` / `perception_only`。通道：odom / cmd / image / state。时钟：单调轴。

recorder 边侧 `convert` 可只写 JSONL；云端 `autoloop-convert` 走 parquet + 可选视频。

## 命名

- Include：`#include "common/..."`, `"robot/..."`, `"cloud/..."`（`AUTOLOOP_SRC_ROOT`）。
- 目标：`autoloop_<domain>_<unit>`；测试 `<unit>_test.cpp`。
- Proto：`autoloop.common`；recorder：`autoloop.robot.recorder.proto`。
- YAML 蛇形；C++ PascalCase。
- 归档：`archive/{robot_id}/{episode_id}/`。
